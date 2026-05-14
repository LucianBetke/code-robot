"""
robot_monitor.py
=================
Serial-Monitor fuer das Roboter-System (vorne + hinten).

Verbinde dich mit dem COM-Port des VORNE-Arduino (USB).

Aktuelle Log-Struktur:

  Nutz-/Protokolldaten ohne '#':
    PING / ACK / KA
    VSOL,<frameId>,<hiLiSoll>,<hiReSoll>
    VSOL_OK,<frameId>
    VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>

  Debug-/Messdaten mit '#':
    #INFO,...
    #EVENT,startCmd,param=2.00,durationMs=2000
    #HDR,WHEELS,ms,VoLi_s,VoLi_i,VoLi_pwm,...
    #WHEELS,0,0.30,0.00,0,...
    #CHASSIS,...   optional spaeter

Dieses Programm wertet aus:
  #HDR
  #WHEELS
  #CHASSIS
  #EVENT

UART-Nutzdaten wie VSOL, VSOL_OK, VIST, KA, PING, ACK werden im Terminal angezeigt,
aber nicht als Messdaten geplottet.

Abhaengigkeiten:
  pip install pyserial matplotlib
"""

import sys
import csv
import time
import threading
import argparse
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")          # ggf. auf "Qt5Agg" aendern
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ─────────────────────────────────────────────
# Einstellungen
# ─────────────────────────────────────────────

DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
MAX_POINTS = 3000
UPDATE_MS = 200


# ─────────────────────────────────────────────
# Gemeinsamer Datenpuffer
# ─────────────────────────────────────────────

class Store:
    def __init__(self):
        self.lock = threading.Lock()

        self.ready = False
        self.mode = None              # "WHEELS" oder "CHASSIS"

        self.raw_cols = []            # Spalten vom Arduino-Header
        self.cols = []                # CSV-/Puffer-Spalten inkl. Zusatzspalten
        self.bufs = {}

        self._csv_file = None
        self._csv_writer = None

        # Zusatzinformationen fuer mehrere CMDT-Befehle
        self.cmd_index = 0
        self.current_offset_ms = 0.0
        self.current_duration_ms = 0.0
        self.sample_index = 0

    def init_cols(self, mode: str, raw_cols):
        mode = mode.upper()

        with self.lock:
            self.mode = mode
            self.raw_cols = list(raw_cols)

            # Zusatzspalten:
            # sample     = laufender Messpunktzaehler innerhalb des aktuellen Laufs
            # cmd_index  = welcher CMDT-Befehl innerhalb des aktuellen Laufs
            # t_plot_ms  = fortlaufende Zeit innerhalb des aktuellen Script-Laufs
            self.cols = ["sample", "cmd_index", "t_plot_ms"] + self.raw_cols

            # WICHTIG:
            # Bei jeder neuen #HDR-Zeile wird ein neuer Arduino-/Script-Lauf angenommen.
            # Deshalb werden Plotpuffer, Zeitbasis und CMDT-Zaehler zurueckgesetzt.
            self.bufs = {c: deque(maxlen=MAX_POINTS) for c in self.cols}
            self.ready = True

            self.cmd_index = 0
            self.current_offset_ms = 0.0
            self.current_duration_ms = 0.0
            self.sample_index = 0

        print(f"[Mode] {self.mode}")
        print(f"[Header] {self.cols}")
        print("[Run] neuer Lauf, Zeitzaehler auf 0 gesetzt")

    def open_csv(self, path):
        with self.lock:
            if self._csv_writer is not None:
                return

            self._csv_file = open(path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.DictWriter(
                self._csv_file,
                fieldnames=self.cols,
                extrasaction="ignore"
            )
            self._csv_writer.writeheader()

        print(f"[CSV] Speichere nach: {path}")

    def close_csv(self):
        with self.lock:
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
                self._csv_writer = None

    def start_command(self, duration_ms: float):
        with self.lock:
            if self.cmd_index > 0:
                self.current_offset_ms += self.current_duration_ms

            self.cmd_index += 1
            self.current_duration_ms = duration_ms

        print(f"[CMD] start #{self.cmd_index}, durationMs={duration_ms:.0f}")

    def push_values(self, mode: str, values):
        mode = mode.upper()

        with self.lock:
            if not self.ready:
                return

            if mode != self.mode:
                print(f"[Warnung] Datenmodus {mode} passt nicht zu Header {self.mode}")
                return

            if len(values) != len(self.raw_cols):
                print(f"[Laenge?] erwartet {len(self.raw_cols)}, bekommen {len(values)}")
                return

            row_raw = dict(zip(self.raw_cols, values))

            ms = row_raw.get("ms", 0.0)
            t_plot_ms = self.current_offset_ms + ms

            self.sample_index += 1

            row = {
                "sample": self.sample_index,
                "cmd_index": self.cmd_index,
                "t_plot_ms": t_plot_ms,
            }

            row.update(row_raw)

            for k, v in row.items():
                if k in self.bufs:
                    self.bufs[k].append(v)

            if self._csv_writer:
                self._csv_writer.writerow(row)

    def snapshot(self):
        with self.lock:
            return {c: list(v) for c, v in self.bufs.items()}, self.mode


store = Store()


# ─────────────────────────────────────────────
# Parser-Hilfsfunktionen
# ─────────────────────────────────────────────

IGNORE_PREFIXES = (
    "PING",
    "PONG",
    "ACK",
    "KA",
    "VSOL",
    "VSOL_OK",
    "VIST",
)


def parse_key_value_parts(parts):
    result = {}

    for part in parts:
        if "=" not in part:
            continue

        key, value = part.split("=", 1)
        result[key.strip()] = value.strip()

    return result


def parse_float_list(parts):
    return [float(p.strip()) for p in parts]


# ─────────────────────────────────────────────
# Serial-Thread
# ─────────────────────────────────────────────

def serial_thread(port: str, baud: int, csv_path: str):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[Fehler] Port '{port}' nicht geoeffnet: {e}")
        print("Verfuegbare Ports:")

        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")

        sys.exit(1)

    print(f"[Serial] {port} @ {baud} baud - warte auf Daten ...")
    time.sleep(2)

    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("[Serial] Verbindung verloren.")
            break

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()

        if not line:
            continue

        # UART-/Nutztelegramme ignorieren, aber im Terminal zeigen.
        if any(line.startswith(p) for p in IGNORE_PREFIXES):
            print(f"[uart]  {line}")
            continue

        # Nur Debug-/Messzeilen beginnen mit '#'
        if not line.startswith("#"):
            print(f"[?] {line}")
            continue

        content = line[1:]
        parts = [p.strip() for p in content.split(",")]

        if not parts:
            continue

        tag = parts[0].upper()

        # ----------------------------------------------------
        # Neuer Header:
        # #HDR,WHEELS,ms,...
        # #HDR,CHASSIS,ms,...
        #
        # WICHTIG:
        # Jeder neue Header startet im Python-Monitor einen neuen Lauf.
        # Dadurch wird die Zeitachse nach erneutem Schliessen des Schalters
        # wieder auf 0 gesetzt.
        # ----------------------------------------------------

        if tag == "HDR":
            if len(parts) < 3:
                print(f"[Header?] {line}")
                continue

            mode = parts[1].upper()
            cols = parts[2:]

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        # ----------------------------------------------------
        # Rueckwaertskompatibilitaet fuer alte Version:
        # #ms,...
        # ----------------------------------------------------

        if tag == "MS":
            cols = parts
            mode = "CHASSIS" if "vx_i" in cols else "WHEELS"

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        # ----------------------------------------------------
        # Events:
        # #EVENT,startCmd,param=2.00,durationMs=2000
        # ----------------------------------------------------

        if tag == "EVENT":
            print(f"[event] {line}")

            if len(parts) >= 2 and parts[1] == "startCmd":
                kv = parse_key_value_parts(parts[2:])

                try:
                    duration_ms = float(kv.get("durationMs", "0"))
                except ValueError:
                    duration_ms = 0.0

                store.start_command(duration_ms)

            continue

        # ----------------------------------------------------
        # Info-Zeilen:
        # #INFO,...
        # ----------------------------------------------------

        if tag == "INFO":
            print(f"[info]  {line}")
            continue

        # ----------------------------------------------------
        # Sonstige Debug-/Statuszeilen:
        # #CONNECTED
        # #DISCONNECTED
        # #Warte auf Handshake...
        # #Handshake1 OK
        # ----------------------------------------------------

        if tag in ("CONNECTED", "DISCONNECTED"):
            print(f"[link]  {line}")
            continue

        if content.startswith("Warte auf Handshake") or content.startswith("Handshake"):
            print(f"[link]  {line}")
            continue

        # ----------------------------------------------------
        # Messdaten:
        # #WHEELS,0,...
        # #CHASSIS,0,...
        # ----------------------------------------------------

        if tag in ("WHEELS", "CHASSIS"):
            if not store.ready:
                print(f"[Daten ohne Header] {line}")
                continue

            try:
                vals = parse_float_list(parts[1:])
            except ValueError:
                print(f"[Wert?] {line}")
                continue

            store.push_values(tag, vals)
            continue

        # ----------------------------------------------------
        # Alte Datenzeile:
        # #100,0.30,...
        # ----------------------------------------------------

        if store.ready:
            try:
                vals = parse_float_list(parts)
            except ValueError:
                print(f"[debug] {line}")
                continue

            store.push_values(store.mode, vals)
            continue

        print(f"[debug] {line}")

    store.close_csv()
    ser.close()


# ─────────────────────────────────────────────
# Farben
# ─────────────────────────────────────────────

COLORS_RAD = {
    "VoLi": "#1f77b4",
    "VoRe": "#ff7f0e",
    "HiLi": "#2ca02c",
    "HiRe": "#d62728",
}

COLORS_VEH = {
    "vx_i": "#9467bd",
    "vy_i": "#8c564b",
    "wz_i": "#e377c2",
}


# ─────────────────────────────────────────────
# Plot-Hilfsfunktion
# ─────────────────────────────────────────────

def build_command_boundary_series(t_sec, values, cmd_indices, local_ms):
    """
    Erzeugt eine durchgehende Linie mit senkrechtem Sprung an CMDT-Grenzen.

    Zweck:
      - innerhalb eines CMDT-Befehls normale Linien
      - keine Treppe bei jedem Messpunkt
      - keine Luecken zwischen Befehlen
      - keine schraege Verbindung ueber einen CMDT-Wechsel

    Diese Funktion ist nur fuer Sollwerte und PWM gedacht.
    Istwerte werden bewusst normal geplottet.
    """
    n = min(len(t_sec), len(values), len(cmd_indices), len(local_ms))

    if n == 0:
        return [], []

    x = [t_sec[0]]
    y = [values[0]]

    for i in range(1, n):
        cmd_changed = cmd_indices[i] != cmd_indices[i - 1]

        if cmd_changed:
            # Die echte CMDT-Grenze liegt bei t_plot_ms - lokaler ms-Zeit
            boundary = t_sec[i] - (local_ms[i] / 1000.0)

            # Schutz gegen Rundungsfehler oder minimal unsaubere Zeitstempel
            if boundary < x[-1]:
                boundary = x[-1]

            # alten Wert bis zur Grenze halten
            x.append(boundary)
            y.append(y[-1])

            # an derselben x-Position auf den neuen Wert springen
            x.append(boundary)
            y.append(values[i])

        x.append(t_sec[i])
        y.append(values[i])

    return x, y


# ─────────────────────────────────────────────
# Live-Plot
# ─────────────────────────────────────────────

def start_plot():
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle("Robot Monitor", fontsize=13)

    def update(_):
        if not store.ready:
            return

        d, mode = store.snapshot()

        # Fortlaufende Plot-Zeit innerhalb des aktuellen Arduino-/Script-Laufs.
        t_ms = d.get("t_plot_ms", [])

        if not t_ms:
            return

        t = [x / 1000.0 for x in t_ms]

        for ax in axes:
            ax.cla()
            ax.grid(True, linestyle="--", alpha=0.5)

        axes[1].set_xlabel("Zeit [s]")

        if mode == "WHEELS":
            ax_v = axes[0]
            ax_pwm = axes[1]

            ax_v.set_ylabel("Geschwindigkeit [m/s]")
            ax_pwm.set_ylabel("PWM")

            cmd_indices = d.get("cmd_index", [])
            local_ms = d.get("ms", [])

            for name, col in COLORS_RAD.items():
                s_key = f"{name}_s"
                i_key = f"{name}_i"
                pwm_key = f"{name}_pwm"

                s = d.get(s_key, [])
                ist = d.get(i_key, [])
                pwm = d.get(pwm_key, [])

                n = min(len(t), len(s), len(ist), len(cmd_indices), len(local_ms))

                if n:
                    # Sollwerte: normal verbunden, aber an CMDT-Grenzen senkrecht.
                    x_soll, y_soll = build_command_boundary_series(
                        t[:n],
                        s[:n],
                        cmd_indices[:n],
                        local_ms[:n]
                    )

                    ax_v.plot(
                        x_soll,
                        y_soll,
                        linestyle="--",
                        color=col,
                        alpha=0.6,
                        label=f"{name} Soll"
                    )

                    # Istwerte: echte Messwerte, ganz normal durchzeichnen.
                    # Keine kuenstliche Unterbrechung, keine kuenstliche Treppe.
                    ax_v.plot(
                        t[:n],
                        ist[:n],
                        linestyle="-",
                        color=col,
                        label=f"{name} Ist"
                    )

                n_p = min(len(t), len(pwm), len(cmd_indices), len(local_ms))

                if n_p:
                    # PWM: normale Linie, aber an CMDT-Grenzen senkrecht.
                    x_pwm, y_pwm = build_command_boundary_series(
                        t[:n_p],
                        pwm[:n_p],
                        cmd_indices[:n_p],
                        local_ms[:n_p]
                    )

                    ax_pwm.plot(
                        x_pwm,
                        y_pwm,
                        linestyle="-",
                        color=col,
                        label=f"{name} PWM"
                    )

            ax_v.legend(loc="lower right", fontsize=7, ncol=4)
            ax_pwm.legend(loc="lower right", fontsize=7, ncol=4)

        elif mode == "CHASSIS":
            ax_rad = axes[0]
            ax_veh = axes[1]

            ax_rad.set_ylabel("Radgeschwindigkeit Ist [m/s]")
            ax_veh.set_ylabel("Fahrzeug [m/s / rad/s]")

            for name, col in COLORS_RAD.items():
                vals = d.get(f"{name}_i", [])
                n = min(len(t), len(vals))

                if n:
                    ax_rad.plot(
                        t[:n],
                        vals[:n],
                        color=col,
                        label=f"{name}"
                    )

            for key, col in COLORS_VEH.items():
                vals = d.get(key, [])
                n = min(len(t), len(vals))

                if n:
                    ax_veh.plot(
                        t[:n],
                        vals[:n],
                        color=col,
                        label=key
                    )

            ax_rad.legend(loc="lower right", fontsize=8, ncol=4)
            ax_veh.legend(loc="lower right", fontsize=8, ncol=3)

        n_pts = len(t)
        cmd_values = d.get("cmd_index", [])
        last_cmd = cmd_values[-1] if cmd_values else 0

        axes[0].set_title(
            f"{mode} - {n_pts} Messpunkte - CMDT #{int(last_cmd)} - t = {t[-1]:.1f} s",
            fontsize=10
        )

        fig.tight_layout()

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=UPDATE_MS,
        cache_frame_data=False
    )

    plt.show()
    return ani


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Robot Serial Monitor fuer vorne-Arduino"
    )

    p.add_argument(
        "--port",
        "-p",
        default=DEFAULT_PORT,
        help=f"COM-Port des vorne-Arduino (Standard: {DEFAULT_PORT})"
    )

    p.add_argument(
        "--baud",
        "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baudrate (Standard: {DEFAULT_BAUD})"
    )

    p.add_argument(
        "--csv",
        "-c",
        default=None,
        help="CSV-Dateiname (Standard: robot_YYYYMMDD_HHMMSS.csv)"
    )

    p.add_argument(
        "--list",
        "-l",
        action="store_true",
        help="Verfuegbare COM-Ports anzeigen"
    )

    p.add_argument(
        "--no-plot",
        action="store_true",
        help="Nur Terminal, kein Live-Plot"
    )

    args = p.parse_args()

    if args.list:
        for pt in serial.tools.list_ports.comports():
            print(f"  {pt.device:12s} {pt.description}")

        return

    csv_path = args.csv or f"robot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    t = threading.Thread(
        target=serial_thread,
        args=(args.port, args.baud, csv_path),
        daemon=True
    )

    t.start()

    if args.no_plot:
        print("Kein Plot aktiv. STRG+C zum Beenden.")

        try:
            t.join()
        except KeyboardInterrupt:
            pass
    else:
        start_plot()


if __name__ == "__main__":
    main()