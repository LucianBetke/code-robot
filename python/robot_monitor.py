"""
robot_monitor.py
=================
Serial-Monitor für das Roboter-System (vorne + hinten).

Verbinde dich mit dem COM-Port des VORNE-Arduino (USB).

Was auf dem Port ankommt:
  - PING / ACK / KA         -> UartLink-Protokoll (wird ignoriert)
  - VSOL,xxx,xxx            -> Soll-Werte die an hinten gesendet werden (wird ignoriert)
  - #ms,VoLi_s,...          -> CSV-Header (einmalig beim Start)
  - #100,0.30,0.00,80,...   -> Datenzeilen (alle 100 ms, nur wenn Befehl aktiv)

Spalten der Datenzeilen:
  ms        Zeitstempel [ms] seit Befehlsstart (0 wenn kein Befehl)
  VoLi_s    Vorne-Links  Soll  [m/s]
  VoLi_i    Vorne-Links  Ist   [m/s]
  VoLi_pwm  Vorne-Links  PWM   [-255..255]
  VoRe_s    Vorne-Rechts Soll  [m/s]
  VoRe_i    Vorne-Rechts Ist   [m/s]
  VoRe_pwm  Vorne-Rechts PWM
  HiLi_s    Hinten-Links Soll  [m/s]  (über VSOL gesendet)
  HiLi_i    Hinten-Links Ist   [m/s]  (über VIST empfangen)
  HiLi_pwm  Hinten-Links PWM         (über VIST empfangen)
  HiRe_s    Hinten-Rechts Soll [m/s]
  HiRe_i    Hinten-Rechts Ist  [m/s]
  HiRe_pwm  Hinten-Rechts PWM

Abhängigkeiten:
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
matplotlib.use("TkAgg")          # ggf. auf "Qt5Agg" ändern
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ─────────────────────────────────────────────
# Einstellungen
# ─────────────────────────────────────────────
DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
MAX_POINTS   = 3000       # Punkte im Ring-Puffer
UPDATE_MS    = 200        # Plot-Update-Intervall

EXPECTED_COLS = [
    "ms",
    "VoLi_s", "VoLi_i", "VoLi_pwm",
    "VoRe_s", "VoRe_i", "VoRe_pwm",
    "HiLi_s", "HiLi_i", "HiLi_pwm",
    "HiRe_s", "HiRe_i", "HiRe_pwm",
]

# ─────────────────────────────────────────────
# Gemeinsamer Datenpuffer (thread-safe)
# ─────────────────────────────────────────────
class Store:
    def __init__(self):
        self.lock        = threading.Lock()
        self.ready       = False          # Header wurde empfangen
        self.cols        = []
        self.bufs        = {}             # {col: deque}
        self._csv_file   = None
        self._csv_writer = None

    def init_cols(self, cols):
        with self.lock:
            self.cols  = cols
            self.bufs  = {c: deque(maxlen=MAX_POINTS) for c in cols}
            self.ready = True

    def push(self, row: dict):
        with self.lock:
            for k, v in row.items():
                if k in self.bufs:
                    self.bufs[k].append(v)
            if self._csv_writer:
                self._csv_writer.writerow(row)

    def snapshot(self):
        """Gibt eine Kopie aller Buffer zurück (für Plot-Thread)."""
        with self.lock:
            return {c: list(v) for c, v in self.bufs.items()}

    def open_csv(self, path):
        self._csv_file   = open(path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=self.cols, extrasaction="ignore")
        self._csv_writer.writeheader()
        print(f"[CSV] Speichere nach: {path}")

    def close_csv(self):
        if self._csv_file:
            self._csv_file.close()


store = Store()


# ─────────────────────────────────────────────
# Serial-Thread
# ─────────────────────────────────────────────
IGNORE_PREFIXES = ("PING", "PONG", "ACK", "KA", "VSOL", "VIST")

def serial_thread(port: str, baud: int, csv_path: str):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[Fehler] Port '{port}' nicht geöffnet: {e}")
        print("Verfügbare Ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        sys.exit(1)

    print(f"[Serial] {port} @ {baud} baud  – warte auf Daten …")
    time.sleep(2)   # Arduino-Reset abwarten

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

        # ── UartLink-Protokoll ignorieren ────────────────────
        if any(line.startswith(p) for p in IGNORE_PREFIXES):
            print(f"[uart]  {line}")
            continue

        # ── Zeilen die mit # beginnen ─────────────────────────
        if line.startswith("#"):
            content = line[1:]          # '#' abschneiden
            parts   = content.split(",")

            # Header-Zeile: erste Spalte = "ms" (Text)
            if parts[0].strip() == "ms":
                cols = [p.strip() for p in parts]
                store.init_cols(cols)
                store.open_csv(csv_path)
                print(f"[Header] {cols}")
                continue

            # Datenzeile: erste Spalte = Zahl
            if not store.ready:
                continue

            try:
                vals = [float(p) for p in parts]
            except ValueError:
                print(f"[?] {line}")
                continue

            if len(vals) != len(store.cols):
                print(f"[Länge?] erwartet {len(store.cols)}, bekommen {len(vals)}: {line}")
                continue

            row = dict(zip(store.cols, vals))
            store.push(row)
            continue

        # ── alles andere ──────────────────────────────────────
        print(f"[?] {line}")

    store.close_csv()
    ser.close()


# ─────────────────────────────────────────────
# Live-Plot (läuft im Main-Thread)
# ─────────────────────────────────────────────
def start_plot():
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Robot Monitor – Vorne & Hinten", fontsize=13)

    ax_v   = axes[0]    # Geschwindigkeit
    ax_e   = axes[1]    # Regelfehler (Soll - Ist)
    ax_pwm = axes[2]    # PWM

    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.5)

    COLORS = {
        "VoLi": "#1f77b4",   # blau
        "VoRe": "#ff7f0e",   # orange
        "HiLi": "#2ca02c",   # grün
        "HiRe": "#d62728",   # rot
    }

    def update(_):
        if not store.ready:
            return

        d = store.snapshot()
        t_ms = d.get("ms", [])
        if not t_ms:
            return

        t = [x / 1000.0 for x in t_ms]   # ms → s

        for ax in axes:
            ax.cla()
            ax.grid(True, linestyle="--", alpha=0.5)

        ax_v.set_ylabel("Geschwindigkeit [m/s]")
        ax_e.set_ylabel("Fehler (Soll−Ist) [m/s]")
        ax_pwm.set_ylabel("PWM")
        ax_pwm.set_xlabel("Zeit [s]")

        for name, col in COLORS.items():
            s_key   = f"{name}_s"
            i_key   = f"{name}_i"
            pwm_key = f"{name}_pwm"

            s   = d.get(s_key,   [])
            ist = d.get(i_key,   [])
            pwm = d.get(pwm_key, [])

            n = min(len(t), len(s), len(ist))
            if n == 0:
                continue

            t_n   = t[:n]
            s_n   = s[:n]
            ist_n = ist[:n]
            err_n = [s_n[i] - ist_n[i] for i in range(n)]

            # Soll gestrichelt, Ist durchgehend
            ax_v.plot(t_n, s_n,   linestyle="--", color=col, alpha=0.6,
                      label=f"{name} Soll")
            ax_v.plot(t_n, ist_n, linestyle="-",  color=col,
                      label=f"{name} Ist")
            ax_e.plot(t_n, err_n, linestyle="-",  color=col,
                      label=f"{name} Err")

            n_p = min(len(t), len(pwm))
            if n_p:
                ax_pwm.plot(t[:n_p], pwm[:n_p], linestyle="-", color=col,
                            label=f"{name} PWM")

        ax_v.legend(loc="lower right", fontsize=7, ncol=4)
        ax_e.legend(loc="upper right", fontsize=7, ncol=4)
        ax_pwm.legend(loc="lower right", fontsize=7, ncol=4)

        n_pts = len(t)
        ax_v.set_title(f"{n_pts} Messpunkte  –  t = {t[-1]:.1f} s" if n_pts else "warte …",
                       fontsize=10)
        fig.tight_layout()

    ani = animation.FuncAnimation(
        fig, update, interval=UPDATE_MS, cache_frame_data=False)
    plt.show()
    return ani   # Referenz halten (GC-Schutz)


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description="Robot Serial Monitor (vorne-Arduino, 4-Rad-System)")
    p.add_argument("--port", "-p", default=DEFAULT_PORT,
                   help=f"COM-Port des vorne-Arduino (Standard: {DEFAULT_PORT})")
    p.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD,
                   help=f"Baudrate (Standard: {DEFAULT_BAUD})")
    p.add_argument("--csv", "-c", default=None,
                   help="CSV-Dateiname (Standard: robot_YYYYMMDD_HHMMSS.csv)")
    p.add_argument("--list", "-l", action="store_true",
                   help="Verfügbare COM-Ports anzeigen")
    p.add_argument("--no-plot", action="store_true",
                   help="Nur Terminal, kein Live-Plot")
    args = p.parse_args()

    if args.list:
        for pt in serial.tools.list_ports.comports():
            print(f"  {pt.device:12s} {pt.description}")
        return

    csv_path = args.csv or f"robot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    t = threading.Thread(
        target=serial_thread,
        args=(args.port, args.baud, csv_path),
        daemon=True)
    t.start()

    if args.no_plot:
        print("Kein Plot aktiv. STRG+C zum Beenden.")
        try:
            t.join()
        except KeyboardInterrupt:
            pass
    else:
        ani = start_plot()   # blockiert bis Fenster geschlossen


if __name__ == "__main__":
    main()
