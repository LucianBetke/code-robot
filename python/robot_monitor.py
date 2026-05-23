# File: robot_monitor.py

"""
robot_monitor.py
=================
Serial-Monitor fuer das Roboter-System (vorne + hinten).

Verbinde dich mit dem COM-Port des VORNE-Arduino (USB).

Aktuelle Log-Struktur:

  Nutz-/Protokolldaten ohne '#':
    Opening port
    Port open
    Port closed
    PING / ACK / KA
    VSOL,<frameId>,<ackFlag>,<hiLiSoll>,<hiReSoll>

  Debug-/Messdaten mit '#':
    #Warte auf Handshake...
    #CONNECTED
    #Handshake1 OK
    #INFO,...
    #WHEELS,ms,VoLi_s,VoLi_i,VoLi_pwm,VoRe_s,VoRe_i,VoRe_pwm,HiLi_s,HiLi_i,HiLi_pwm,HiRe_s,HiRe_i,HiRe_pwm

Dieses Programm wertet aus:
  #WHEELS

Dieses Programm ignoriert bewusst:
  PING / ACK / KA
  VSOL
  alte #HDR-Zeilen, falls sie doch noch auftauchen
  #CNTF
  #ODOM
  #EVENT

Grund:
  Der Arduino sendet aktuell keine #HDR-Zeilen mehr.
  Python kennt das feste #WHEELS-Format selbst.

Korrektur:
  Mehrere Fahrlaeufe werden anhand eines Zeitruecksprungs erkannt.
  Dadurch werden alte Kurven nicht mehr diagonal mit neuen Kurven verbunden.

Abhaengigkeiten:
  pip install pyserial matplotlib pillow pywin32
"""

import sys
import csv
import time
import threading
import argparse
import os
import io
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ─────────────────────────────────────────────
# Einstellungen
# ─────────────────────────────────────────────

DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
MAX_POINTS = 3000
UPDATE_MS = 200

PLOT_MODES = ("WHEELS", "CHASSIS")

# Neuer Lauf:
# Wenn die lokale Arduino-Zeit deutlich zurueckspringt,
# wird ein neuer Fahr-/Messlauf erkannt.
WHEEL_RUN_RESET_TIME_DROP_MS = 500.0

# Sehr kurze Fragmente werden nicht geplottet.
WHEEL_MIN_RUN_POINTS = 2


# Festes Format fuer die neue Arduino-Ausgabe ohne #HDR:
#
# #WHEELS,
#   ms,
#   VoLi_s, VoLi_i, VoLi_pwm,
#   VoRe_s, VoRe_i, VoRe_pwm,
#   HiLi_s, HiLi_i, HiLi_pwm,
#   HiRe_s, HiRe_i, HiRe_pwm
#
DEFAULT_WHEELS_COLS = [
    "ms",
    "VoLi_s", "VoLi_i", "VoLi_pwm",
    "VoRe_s", "VoRe_i", "VoRe_pwm",
    "HiLi_s", "HiLi_i", "HiLi_pwm",
    "HiRe_s", "HiRe_i", "HiRe_pwm",
]


# Reserve fuer spaetere Chassis-Ausgabe ohne #HDR.
# Solange Arduino kein festes #CHASSIS-Format sendet, bleibt das leer.
DEFAULT_CHASSIS_COLS = []


# ─────────────────────────────────────────────
# Gemeinsamer Datenpuffer
# ─────────────────────────────────────────────

class Store:
    def __init__(self):
        self.lock = threading.Lock()

        self.ready = False
        self.mode = None

        self.raw_cols = []
        self.cols = []
        self.bufs = {}

        self._csv_file = None
        self._csv_writer = None

        self.cmd_index = 0
        self.current_offset_ms = 0.0
        self.current_duration_ms = 0.0
        self.sample_index = 0

    def init_cols(self, mode: str, raw_cols):
        mode = mode.upper()

        with self.lock:
            self.mode = mode
            self.raw_cols = list(raw_cols)

            self.cols = ["sample", "cmd_index", "t_plot_ms"] + self.raw_cols
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
    "Opening port",
    "Port open",
    "Port closed",
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


def ensure_default_header(mode: str, csv_path: str) -> bool:
    """
    Legt beim ersten Datensatz automatisch die festen Spalten an.

    Grund:
      Der Arduino sendet aktuell keine #HDR-Zeilen mehr.
      Python kennt das feste #WHEELS-Format selbst.
    """
    mode = mode.upper()

    if store.ready:
        return True

    if mode == "WHEELS":
        store.init_cols("WHEELS", DEFAULT_WHEELS_COLS)
        store.open_csv(csv_path)
        return True

    if mode == "CHASSIS" and DEFAULT_CHASSIS_COLS:
        store.init_cols("CHASSIS", DEFAULT_CHASSIS_COLS)
        store.open_csv(csv_path)
        return True

    return False


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

        if any(line.startswith(p) for p in IGNORE_PREFIXES):
            print(f"[uart]  {line}")
            continue

        if not line.startswith("#"):
            print(f"[?] {line}")
            continue

        content = line[1:]
        parts = [p.strip() for p in content.split(",")]

        if not parts:
            continue

        tag = parts[0].upper()

        # Alte Header werden weiterhin akzeptiert.
        # Normalerweise kommen sie jetzt aber nicht mehr.
        if tag == "HDR":
            if len(parts) < 3:
                print(f"[Header?] {line}")
                continue

            mode = parts[1].upper()
            cols = parts[2:]

            if mode not in PLOT_MODES:
                print(f"[Header ignoriert] {line}")
                continue

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        # Alte MS-Variante bleibt als Rueckfall erhalten.
        if tag == "MS":
            cols = parts
            mode = "CHASSIS" if "vx_i" in cols else "WHEELS"

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        # Alte EVENT-Zeilen werden weiterhin verstanden,
        # sind in deiner aktuellen Ausgabe aber abgeschaltet.
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

        if tag == "INFO":
            print(f"[info]  {line}")
            continue

        if tag in ("CONNECTED", "DISCONNECTED"):
            print(f"[link]  {line}")
            continue

        if content.startswith("Warte auf Handshake") or content.startswith("Handshake"):
            print(f"[link]  {line}")
            continue

        if tag == "CNTF":
            continue

        if tag == "ODOM":
            continue

        if tag in ("WHEELS", "CHASSIS"):
            if not ensure_default_header(tag, csv_path):
                print(f"[Daten ohne bekanntes Format] {line}")
                continue

            try:
                vals = parse_float_list(parts[1:])
            except ValueError:
                print(f"[Wert?] {line}")
                continue

            store.push_values(tag, vals)
            continue

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
# Plot-Hilfsfunktionen
# ─────────────────────────────────────────────

def build_command_boundary_series(t_sec, values, cmd_indices, local_ms):
    """
    Erzeugt eine durchgehende Linie mit senkrechtem Sprung an CMD-Grenzen.

    Zweck:
      - innerhalb eines Befehls normale Linien
      - keine Treppe bei jedem Messpunkt
      - keine Luecken zwischen Befehlen
      - keine schraege Verbindung ueber einen Befehlswechsel

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
            boundary = t_sec[i] - (local_ms[i] / 1000.0)

            if boundary < x[-1]:
                boundary = x[-1]

            x.append(boundary)
            y.append(y[-1])

            x.append(boundary)
            y.append(values[i])

        x.append(t_sec[i])
        y.append(values[i])

    return x, y


def split_time_runs(local_ms):
    """
    Teilt Messdaten in mehrere Laeufe.

    Ursache des bisherigen Plotfehlers:
      Beim naechsten Fahrversuch startet ms wieder bei 0.
      Matplotlib verbindet aber den letzten Punkt des alten Laufs
      mit dem ersten Punkt des neuen Laufs. Dadurch entstehen
      schraege Verbindungslinien.

    Loesung:
      Wenn ms deutlich zurueckspringt, wird ein neuer Lauf begonnen.
      Jeder Lauf wird separat geplottet.
    """
    runs = []

    if not local_ms:
        return runs

    start = 0

    for i in range(1, len(local_ms)):
        try:
            prev_t = float(local_ms[i - 1])
            curr_t = float(local_ms[i])
        except ValueError:
            continue

        time_reset = curr_t < (prev_t - WHEEL_RUN_RESET_TIME_DROP_MS)

        if time_reset:
            if i - start >= WHEEL_MIN_RUN_POINTS:
                runs.append((start, i))

            start = i

    if len(local_ms) - start >= WHEEL_MIN_RUN_POINTS:
        runs.append((start, len(local_ms)))

    return runs


def safe_slice(values, start, end):
    if not values:
        return []

    end2 = min(end, len(values))

    if start >= end2:
        return []

    return values[start:end2]


# ─────────────────────────────────────────────
# Meldungen
# ─────────────────────────────────────────────

def show_message(title, text):
    print(f"[{title}] {text}")

    try:
        import tkinter.messagebox as messagebox
        messagebox.showinfo(title, text)
    except Exception:
        pass


# ─────────────────────────────────────────────
# Plot als Bild erzeugen
# ─────────────────────────────────────────────

def figure_to_pil_image(fig, dpi=150):
    """
    Erzeugt aus der aktuellen Matplotlib-Figure ein PIL-Bild.

    Wichtig:
      Kein bbox_inches='tight'.
      Dadurch wird nichts an der rechten Seite abgeschnitten.
    """
    from PIL import Image

    fig.canvas.draw()

    buffer = io.BytesIO()

    fig.savefig(
        buffer,
        format="png",
        dpi=dpi,
        facecolor="white"
    )

    buffer.seek(0)

    image = Image.open(buffer).convert("RGB")
    return image


# ─────────────────────────────────────────────
# Bild in Zwischenablage kopieren
# ─────────────────────────────────────────────

def copy_plot_to_clipboard(fig):
    """
    Kopiert die aktuelle Figure als echtes Bild in die Windows-Zwischenablage.

    Danach kann man z. B. in Word, Paint oder LibreOffice mit Strg+V einfuegen.
    """
    if os.name != "nt":
        show_message("Clipboard", "Bild-Zwischenablage ist hier nur fuer Windows eingebaut.")
        return

    try:
        import win32clipboard
        import win32con
    except ImportError:
        show_message(
            "Clipboard",
            "Fehlende Pakete. Bitte ausfuehren:\n\npy -m pip install --upgrade pillow pywin32"
        )
        return

    try:
        image = figure_to_pil_image(fig, dpi=150)

        bmp_buffer = io.BytesIO()
        image.save(bmp_buffer, "BMP")

        # Windows CF_DIB erwartet BMP ohne die ersten 14 Byte BMP-Dateikopf.
        dib_data = bmp_buffer.getvalue()[14:]

        win32clipboard.OpenClipboard()
        try:
            win32clipboard.EmptyClipboard()
            win32clipboard.SetClipboardData(win32con.CF_DIB, dib_data)
        finally:
            win32clipboard.CloseClipboard()

        show_message("Clipboard", "Plot wurde als Bild in die Zwischenablage kopiert.")

    except Exception as e:
        show_message("Clipboard Fehler", f"Kopieren fehlgeschlagen:\n\n{e}")


def add_extra_plot_buttons(fig):
    """
    Fuegt einen eigenen Button unter der Matplotlib-Oberflaeche hinzu:
      - Bild kopieren

    Der direkte Druckbutton wurde bewusst entfernt.
    Grund:
      Windows-Druckertreiber, Skalierung, Papierformat und Randlogik sind
      fuer direkte Python-Druckausgabe zu unzuverlaessig.
    """
    try:
        import tkinter as tk

        manager = plt.get_current_fig_manager()
        window = manager.window

        button_frame = tk.Frame(window, bd=1, relief=tk.GROOVE)

        btn_copy = tk.Button(
            button_frame,
            text="Bild kopieren",
            command=lambda: copy_plot_to_clipboard(fig),
            width=18
        )

        btn_copy.pack(side=tk.LEFT, padx=4, pady=3)

        button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        print("[Plot] Zusatzbutton geladen: Bild kopieren")

    except Exception as e:
        print(f"[Plot] Zusatzbutton konnte nicht geladen werden: {e}")


# ─────────────────────────────────────────────
# Live-Plot
# ─────────────────────────────────────────────

def start_plot():
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("Robot Monitor", fontsize=13)

    def apply_layout():
        fig.subplots_adjust(
            left=0.065,
            right=0.985,
            top=0.88,
            bottom=0.09,
            hspace=0.18
        )

    apply_layout()

    add_extra_plot_buttons(fig)

    def maximize_and_fit_to_window():
        try:
            manager = plt.get_current_fig_manager()
            window = manager.window

            window.state("zoomed")
            window.update_idletasks()

            widget = fig.canvas.get_tk_widget()
            width_px = widget.winfo_width()
            height_px = widget.winfo_height()

            dpi = fig.get_dpi()

            if width_px > 200 and height_px > 200:
                fig.set_size_inches(width_px / dpi, height_px / dpi, forward=True)

            apply_layout()
            fig.canvas.draw_idle()

            print("[Plot] Fenster maximiert und Figure an sichtbare Flaeche angepasst")

        except Exception as e:
            print(f"[Plot] Maximieren nicht moeglich: {e}")

    try:
        manager = plt.get_current_fig_manager()
        manager.window.after(300, maximize_and_fit_to_window)
    except Exception as e:
        print(f"[Plot] Maximieren nicht vorbereitet: {e}")

    def update(_):
        if not store.ready:
            return

        d, mode = store.snapshot()

        local_ms_all = d.get("ms", [])

        if not local_ms_all:
            return

        runs = split_time_runs(local_ms_all)

        if not runs:
            return

        for ax in axes:
            ax.cla()
            ax.grid(True, linestyle="--", alpha=0.5)

        axes[1].set_xlabel("Zeit [s]")

        if mode == "WHEELS":
            update_wheels_plot(d, axes, runs)

        elif mode == "CHASSIS":
            update_chassis_plot(d, axes, runs)

        total_points = sum(end - start for start, end in runs)
        run_count = len(runs)

        last_start, last_end = runs[-1]
        last_ms_values = safe_slice(local_ms_all, last_start, last_end)
        last_t = float(last_ms_values[-1]) / 1000.0 if last_ms_values else 0.0

        cmd_values = d.get("cmd_index", [])
        last_cmd = cmd_values[last_end - 1] if cmd_values and last_end <= len(cmd_values) else 0

        axes[0].set_title(
            f"{mode} - {run_count} Laeufe - {total_points} Messpunkte - "
            f"CMD #{int(last_cmd)} - letzter Lauf t = {last_t:.1f} s",
            fontsize=10
        )

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=UPDATE_MS,
        cache_frame_data=False
    )

    plt.show()
    return ani


def update_wheels_plot(d, axes, runs):
    ax_v = axes[0]
    ax_pwm = axes[1]

    ax_v.set_ylabel("Geschwindigkeit [m/s]")
    ax_pwm.set_ylabel("PWM")

    cmd_indices_all = d.get("cmd_index", [])
    local_ms_all = d.get("ms", [])

    last_run_index = len(runs) - 1

    for run_index, (start, end) in enumerate(runs):
        is_last_run = run_index == last_run_index

        # Alte Laeufe bleiben sichtbar, aber heller.
        alpha_ist = 1.0 if is_last_run else 0.35
        alpha_soll = 0.6 if is_last_run else 0.25
        alpha_pwm = 1.0 if is_last_run else 0.35

        local_ms = safe_slice(local_ms_all, start, end)
        cmd_indices = safe_slice(cmd_indices_all, start, end)

        if not local_ms:
            continue

        t = [float(x) / 1000.0 for x in local_ms]

        for name, col in COLORS_RAD.items():
            s_key = f"{name}_s"
            i_key = f"{name}_i"
            pwm_key = f"{name}_pwm"

            s = safe_slice(d.get(s_key, []), start, end)
            ist = safe_slice(d.get(i_key, []), start, end)
            pwm = safe_slice(d.get(pwm_key, []), start, end)

            n = min(len(t), len(s), len(ist), len(cmd_indices), len(local_ms))

            if n:
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
                    alpha=alpha_soll,
                    label=f"{name} Soll" if is_last_run else "_nolegend_"
                )

                ax_v.plot(
                    t[:n],
                    ist[:n],
                    linestyle="-",
                    color=col,
                    alpha=alpha_ist,
                    label=f"{name} Ist" if is_last_run else "_nolegend_"
                )

            n_p = min(len(t), len(pwm), len(cmd_indices), len(local_ms))

            if n_p:
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
                    alpha=alpha_pwm,
                    label=f"{name} PWM" if is_last_run else "_nolegend_"
                )

    ax_v.legend(loc="lower right", fontsize=7, ncol=4)
    ax_pwm.legend(loc="lower right", fontsize=7, ncol=4)

    ax_pwm.set_ylim(-255, 255)


def update_chassis_plot(d, axes, runs):
    ax_rad = axes[0]
    ax_veh = axes[1]

    ax_rad.set_ylabel("Radgeschwindigkeit Ist [m/s]")
    ax_veh.set_ylabel("Fahrzeug [m/s / rad/s]")

    local_ms_all = d.get("ms", [])
    last_run_index = len(runs) - 1

    for run_index, (start, end) in enumerate(runs):
        is_last_run = run_index == last_run_index
        alpha = 1.0 if is_last_run else 0.35

        local_ms = safe_slice(local_ms_all, start, end)

        if not local_ms:
            continue

        t = [float(x) / 1000.0 for x in local_ms]

        for name, col in COLORS_RAD.items():
            vals = safe_slice(d.get(f"{name}_i", []), start, end)
            n = min(len(t), len(vals))

            if n:
                ax_rad.plot(
                    t[:n],
                    vals[:n],
                    color=col,
                    alpha=alpha,
                    label=f"{name}" if is_last_run else "_nolegend_"
                )

        for key, col in COLORS_VEH.items():
            vals = safe_slice(d.get(key, []), start, end)
            n = min(len(t), len(vals))

            if n:
                ax_veh.plot(
                    t[:n],
                    vals[:n],
                    color=col,
                    alpha=alpha,
                    label=key if is_last_run else "_nolegend_"
                )

    ax_rad.legend(loc="lower right", fontsize=8, ncol=4)
    ax_veh.legend(loc="lower right", fontsize=8, ncol=3)


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