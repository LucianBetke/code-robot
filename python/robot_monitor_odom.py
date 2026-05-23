"""
robot_monitor_odom.py
=====================
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
    #ODOM,ms,x_cm,y_cm,path_cm,phi_deg

Dieses Programm kann zwei Hauptanzeigen:

  DEFAULT_DISPLAY_MODE = "ODOM"
    x-Achse unten: path_cm
    x-Achse oben:  Zeit des letzten Laufes [s]

    Diagramm 1:    y_world ueber Weg
    Diagramm 2:    y_body ueber Weg
    Diagramm 3:    phi_deg Verdrehung ueber Weg

    Wichtig:
      Der Arduino liefert nach dem Speicherumbau lokale Koordinaten:
        x_cm = x_body_cm
        y_cm = y_body_cm

      Python berechnet daraus zusaetzlich:
        x_world_cm
        y_world_cm

    Mehrere Durchlaeufe werden automatisch erkannt:
      Wenn path_cm stark zurueckspringt, beginnt ein neuer Lauf.
      Alte Kurven bleiben erhalten.
      Die Legende zeigt Lauf 1, Lauf 2, Lauf 3 ...

  DEFAULT_DISPLAY_MODE = "WHEELS"
    alter Radplot:
    oben: Soll/Ist-Geschwindigkeit
    unten: PWM

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
import math
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

DEFAULT_DISPLAY_MODE = "ODOM"

PLOT_MODES = ("WHEELS", "ODOM", "CHASSIS")

ODOM_RESET_DROP_CM = 10.0
ODOM_RESET_TIME_DROP_MS = 500.0
ODOM_MIN_RUN_POINTS = 2
ODOM_TIME_AXIS_TICKS = 6

ODOM_STATUS_TEXT_Y = 0.925
ODOM_STATUS_TEXT_FONTSIZE = 10
ODOM_FIGSIZE = (11, 8)


DEFAULT_WHEELS_COLS = [
    "ms",
    "VoLi_s", "VoLi_i", "VoLi_pwm",
    "VoRe_s", "VoRe_i", "VoRe_pwm",
    "HiLi_s", "HiLi_i", "HiLi_pwm",
    "HiRe_s", "HiRe_i", "HiRe_pwm",
]


DEFAULT_ODOM_COLS = [
    "ms",
    "x_cm",
    "y_cm",
    "path_cm",
    "phi_deg",
]


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
        print("[Run] neuer Datenpuffer angelegt")

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


def display_mode_allows(display_mode: str, tag: str) -> bool:
    display_mode = display_mode.upper()
    tag = tag.upper()

    if display_mode == "AUTO":
        return True

    return display_mode == tag


def ensure_default_header(mode: str, csv_path: str) -> bool:
    mode = mode.upper()

    if store.ready:
        return store.mode == mode

    if mode == "WHEELS":
        store.init_cols("WHEELS", DEFAULT_WHEELS_COLS)
        store.open_csv(csv_path)
        return True

    if mode == "ODOM":
        store.init_cols("ODOM", DEFAULT_ODOM_COLS)
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

def serial_thread(port: str, baud: int, csv_path: str, display_mode: str):
    display_mode = display_mode.upper()

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[Fehler] Port '{port}' nicht geoeffnet: {e}")
        print("Verfuegbare Ports:")

        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")

        sys.exit(1)

    print(f"[Serial] {port} @ {baud} baud - warte auf Daten ...")
    print(f"[Plotmodus] {display_mode}")
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

        if tag == "HDR":
            if len(parts) < 3:
                print(f"[Header?] {line}")
                continue

            mode = parts[1].upper()
            cols = parts[2:]

            if mode not in PLOT_MODES:
                print(f"[Header ignoriert] {line}")
                continue

            if not display_mode_allows(display_mode, mode):
                continue

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        if tag == "MS":
            cols = parts
            mode = "CHASSIS" if "vx_i" in cols else "WHEELS"

            if not display_mode_allows(display_mode, mode):
                continue

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

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

        if tag in ("WHEELS", "ODOM", "CHASSIS"):
            if not display_mode_allows(display_mode, tag):
                continue

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


def split_odom_runs(path_cm, y_cm, phi_deg, x_cm, t_ms):
    n = min(len(path_cm), len(y_cm), len(phi_deg))

    runs = []
    current = {
        "path": [],
        "y": [],
        "phi": [],
        "x": [],
        "t_ms": [],
    }

    for i in range(n):
        p = float(path_cm[i])
        y = float(y_cm[i])
        phi = float(phi_deg[i])

        x = float(x_cm[i]) if i < len(x_cm) else 0.0
        t = float(t_ms[i]) if i < len(t_ms) else 0.0

        if current["path"]:
            prev_p = current["path"][-1]
            prev_t = current["t_ms"][-1]

            path_reset = p < (prev_p - ODOM_RESET_DROP_CM)
            time_reset = t < (prev_t - ODOM_RESET_TIME_DROP_MS)

            if path_reset or time_reset:
                runs.append(current)
                current = {
                    "path": [],
                    "y": [],
                    "phi": [],
                    "x": [],
                    "t_ms": [],
                }

        current["path"].append(p)
        current["y"].append(y)
        current["phi"].append(phi)
        current["x"].append(x)
        current["t_ms"].append(t)

    if current["path"]:
        runs.append(current)

    return runs


def compute_world_coordinates(run):
    """
    Rekonstruiert Weltkoordinaten aus lokalen Arduino-Koordinaten.

    Eingang aus #ODOM nach dem Arduino-Umbau:
      x_cm   = x_body_cm
      y_cm   = y_body_cm
      phi_deg

    Vorgehen:
      Aus den Differenzen dx_body/dy_body pro Messschritt wird mit phi_mid
      wieder dx_world/dy_world gebildet. So bleibt der Arduino ohne sin/cos,
      aber Python kann trotzdem die Weltbahn anzeigen.
    """
    x_body = run["x"]
    y_body = run["y"]
    phi_deg = run["phi"]

    n = min(len(x_body), len(y_body), len(phi_deg))

    if n == 0:
        return [], []

    x_world = [0.0]
    y_world = [0.0]

    for i in range(1, n):
        dx_body = x_body[i] - x_body[i - 1]
        dy_body = y_body[i] - y_body[i - 1]

        phi_prev = math.radians(phi_deg[i - 1])
        phi_now = math.radians(phi_deg[i])
        dphi = phi_now - phi_prev
        phi_mid = phi_prev + 0.5 * dphi

        c = math.cos(phi_mid)
        s = math.sin(phi_mid)

        dx_world = dx_body * c - dy_body * s
        dy_world = dx_body * s + dy_body * c

        x_world.append(x_world[-1] + dx_world)
        y_world.append(y_world[-1] + dy_world)

    return x_world, y_world


def get_default_color(index):
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])

    if not colors:
        return None

    return colors[index % len(colors)]


def interpolate_time_for_path(path_values, time_values_ms, path_target):
    n = min(len(path_values), len(time_values_ms))

    if n == 0:
        return 0.0

    if path_target <= path_values[0]:
        return time_values_ms[0] / 1000.0

    for i in range(1, n):
        p0 = path_values[i - 1]
        p1 = path_values[i]

        if p0 <= path_target <= p1:
            t0 = time_values_ms[i - 1]
            t1 = time_values_ms[i]

            dp = p1 - p0

            if abs(dp) < 1e-9:
                return t1 / 1000.0

            alpha = (path_target - p0) / dp
            return (t0 + alpha * (t1 - t0)) / 1000.0

    return time_values_ms[n - 1] / 1000.0


def make_even_ticks(start_value, end_value, count):
    if count <= 1:
        return [start_value]

    step = (end_value - start_value) / (count - 1)
    return [start_value + i * step for i in range(count)]


def update_time_axis(time_ax, path_values, time_values_ms, max_path):
    """
    Aktualisiert die obere Zeitachse.

    Wichtiger Fehlerfix:
    Diese Achse kommt von twiny() und teilt sich die y-Achse mit der Hauptachse.
    Deshalb darf hier weder time_ax.cla() noch time_ax.set_yticks([]) stehen.
    Sonst verschwinden die Zahlenwerte auf der Ordinate.
    """
    time_ax.set_visible(True)

    x_max = max_path * 1.03 if max_path > 0.0 else 1.0
    time_ax.set_xlim(0.0, x_max)

    time_ax.grid(False)

    time_ax.tick_params(
        axis="y",
        which="both",
        left=False,
        right=False,
        labelleft=False,
        labelright=False
    )

    time_ax.xaxis.set_ticks_position("top")
    time_ax.xaxis.set_label_position("top")
    time_ax.tick_params(
        axis="x",
        which="both",
        top=True,
        labeltop=True,
        bottom=False,
        labelbottom=False,
        pad=3
    )
    time_ax.set_xlabel("Zeit letzter Lauf [s]", labelpad=8)

    time_ax.spines["bottom"].set_visible(False)
    time_ax.spines["left"].set_visible(False)
    time_ax.spines["right"].set_visible(False)
    time_ax.spines["top"].set_visible(True)

    if not path_values or not time_values_ms or max_path <= 0.0:
        time_ax.set_xticks([])
        return

    ticks = make_even_ticks(0.0, max_path, ODOM_TIME_AXIS_TICKS)
    labels = [
        f"{interpolate_time_for_path(path_values, time_values_ms, tick):.1f}"
        for tick in ticks
    ]

    time_ax.set_xticks(ticks)
    time_ax.set_xticklabels(labels)


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
    fig, axes = plt.subplots(3, 1, figsize=ODOM_FIGSIZE, sharex=False)
    fig.suptitle("Robot Monitor", fontsize=13, y=0.988)

    status_text = fig.text(
        0.5,
        ODOM_STATUS_TEXT_Y,
        "",
        ha="center",
        va="top",
        fontsize=ODOM_STATUS_TEXT_FONTSIZE
    )

    time_axes = [ax.twiny() for ax in axes]

    def apply_layout():
        fig.subplots_adjust(
            left=0.075,
            right=0.985,
            top=0.82,
            bottom=0.075,
            hspace=0.72
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

        if mode == "ODOM":
            update_odom_plot(d, axes, time_axes, status_text)
        elif mode == "WHEELS":
            status_text.set_text("")
            hide_time_axes(time_axes)
            update_wheels_plot(d, axes)
        elif mode == "CHASSIS":
            status_text.set_text("")
            hide_time_axes(time_axes)
            update_chassis_plot(d, axes)

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=UPDATE_MS,
        cache_frame_data=False
    )

    plt.show()
    return ani


def hide_time_axes(time_axes):
    """
    Blendet die oberen Zeitachsen aus.

    Auch hier kein cla() und kein set_yticks([]), weil twiny()
    die y-Achse mit der Hauptachse teilt.
    """
    for ax in time_axes:
        ax.set_visible(False)
        ax.set_xlabel("")
        ax.set_xticks([])

        ax.tick_params(
            axis="x",
            which="both",
            top=False,
            labeltop=False,
            bottom=False,
            labelbottom=False
        )

        ax.tick_params(
            axis="y",
            which="both",
            left=False,
            right=False,
            labelleft=False,
            labelright=False
        )


def show_time_axes(time_axes):
    for ax in time_axes:
        ax.set_visible(True)


def clear_axes(axes):
    for ax in axes:
        ax.cla()
        ax.grid(True, linestyle="--", alpha=0.5)


def update_odom_plot(d, axes, time_axes, status_text=None):
    for ax in axes:
        ax.set_visible(True)

    show_time_axes(time_axes)

    path_cm = d.get("path_cm", [])
    y_body_cm = d.get("y_cm", [])
    phi_deg = d.get("phi_deg", [])
    x_body_cm = d.get("x_cm", [])
    t_ms = d.get("ms", [])

    runs = split_odom_runs(path_cm, y_body_cm, phi_deg, x_body_cm, t_ms)

    visible_runs = [
        run for run in runs
        if len(run["path"]) >= ODOM_MIN_RUN_POINTS
    ]

    if not visible_runs:
        return

    clear_axes(axes)

    ax_y_world = axes[0]
    ax_y_body = axes[1]
    ax_phi = axes[2]

    max_path = 0.0
    last_world = {
        "x": 0.0,
        "y": 0.0,
    }

    for run_index, run in enumerate(visible_runs, start=1):
        color = get_default_color(run_index - 1)

        path = run["path"]
        y_body = run["y"]
        phi = run["phi"]
        x_world, y_world = compute_world_coordinates(run)

        if path:
            max_path = max(max_path, max(path))

        plot_kwargs_world = {
            "linestyle": "-",
            "label": f"y_world Lauf {run_index}",
        }

        plot_kwargs_body = {
            "linestyle": "-",
            "label": f"y_body Lauf {run_index}",
        }

        plot_kwargs_phi = {
            "linestyle": "-",
            "label": f"phi Lauf {run_index}",
        }

        if color is not None:
            plot_kwargs_world["color"] = color
            plot_kwargs_body["color"] = color
            plot_kwargs_phi["color"] = color

        n_world = min(len(path), len(y_world))
        if n_world:
            ax_y_world.plot(path[:n_world], y_world[:n_world], **plot_kwargs_world)

        n_body = min(len(path), len(y_body))
        if n_body:
            ax_y_body.plot(path[:n_body], y_body[:n_body], **plot_kwargs_body)

        n_phi = min(len(path), len(phi))
        if n_phi:
            ax_phi.plot(path[:n_phi], phi[:n_phi], **plot_kwargs_phi)

        if run is visible_runs[-1] and x_world and y_world:
            last_world["x"] = x_world[-1]
            last_world["y"] = y_world[-1]

    ax_y_world.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)
    ax_y_body.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)
    ax_phi.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)

    ax_y_world.set_xlabel("Weg path [cm]")
    ax_y_world.set_ylabel("y_world [cm]")

    ax_y_body.set_xlabel("Weg path [cm]")
    ax_y_body.set_ylabel("y_body [cm]")

    ax_phi.set_xlabel("Weg path [cm]")
    ax_phi.set_ylabel("phi [deg]")

    if max_path > 0.0:
        x_max = max_path * 1.03
        ax_y_world.set_xlim(0.0, x_max)
        ax_y_body.set_xlim(0.0, x_max)
        ax_phi.set_xlim(0.0, x_max)

    # Ordinaten-Zahlenwerte explizit sichtbar lassen.
    ax_y_world.tick_params(axis="y", which="both", labelleft=True, left=True)
    ax_y_body.tick_params(axis="y", which="both", labelleft=True, left=True)
    ax_phi.tick_params(axis="y", which="both", labelleft=True, left=True)

    ax_y_world.legend(loc="lower left", fontsize=8, ncol=2)
    ax_y_body.legend(loc="lower left", fontsize=8, ncol=2)
    ax_phi.legend(loc="lower left", fontsize=8, ncol=2)

    last_run = visible_runs[-1]

    last_path = last_run["path"][-1]
    last_x_body = last_run["x"][-1]
    last_y_body = last_run["y"][-1]
    last_phi = last_run["phi"][-1]
    last_t = last_run["t_ms"][-1] / 1000.0

    total_points = sum(len(run["path"]) for run in visible_runs)
    run_count = len(visible_runs)

    for time_ax in time_axes:
        update_time_axis(
            time_ax,
            last_run["path"],
            last_run["t_ms"],
            max_path
        )

    status_line = (
        f"ODOM - {run_count} Laeufe - {total_points} Messpunkte - "
        f"letzter Lauf: path = {last_path:.2f} cm - "
        f"x_body = {last_x_body:.2f} cm - "
        f"y_body = {last_y_body:.2f} cm - "
        f"x_world = {last_world['x']:.2f} cm - "
        f"y_world = {last_world['y']:.2f} cm - "
        f"phi = {last_phi:.2f} deg - t = {last_t:.1f} s"
    )

    if status_text is not None:
        status_text.set_text(status_line)
        ax_y_world.set_title("")
    else:
        ax_y_world.set_title(status_line, fontsize=10, pad=28)


def update_wheels_plot(d, axes):
    for ax in axes:
        ax.set_visible(False)

    axes[0].set_visible(True)
    axes[1].set_visible(True)

    t_ms = d.get("t_plot_ms", [])

    if not t_ms:
        return

    t = [x / 1000.0 for x in t_ms]

    clear_axes(axes[:2])

    axes[1].set_xlabel("Zeit [s]")

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

            ax_v.plot(
                t[:n],
                ist[:n],
                linestyle="-",
                color=col,
                label=f"{name} Ist"
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
                label=f"{name} PWM"
            )

    ax_v.legend(loc="lower right", fontsize=7, ncol=4)
    ax_pwm.legend(loc="lower right", fontsize=7, ncol=4)

    ax_pwm.set_ylim(-255, 255)

    n_pts = len(t)

    ax_v.set_title(
        f"WHEELS - {n_pts} Messpunkte - t = {t[-1]:.1f} s",
        fontsize=10
    )


def update_chassis_plot(d, axes):
    for ax in axes:
        ax.set_visible(False)

    axes[0].set_visible(True)
    axes[1].set_visible(True)

    t_ms = d.get("t_plot_ms", [])

    if not t_ms:
        return

    t = [x / 1000.0 for x in t_ms]

    clear_axes(axes[:2])

    axes[1].set_xlabel("Zeit [s]")

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

    ax_rad.set_title(
        f"CHASSIS - {n_pts} Messpunkte - t = {t[-1]:.1f} s",
        fontsize=10
    )


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
        "--mode",
        "-m",
        default=DEFAULT_DISPLAY_MODE,
        choices=[
            "ODOM", "WHEELS", "CHASSIS", "AUTO",
            "odom", "wheels", "chassis", "auto"
        ],
        help=f"Plotmodus: ODOM, WHEELS, CHASSIS oder AUTO (Standard: {DEFAULT_DISPLAY_MODE})"
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

    display_mode = args.mode.upper()

    csv_path = args.csv or f"robot_{display_mode.lower()}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    t = threading.Thread(
        target=serial_thread,
        args=(args.port, args.baud, csv_path, display_mode),
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