# ============================================================
# Datei: serial_io.py
# Zweck:
#   - Gemeinsamer Datenpuffer Store
#   - Serial-Port oeffnen und lesen
#   - Arduino-Zeilen parsen
#   - #WHEELS / #ODOM / #CHASSIS in den Store schreiben
#   - CSV-Datei schreiben
#
# Diese Datei zeichnet nichts.
# Plotten macht plotter.py.
# Starten macht main.py.
# Einstellungen stehen in config.py.
# ============================================================

import csv
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports

from config import *


# ============================================================
# Gemeinsamer Datenpuffer
# ============================================================

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
        self.current_duration_ms = 0.0
        self.sample_index = 0

        self.time_base_ms = 0.0
        self.last_raw_ms = None
        self.last_plot_ms = None
        self.cmd_index_at_last_sample = 0

    def init_cols(self, mode: str, raw_cols):
        mode = mode.upper()

        with self.lock:
            self.mode = mode
            self.raw_cols = list(raw_cols)

            self.cols = ["sample", "cmd_index", "t_plot_ms"] + self.raw_cols
            self.bufs = {c: deque(maxlen=MAX_POINTS) for c in self.cols}
            self.ready = True

            self.cmd_index = 0
            self.current_duration_ms = 0.0
            self.sample_index = 0

            self.time_base_ms = 0.0
            self.last_raw_ms = None
            self.last_plot_ms = None
            self.cmd_index_at_last_sample = 0

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
            self.cmd_index += 1
            self.current_duration_ms = duration_ms

        print(f"[CMD] start #{self.cmd_index}, durationMs={duration_ms:.0f}")

    def _compute_plot_time_ms(self, raw_ms: float) -> float:
        if self.last_raw_ms is None:
            if self.cmd_index == 0:
                self.cmd_index = 1

            plot_ms = raw_ms

            self.last_raw_ms = raw_ms
            self.last_plot_ms = plot_ms
            self.cmd_index_at_last_sample = self.cmd_index

            return plot_ms

        raw_time_reset = raw_ms < (self.last_raw_ms - PLOT_TIME_RESET_DROP_MS)

        if raw_time_reset:
            if self.last_plot_ms is not None:
                self.time_base_ms = self.last_plot_ms

            if self.cmd_index == self.cmd_index_at_last_sample:
                self.cmd_index += 1

            print(
                f"[Zeitreset erkannt] raw_ms {self.last_raw_ms:.0f} -> {raw_ms:.0f}, "
                f"neuer Zeitversatz = {self.time_base_ms:.0f} ms, "
                f"cmd_index = {self.cmd_index}"
            )

        plot_ms = self.time_base_ms + raw_ms

        if self.last_plot_ms is not None and plot_ms < self.last_plot_ms:
            plot_ms = self.last_plot_ms

        self.last_raw_ms = raw_ms
        self.last_plot_ms = plot_ms
        self.cmd_index_at_last_sample = self.cmd_index

        return plot_ms

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

            raw_ms = float(row_raw.get("ms", 0.0))
            t_plot_ms = self._compute_plot_time_ms(raw_ms)

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


# ============================================================
# Globale Store-Instanz
# ============================================================
#
# plotter.py liest daraus.
# serial_io.py schreibt hinein.
# ============================================================

store = Store()


# ============================================================
# Parser-Hilfsfunktionen
# ============================================================

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


# ============================================================
# COM-Ports anzeigen
# ============================================================

def print_available_ports():
    print("Verfuegbare Ports:")

    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  {p.description}")


# ============================================================
# Serial-Thread
# ============================================================

def serial_thread(port: str, baud: int, csv_path: str, display_mode: str):
    display_mode = display_mode.upper()

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[Fehler] Port '{port}' nicht geoeffnet: {e}")
        print_available_ports()
        return

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

        # ----------------------------------------------------
        # Normale UART-/Protokollzeilen ohne '#'
        # ----------------------------------------------------

        if any(line.startswith(p) for p in IGNORE_PREFIXES):
            print(f"[uart]  {line}")
            continue

        if not line.startswith("#"):
            print(f"[?] {line}")
            continue

        # ----------------------------------------------------
        # Debug- und Messdaten mit '#'
        # ----------------------------------------------------

        content = line[1:]
        parts = [p.strip() for p in content.split(",")]

        if not parts:
            continue

        tag = parts[0].upper()

        # ----------------------------------------------------
        # Alter Header-Modus:
        # #HDR,WHEELS,ms,...
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Alter MS-Modus:
        # #MS,...
        # bleibt zur Sicherheit noch drin
        # ----------------------------------------------------

        if tag == "MS":
            cols = parts
            mode = "CHASSIS" if "vx_i" in cols else "WHEELS"

            if not display_mode_allows(display_mode, mode):
                continue

            store.init_cols(mode, cols)
            store.open_csv(csv_path)
            continue

        # ----------------------------------------------------
        # Ereignisse, z. B. alter startCmd-Mechanismus
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
        # Info- und Link-Zeilen
        # ----------------------------------------------------

        if tag == "INFO":
            print(f"[info]  {line}")
            continue

        if tag in ("CONNECTED", "DISCONNECTED"):
            print(f"[link]  {line}")
            continue

        if content.startswith("Warte auf Handshake") or content.startswith("Handshake"):
            print(f"[link]  {line}")
            continue

        # ----------------------------------------------------
        # Alte Debug-Zeilen, die den Plot nicht fuettern sollen
        # ----------------------------------------------------

        if tag == "CNTF":
            continue

        # ----------------------------------------------------
        # Neue feste Messformate:
        # #WHEELS,...
        # #ODOM,...
        # #CHASSIS,...
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Fallback:
        # Wenn ein Modus bereits aktiv ist, versuche reine Zahlen
        # als Datenzeile zu lesen.
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