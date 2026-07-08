# serial_io.py
# ============================================================
# Aufgabe:
#  - Serielle Verbindung zum Front-Nano lesen
#  - Arduino-Zeilen parsen
#  - Daten threadsicher im Store ablegen
#
# Unterstuetzte Diagnosezeilen:
#  - #CMDP_BEGIN,id,vx,vy,wz,target
#  - #ODOM,id,ms,path_cm100,x_body_cm100,y_body_cm100,phi_deg100
#  - #WHEELS,ms,...
#
# Wichtig:
#  - Arduino sendet ODOM als Integerwerte mit Faktor 100.
#    Python wandelt diese Werte beim Einlesen wieder in cm bzw. Grad um.
#  - WHEELS-Frames erhalten die cmd_id des zuletzt empfangenen
#    CMDP_BEGIN. Damit kann ein WHEELS-Frame eindeutig einem Lauf
#    zugeordnet und mit dem passenden ODOM-Eintrag (cmd_id, ms)
#    verknuepft werden. Settle-Frames zwischen zwei CMDP_BEGIN-Bloecken
#    behalten die cmd_id des vorherigen Laufs; sie haben keinen
#    passenden ODOM-Eintrag und werden im Plot weggefiltert.
# ============================================================

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock, Event
from typing import Optional, TextIO
import csv
import time


try:
    import serial
except ImportError:
    serial = None


# ============================================================
# Konstanten
# ============================================================

ODOM_SCALE = 100.0


# ============================================================
# Datentypen
# ============================================================

@dataclass
class CmdpBegin:
    cmd_id: int
    vx_cms: float
    vy_cms: float
    wz_deg_s: float
    target: float


@dataclass
class OdomSample:
    cmd_id: int
    ms: float
    path_cm: float
    x_body_cm: float
    y_body_cm: float
    phi_deg: float


@dataclass
class WheelSample:
    cmd_id: int
    ms: float
    values: list[float] = field(default_factory=list)


# ============================================================
# Store
# ============================================================

class Store:
    def __init__(self, max_rows: int = 20000) -> None:
        self.max_rows = int(max_rows)
        self.lock = Lock()

        self.cmdp_by_id: dict[int, CmdpBegin] = {}
        self.cmdp_order: list[int] = []
        self.odom_rows: list[OdomSample] = []
        self.wheels_rows: list[WheelSample] = []

        # cmd_id des zuletzt empfangenen CMDP_BEGIN.
        # Wird WHEELS-Frames mitgegeben, damit Settle/Lauf unterscheidbar sind.
        self.current_cmd_id: int = -1

    def clear(self) -> None:
        with self.lock:
            self.cmdp_by_id.clear()
            self.cmdp_order.clear()
            self.odom_rows.clear()
            self.wheels_rows.clear()
            self.current_cmd_id = -1

    def _trim(self) -> None:
        if len(self.odom_rows) > self.max_rows:
            self.odom_rows = self.odom_rows[-self.max_rows:]
        if len(self.wheels_rows) > self.max_rows:
            self.wheels_rows = self.wheels_rows[-self.max_rows:]

    def add_cmdp_begin(self, msg: CmdpBegin) -> None:
        with self.lock:
            if msg.cmd_id not in self.cmdp_by_id:
                self.cmdp_order.append(msg.cmd_id)
            self.cmdp_by_id[msg.cmd_id] = msg
            self.current_cmd_id = msg.cmd_id

    def add_odom(self, row: OdomSample) -> None:
        with self.lock:
            self.odom_rows.append(row)
            self._trim()

    def add_wheels(self, ms: float, values: list[float]) -> None:
        with self.lock:
            self.wheels_rows.append(WheelSample(
                cmd_id=self.current_cmd_id,
                ms=ms,
                values=values,
            ))
            self._trim()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "cmdp_by_id": dict(self.cmdp_by_id),
                "cmdp_order": list(self.cmdp_order),
                "odom_rows": list(self.odom_rows),
                "wheels_rows": list(self.wheels_rows),
            }


def make_store(max_rows: int = 20000) -> Store:
    return Store(max_rows=max_rows)


# ============================================================
# Zeilen-Parser
# ============================================================

def parse_line(line: str, store: Store) -> bool:
    line = line.strip()
    if not line:
        return False

    try:
        if line.startswith("#CMDP_BEGIN,"):
            parts = line.split(",")
            if len(parts) != 6:
                return False
            store.add_cmdp_begin(CmdpBegin(
                cmd_id   = int(parts[1]),
                vx_cms   = float(parts[2]),
                vy_cms   = float(parts[3]),
                wz_deg_s = float(parts[4]),
                target   = float(parts[5]),
            ))
            return True

        if line.startswith("#ODOM,"):
            parts = line.split(",")
            if len(parts) != 7:
                return False
            store.add_odom(OdomSample(
                cmd_id    = int(parts[1]),
                ms        = float(parts[2]),
                path_cm   = int(parts[3]) / ODOM_SCALE,
                x_body_cm = int(parts[4]) / ODOM_SCALE,
                y_body_cm = int(parts[5]) / ODOM_SCALE,
                phi_deg   = int(parts[6]) / ODOM_SCALE,
            ))
            return True

        if line.startswith("#WHEELS,"):
            parts = line.split(",")
            if len(parts) < 2:
                return False
            store.add_wheels(
                ms     = float(parts[1]),
                values = [float(p) for p in parts[2:]],
            )
            return True

    except (ValueError, IndexError):
        return False

    return False


# ============================================================
# Serielle Verbindung
# ============================================================

def open_serial(port: str, baud: int, timeout: float = 0.2):
    if serial is None:
        raise RuntimeError(
            "pyserial ist nicht installiert. Installiere es mit: pip install pyserial"
        )
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def serial_thread(
    port: str,
    baud: int,
    store: Store,
    stop_event: Optional[Event] = None,
    csv_file: Optional[TextIO] = None,
    echo: bool = True,
) -> None:
    """Liest serielle Zeilen, schreibt sie ins CSV (falls offen) und parst sie in den Store."""
    if stop_event is None:
        stop_event = Event()

    csv_writer = None
    if csv_file is not None:
        csv_writer = csv.writer(csv_file, lineterminator="\n")
        csv_writer.writerow(["line"])

    print("Opening port")
    ser = open_serial(port, baud)
    print("Port open")

    try:
        while not stop_event.is_set():
            raw = ser.readline()
            if not raw:
                time.sleep(0.002)
                continue

            line = raw.decode("utf-8", errors="replace").strip()

            if echo:
                print(line)

            if csv_writer is not None:
                csv_writer.writerow([line])
                csv_file.flush()

            parse_line(line, store)
    finally:
        try:
            ser.close()
        finally:
            print("Port closed")
