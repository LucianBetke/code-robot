# serial_io.py
# ============================================================
# Aufgabe:
#  - Serielle Verbindung zum Front-Nano lesen
#  - Arduino-Zeilen parsen
#  - Daten threadsicher im Store ablegen
#
# Unterstützte Diagnosezeilen:
#  - #CMDP_BEGIN,id,vx,vy,wz,target
#  - #ODOM2,id,ms,path_cm,x_body_cm,y_body_cm,phi_deg
#  - #ODOM,ms,x_body_cm,y_body_cm,path_cm,phi_deg   (Altformat)
#  - #WHEELS,ms,...                                (Altformat)
#
# Wichtig:
#  - #ODOM2 ist das neue eindeutige Format.
#  - #ODOM bleibt nur als Fallback drin.
#  - serial_thread bleibt als Kompatibilitätsname für deine main.py erhalten.
# ============================================================

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock, Thread, Event
from typing import Optional, TextIO
import csv
import time


try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


@dataclass
class CmdpBegin:
    cmd_id: int
    vx_cms: float
    vy_cms: float
    wz_deg_s: float
    target: float


@dataclass
class Odom2Sample:
    cmd_id: int
    ms: float
    path_cm: float
    x_body_cm: float
    y_body_cm: float
    phi_deg: float


@dataclass
class OdomSample:
    ms: float
    x_body_cm: float
    y_body_cm: float
    path_cm: float
    phi_deg: float


@dataclass
class WheelSample:
    ms: float
    values: list[float]


class Store:
    def __init__(self, max_rows: int = 20000) -> None:
        self.max_rows = int(max_rows)

        self.lock = Lock()

        self.cmdp_by_id: dict[int, CmdpBegin] = {}
        self.cmdp_order: list[int] = []

        self.odom2_rows: list[Odom2Sample] = []
        self.odom_rows: list[OdomSample] = []
        self.wheels_rows: list[WheelSample] = []

        self.info_lines: list[str] = []
        self.status_lines: list[str] = []
        self.raw_lines: list[str] = []

        self.last_line: str = ""
        self.line_count: int = 0

    def clear(self) -> None:
        with self.lock:
            self.cmdp_by_id.clear()
            self.cmdp_order.clear()
            self.odom2_rows.clear()
            self.odom_rows.clear()
            self.wheels_rows.clear()
            self.info_lines.clear()
            self.status_lines.clear()
            self.raw_lines.clear()
            self.last_line = ""
            self.line_count = 0

    def _trim(self) -> None:
        if len(self.odom2_rows) > self.max_rows:
            self.odom2_rows = self.odom2_rows[-self.max_rows:]

        if len(self.odom_rows) > self.max_rows:
            self.odom_rows = self.odom_rows[-self.max_rows:]

        if len(self.wheels_rows) > self.max_rows:
            self.wheels_rows = self.wheels_rows[-self.max_rows:]

        if len(self.raw_lines) > self.max_rows:
            self.raw_lines = self.raw_lines[-self.max_rows:]

        if len(self.info_lines) > self.max_rows:
            self.info_lines = self.info_lines[-self.max_rows:]

        if len(self.status_lines) > self.max_rows:
            self.status_lines = self.status_lines[-self.max_rows:]

    def add_cmdp_begin(self, msg: CmdpBegin) -> None:
        with self.lock:
            if msg.cmd_id not in self.cmdp_by_id:
                self.cmdp_order.append(msg.cmd_id)

            self.cmdp_by_id[msg.cmd_id] = msg
            self._trim()

    def add_odom2(self, row: Odom2Sample) -> None:
        with self.lock:
            self.odom2_rows.append(row)
            self._trim()

    def add_odom(self, row: OdomSample) -> None:
        with self.lock:
            self.odom_rows.append(row)
            self._trim()

    def add_wheels(self, row: WheelSample) -> None:
        with self.lock:
            self.wheels_rows.append(row)
            self._trim()

    def add_info(self, line: str) -> None:
        with self.lock:
            self.info_lines.append(line)
            self._trim()

    def add_status(self, line: str) -> None:
        with self.lock:
            self.status_lines.append(line)
            self._trim()

    def add_raw(self, line: str) -> None:
        with self.lock:
            self.raw_lines.append(line)
            self.last_line = line
            self.line_count += 1
            self._trim()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "cmdp_by_id": dict(self.cmdp_by_id),
                "cmdp_order": list(self.cmdp_order),
                "odom2_rows": list(self.odom2_rows),
                "odom_rows": list(self.odom_rows),
                "wheels_rows": list(self.wheels_rows),
                "info_lines": list(self.info_lines),
                "status_lines": list(self.status_lines),
                "raw_lines": list(self.raw_lines),
                "last_line": self.last_line,
                "line_count": self.line_count,
            }


SerialStore = Store


def make_store(max_rows: int = 20000) -> Store:
    return Store(max_rows=max_rows)


def _to_float(text: str) -> float:
    return float(text.strip())


def _to_int(text: str) -> int:
    return int(text.strip())


def parse_line(line: str, store: Store) -> bool:
    line = line.strip()

    if not line:
        return False

    store.add_raw(line)

    try:
        if line.startswith("#CMDP_BEGIN,"):
            # #CMDP_BEGIN,id,vx,vy,wz,target
            parts = line.split(",")

            if len(parts) != 6:
                return False

            msg = CmdpBegin(
                cmd_id=_to_int(parts[1]),
                vx_cms=_to_float(parts[2]),
                vy_cms=_to_float(parts[3]),
                wz_deg_s=_to_float(parts[4]),
                target=_to_float(parts[5]),
            )

            store.add_cmdp_begin(msg)
            return True

        if line.startswith("#ODOM2,"):
            # #ODOM2,id,ms,path_cm,x_body_cm,y_body_cm,phi_deg
            parts = line.split(",")

            if len(parts) != 7:
                return False

            row = Odom2Sample(
                cmd_id=_to_int(parts[1]),
                ms=_to_float(parts[2]),
                path_cm=_to_float(parts[3]),
                x_body_cm=_to_float(parts[4]),
                y_body_cm=_to_float(parts[5]),
                phi_deg=_to_float(parts[6]),
            )

            store.add_odom2(row)
            return True

        if line.startswith("#ODOM,"):
            # Altformat:
            # #ODOM,ms,x_body_cm,y_body_cm,path_cm,phi_deg
            parts = line.split(",")

            if len(parts) != 6:
                return False

            row = OdomSample(
                ms=_to_float(parts[1]),
                x_body_cm=_to_float(parts[2]),
                y_body_cm=_to_float(parts[3]),
                path_cm=_to_float(parts[4]),
                phi_deg=_to_float(parts[5]),
            )

            store.add_odom(row)
            return True

        if line.startswith("#WHEELS,"):
            parts = line.split(",")

            if len(parts) < 2:
                return False

            row = WheelSample(
                ms=_to_float(parts[1]),
                values=[_to_float(p) for p in parts[2:]],
            )

            store.add_wheels(row)
            return True

        if line.startswith("#INFO,"):
            store.add_info(line)
            return True

        if (
            line.startswith("#WAIT")
            or line.startswith("#CON")
            or line.startswith("#HS")
            or line.startswith("#DIS")
        ):
            store.add_status(line)
            return True

        if line in ("PING", "PONG", "ACK", "KA"):
            store.add_status(line)
            return True

        if line.startswith("VSOL,") or line.startswith("VSOL_OK,") or line.startswith("VIST,"):
            store.add_status(line)
            return True

        if line.startswith("#"):
            store.add_status(line)
            return True

        return False

    except (ValueError, IndexError):
        return False


def list_serial_ports() -> list[str]:
    if list_ports is None:
        return []

    return [p.device for p in list_ports.comports()]


def open_serial(port: str, baud: int, timeout: float = 0.2):
    if serial is None:
        raise RuntimeError("pyserial ist nicht installiert. Installiere es mit: pip install pyserial")

    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def _write_csv_header(writer: csv.writer) -> None:
    writer.writerow(["line"])


def serial_reader_worker(
    port: str,
    baud: int,
    store: Store,
    stop_event: Optional[Event] = None,
    csv_file: Optional[TextIO] = None,
    echo: bool = True,
) -> None:
    if stop_event is None:
        stop_event = Event()

    csv_writer: Optional[csv.writer] = None

    if csv_file is not None:
        csv_writer = csv.writer(csv_file, lineterminator="\n")
        _write_csv_header(csv_writer)

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


def serial_thread(*args, **kwargs) -> None:
    """
    Kompatibilitätsfunktion für ältere main.py-Versionen.

    Unterstützte Aufrufe:

      serial_thread(port, baud, store, stop_event=None, csv_file=None, echo=True)

    und zusätzlich:

      serial_thread(store, port, baud, stop_event=None, csv_file=None, echo=True)

    Damit bleibt die alte Zeile in main.py gültig:

      from serial_io import serial_thread
    """

    if len(args) >= 3 and isinstance(args[0], Store):
        store = args[0]
        port = args[1]
        baud = args[2]
        rest = args[3:]

        serial_reader_worker(
            port,
            baud,
            store,
            *rest,
            **kwargs,
        )
        return

    serial_reader_worker(*args, **kwargs)


def start_serial_thread(
    port: str,
    baud: int,
    store: Store,
    stop_event: Optional[Event] = None,
    csv_file: Optional[TextIO] = None,
    echo: bool = True,
) -> tuple[Thread, Event]:
    if stop_event is None:
        stop_event = Event()

    thread = Thread(
        target=serial_reader_worker,
        args=(port, baud, store, stop_event, csv_file, echo),
        daemon=True,
    )

    thread.start()
    return thread, stop_event


# Kompatibilitätsalias für mögliche ältere main.py-Versionen
start_reader_thread = start_serial_thread