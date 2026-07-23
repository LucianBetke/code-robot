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


@dataclass
class UsSample:
    # Ein Ultraschall-Frame vom vorderen Nano.
    # Ungueltige Kanaele (validMask-Bit nicht gesetzt) werden als
    # None gefuehrt, damit im Plot eine Luecke entsteht statt eines
    # veralteten Werts.
    seq: int
    front_mm: Optional[float]
    left_mm: Optional[float]
    right_mm: Optional[float]
    # Weg [cm] aus dem zuletzt empfangenen #ODOM. None, solange noch
    # kein ODOM kam. cmd_id erlaubt spaeter das Trennen mehrerer Laeufe.
    path_cm: Optional[float] = None
    cmd_id: int = -1


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
        self.us_rows: list[UsSample] = []

        # cmd_id des zuletzt empfangenen CMDP_BEGIN.
        # Wird WHEELS-Frames mitgegeben, damit Settle/Lauf unterscheidbar sind.
        self.current_cmd_id: int = -1

        # Weg [cm] aus dem zuletzt empfangenen #ODOM. Wird jedem
        # US-Frame mitgegeben, damit die US-Ansicht ueber den Weg
        # geplottet werden kann.
        self.current_path_cm: Optional[float] = None

    def clear(self) -> None:
        with self.lock:
            self.cmdp_by_id.clear()
            self.cmdp_order.clear()
            self.odom_rows.clear()
            self.wheels_rows.clear()
            self.us_rows.clear()
            self.current_cmd_id = -1
            self.current_path_cm = None

    def _trim(self) -> None:
        if len(self.odom_rows) > self.max_rows:
            self.odom_rows = self.odom_rows[-self.max_rows:]
        if len(self.wheels_rows) > self.max_rows:
            self.wheels_rows = self.wheels_rows[-self.max_rows:]
        if len(self.us_rows) > self.max_rows:
            self.us_rows = self.us_rows[-self.max_rows:]

    def add_cmdp_begin(self, msg: CmdpBegin) -> None:
        with self.lock:
            if msg.cmd_id not in self.cmdp_by_id:
                self.cmdp_order.append(msg.cmd_id)
            self.cmdp_by_id[msg.cmd_id] = msg
            self.current_cmd_id = msg.cmd_id

    def add_odom(self, row: OdomSample) -> None:
        with self.lock:
            self.odom_rows.append(row)
            self.current_path_cm = row.path_cm
            self._trim()

    def add_wheels(self, ms: float, values: list[float]) -> None:
        with self.lock:
            self.wheels_rows.append(WheelSample(
                cmd_id=self.current_cmd_id,
                ms=ms,
                values=values,
            ))
            self._trim()

    def add_us(self, row: UsSample) -> None:
        with self.lock:
            # Weg und Lauf-Zuordnung aus dem aktuellen Stand ergaenzen.
            row.path_cm = self.current_path_cm
            row.cmd_id = self.current_cmd_id
            self.us_rows.append(row)
            self._trim()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "cmdp_by_id": dict(self.cmdp_by_id),
                "cmdp_order": list(self.cmdp_order),
                "odom_rows": list(self.odom_rows),
                "wheels_rows": list(self.wheels_rows),
                "us_rows": list(self.us_rows),
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

        if line.startswith("#US,"):
            # Format: #US,seq,frontMm,leftMm,rightMm,validMask,fAge,lAge,rAge
            parts = line.split(",")
            if len(parts) < 6:
                return False

            seq  = int(parts[1])
            fmm  = int(parts[2])
            lmm  = int(parts[3])
            rmm  = int(parts[4])
            mask = int(parts[5])

            # Bit 0 = Front, Bit 1 = Links, Bit 2 = Rechts.
            store.add_us(UsSample(
                seq      = seq,
                front_mm = fmm if (mask & 0x01) else None,
                left_mm  = lmm if (mask & 0x02) else None,
                right_mm = rmm if (mask & 0x04) else None,
            ))
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
    csv_path: Optional[str] = None,
) -> None:
    """Liest serielle Zeilen, schreibt sie ins CSV und parst sie in den Store.

    CSV-Handhabung:
        Wird csv_path uebergeben, oeffnet dieser Thread die Datei ERST,
        nachdem der Port erfolgreich geoeffnet wurde. Dadurch wird eine
        vorhandene Aufzeichnung nicht mehr geleert, wenn die Verbindung
        gar nicht zustande kommt (z. B. Port belegt). Die Datei wird am
        Ende hier wieder geschlossen.

        Der alte Weg ueber ein bereits offenes csv_file bleibt aus
        Kompatibilitaet erhalten.
    """
    if stop_event is None:
        stop_event = Event()

    print("Opening port")
    ser = open_serial(port, baud)
    print("Port open")

    # CSV erst jetzt oeffnen - nach erfolgreichem Portoeffnen.
    own_csv_file = None
    if csv_file is None and csv_path is not None:
        own_csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        csv_file = own_csv_file

    csv_writer = None
    if csv_file is not None:
        csv_writer = csv.writer(csv_file, lineterminator="\n")
        csv_writer.writerow(["line"])

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
            if own_csv_file is not None:
                own_csv_file.close()

# ============================================================
# CSV-Replay
# ============================================================

def replay_thread(
    csv_path: str,
    store: Store,
    stop_event: Optional[Event] = None,
    echo: bool = True,
    loop: bool = False,
) -> None:
    """Spielt eine gespeicherte robot_last.csv erneut in den Store.

    Die CSV wurde von serial_thread geschrieben: eine Kopfzeile "line"
    und danach je Zeile die rohe Arduino-Zeile (CSV-gequotet). Hier wird
    sie wieder ausgepackt und - wie beim Live-Betrieb - durch
    parse_line(...) in denselben Store geschoben. Alle Ansichten
    (US, ODOM, SPUR, WHEELS) funktionieren damit unveraendert.

    Die Datei wird ohne jede Verzoegerung durchgelesen. Der fertige
    Plot ist also sofort vollstaendig da.

    Parameter:
        loop: True = nach Dateiende von vorn beginnen (Store wird
              vorher geleert).
    """
    if stop_event is None:
        stop_event = Event()

    print(f"Replay: {csv_path}  (loop={loop})")

    while not stop_event.is_set():
        try:
            with open(csv_path, "r", newline="", encoding="utf-8") as f:
                reader = csv.reader(f)

                for row_index, row in enumerate(reader):
                    if stop_event.is_set():
                        break

                    if not row:
                        continue

                    line = row[0].strip()

                    # Kopfzeile ueberspringen.
                    if row_index == 0 and line == "line":
                        continue
                    if not line:
                        continue

                    if echo:
                        print(line)

                    parse_line(line, store)

        except FileNotFoundError:
            print(f"Replay: Datei nicht gefunden: {csv_path}")
            return

        if not loop:
            break

        # Vor dem naechsten Durchlauf den Store leeren, damit die
        # Laeufe nicht endlos aneinandergehaengt werden.
        store.clear()

    print("Replay beendet.")