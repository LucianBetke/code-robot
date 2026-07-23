# ============================================================
# Datei: main.py
# Zweck:
#   - Startdatei fuer den Robot Monitor
#   - Liest Kommandozeilenargumente
#   - Fragt Datenquelle ab (Live vom Port oder CSV-Replay)
#   - Fragt Plotmodus ab
#   - Startet Serial-Thread bzw. Replay-Thread
#   - Startet Live-Plot
#
# Startbeispiele:
#   py main.py
#   py main.py --mode ODOM
#   py main.py --mode SPUR
#   py main.py --mode WHEELS
#   py main.py --port COM7 --mode ODOM
#   py main.py --csv mein_test.csv
#   py main.py --replay robot_last.csv --mode US
#   py main.py --replay robot_last.csv --loop
#   py main.py --list
#
# Ohne Argumente fragt das Programm nach dem Start zuerst nach der
# Datenquelle (Live oder CSV) und danach nach dem Plotmodus. Es ist
# also kein Kommandozeilen-Argument noetig; der gruene Start-Knopf in
# Visual Studio genuegt.
# ============================================================

import argparse
import os
import threading

from config import DEFAULT_PORT
from config import DEFAULT_BAUD
from config import DEFAULT_DISPLAY_MODE
from config import MAX_POINTS
from config import UPDATE_MS

from serial_io import make_store
from serial_io import serial_thread
from serial_io import replay_thread


# ============================================================
# Standard-CSV-Name
# ============================================================
# Diese Datei wird im Live-Betrieb beschrieben und im Replay-Betrieb
# gelesen, wenn beim Abfragen keine andere angegeben wird.

DEFAULT_CSV_NAME = "robot_last.csv"


# ============================================================
# Startauswahl Datenquelle
# ============================================================

def ask_data_source(default_csv: str = DEFAULT_CSV_NAME):
    """Fragt, ob live vom Port oder aus einer CSV gelesen wird.

    Rueckgabe:
        None                -> Live vom Port
        "<pfad zur csv>"    -> Replay dieser CSV
    """
    print()
    print("Woher sollen die Daten kommen?")
    print("  1 = LIVE    - direkt vom Roboter (COM-Port)")
    print(f"  2 = CSV     - gespeicherte Datei erneut abspielen [{default_csv}]")
    print("  Enter = Standard [LIVE]")
    print()

    while True:
        choice = input("Auswahl [1/2]: ").strip().lower()

        if choice in ("", "1", "l", "live", "port", "com"):
            return None

        if choice in ("2", "c", "csv", "replay", "datei", "file"):
            # Optional anderen Dateinamen zulassen. Enter = Standard.
            name = input(
                f"CSV-Datei [{default_csv}]: "
            ).strip()

            if name == "":
                name = default_csv

            if not os.path.isfile(name):
                print(f"Datei nicht gefunden: {name}")
                print("Bitte erneut waehlen.")
                continue

            return name

        print("Ungueltige Auswahl. Bitte 1 fuer LIVE oder 2 fuer CSV eingeben.")


# ============================================================
# Startauswahl Plotmodus
# ============================================================

def ask_display_mode(default_mode: str = DEFAULT_DISPLAY_MODE) -> str:
    default_mode = default_mode.upper()

    print()
    print("Welche Daten willst du anschauen?")
    print("  1 = WHEELS  - Rad-Soll/Ist und PWM")
    print("  2 = ODOM    - Fehlerauswertung: e_quer / e_laengs / phi")
    print("  3 = SPUR    - XY-Fahrweg + Querfehler + phi")
    print("  4 = US      - Abstaende Front / Links / Rechts")
    print(f"  Enter = Standard [{default_mode}]")
    print()

    while True:
        choice = input("Auswahl [1/2/3/4]: ").strip().lower()

        if choice == "":
            return default_mode

        if choice in ("1", "w", "wheel", "wheels", "rad", "raeder", "räder"):
            return "WHEELS"

        if choice in ("2", "o", "odom", "odometrie", "fehler"):
            return "ODOM"

        if choice in ("3", "s", "spur", "track", "xy", "fahrweg"):
            return "SPUR"

        if choice in ("4", "u", "us", "ultraschall", "abstand", "abstaende", "abstände"):
            return "US"

        print("Ungueltige Auswahl. Bitte 1 fuer WHEELS, 2 fuer ODOM, 3 fuer SPUR oder 4 fuer US eingeben.")


def resolve_display_mode(mode_from_args) -> str:
    if mode_from_args is not None:
        return mode_from_args.upper()

    return ask_display_mode(DEFAULT_DISPLAY_MODE)


# ============================================================
# COM-Ports anzeigen
# ============================================================

def print_available_ports():
    print("Verfuegbare COM-Ports:")

    try:
        import serial.tools.list_ports
    except ImportError:
        print("pyserial ist nicht installiert. Installiere es mit: pip install pyserial")
        return

    for pt in serial.tools.list_ports.comports():
        print(f"  {pt.device:12s} {pt.description}")


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Robot Serial Monitor fuer vorne-Arduino"
    )

    parser.add_argument(
        "--port",
        "-p",
        default=DEFAULT_PORT,
        help=f"COM-Port des vorne-Arduino (Standard: {DEFAULT_PORT})"
    )

    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baudrate (Standard: {DEFAULT_BAUD})"
    )

    parser.add_argument(
        "--csv",
        "-c",
        default=None,
        help="CSV-Dateiname fuer den Live-Mitschnitt, sonst robot_last.csv"
    )

    parser.add_argument(
        "--mode",
        "-m",
        default=None,
        choices=[
            "ODOM", "WHEELS", "SPUR", "TRACK", "CHASSIS", "AUTO", "US",
            "odom", "wheels", "spur", "track", "chassis", "auto", "us"
        ],
        help="Plotmodus direkt vorgeben: ODOM, SPUR, WHEELS, US, CHASSIS oder AUTO"
    )

    parser.add_argument(
        "--list",
        "-l",
        action="store_true",
        help="Verfuegbare COM-Ports anzeigen"
    )

    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Nur Terminal, kein Live-Plot"
    )

    parser.add_argument(
        "--replay",
        "-r",
        default=None,
        help="CSV-Datei erneut abspielen statt live vom Port zu lesen"
    )

    parser.add_argument(
        "--loop",
        action="store_true",
        help="Replay in Endlosschleife (Store wird pro Durchlauf geleert)"
    )

    args = parser.parse_args()

    if args.list:
        print_available_ports()
        return

    # ========================================================
    # Datenquelle bestimmen
    # ========================================================
    # Vorrang hat das Kommandozeilen-Argument --replay. Ist es nicht
    # gesetzt, wird interaktiv gefragt: Live oder CSV. So braucht der
    # Start ueber den gruenen Knopf in Visual Studio kein Argument.
    # ========================================================

    if args.replay is not None:
        replay_path = args.replay
    else:
        replay_path = ask_data_source(DEFAULT_CSV_NAME)

    display_mode = resolve_display_mode(args.mode)

    if display_mode == "AUTO":
        display_mode = "ODOM"

    if display_mode == "TRACK":
        display_mode = "SPUR"

    # ========================================================
    # CSV-Datei fuer den Live-Mitschnitt
    # ========================================================
    # Standard:
    #   Es wird immer dieselbe Datei benutzt.
    #   Weil unten mit "w" geoeffnet wird, wird sie bei jedem Start
    #   ueberschrieben. Dadurch bleibt nur der letzte Lauf erhalten.
    #
    # Optional:
    #   Mit --csv dateiname.csv kann weiterhin ein anderer Name angegeben werden.
    #
    # Im Replay-Betrieb wird KEINE CSV zum Schreiben geoeffnet, sonst
    # wuerde die Quelldatei ueberschrieben.
    # ========================================================

    if args.csv is not None:
        csv_path = args.csv
    else:
        csv_path = DEFAULT_CSV_NAME

    store = make_store(MAX_POINTS)
    stop_event = threading.Event()

    if replay_path is not None:
        thread = threading.Thread(
            target=replay_thread,
            args=(
                replay_path,
                store,
                stop_event,
                True,
                args.loop,
            ),
            daemon=True
        )
    else:
        # Live-Betrieb: der Serial-Thread oeffnet die CSV erst nach
        # erfolgreichem Portoeffnen. So wird eine vorhandene Aufnahme
        # nicht geleert, wenn der Port belegt ist oder LIVE nur
        # versehentlich gewaehlt wurde.
        thread = threading.Thread(
            target=serial_thread,
            args=(
                args.port,
                args.baud,
                store,
                stop_event,
                None,       # csv_file: nicht vorab oeffnen
                True,       # echo
                csv_path,   # csv_path: Thread oeffnet selbst
            ),
            daemon=True
        )

    try:
        thread.start()

        if args.no_plot:
            print("Kein Plot aktiv. STRG+C zum Beenden.")
            if replay_path is None:
                print(f"CSV-Datei: {csv_path}")

            try:
                thread.join()
            except KeyboardInterrupt:
                pass
        else:
            # Wichtig:
            # plotter wird erst NACH der Modus-Auswahl importiert.
            # Dadurch kann ein Fehler in plotter.py / plot_odom.py
            # die Auswahlfrage nicht mehr verhindern.
            from plotter import start_plot

            start_plot(
                store,
                mode=display_mode,
                interval_ms=UPDATE_MS
            )

    finally:
        stop_event.set()

        try:
            thread.join(timeout=1.0)
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()