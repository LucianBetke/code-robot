# ============================================================
# Datei: main.py
# Zweck:
#   - Startdatei fuer den Robot Monitor
#   - Liest Kommandozeilenargumente
#   - Fragt Plotmodus ab
#   - Startet Serial-Thread
#   - Startet Live-Plot
#
# Startbeispiele:
#   py main.py
#   py main.py --mode ODOM
#   py main.py --mode WHEELS
#   py main.py --port COM7 --mode ODOM
#   py main.py --list
# ============================================================

import argparse
import threading
from datetime import datetime

from config import DEFAULT_PORT
from config import DEFAULT_BAUD
from config import DEFAULT_DISPLAY_MODE
from config import MAX_POINTS
from config import UPDATE_MS

from serial_io import make_store
from serial_io import serial_thread


# ============================================================
# Startauswahl Plotmodus
# ============================================================

def ask_display_mode(default_mode: str = DEFAULT_DISPLAY_MODE) -> str:
    default_mode = default_mode.upper()

    print()
    print("Welche Daten willst du anschauen?")
    print("  1 = WHEELS  - Rad-Soll/Ist und PWM")
    print("  2 = ODOM    - Odometrie / Weg / Verdrehung")
    print(f"  Enter = Standard [{default_mode}]")
    print()

    while True:
        choice = input("Auswahl [1/2]: ").strip().lower()

        if choice == "":
            return default_mode

        if choice in ("1", "w", "wheel", "wheels", "rad", "raeder", "räder"):
            return "WHEELS"

        if choice in ("2", "o", "odom", "odometrie", "weg"):
            return "ODOM"

        print("Ungueltige Auswahl. Bitte 1 fuer WHEELS oder 2 fuer ODOM eingeben.")


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
        help="CSV-Dateiname, sonst automatisch robot_MODE_DATUM_UHRZEIT.csv"
    )

    parser.add_argument(
        "--mode",
        "-m",
        default=None,
        choices=[
            "ODOM", "WHEELS", "CHASSIS", "AUTO",
            "odom", "wheels", "chassis", "auto"
        ],
        help="Plotmodus direkt vorgeben: ODOM, WHEELS, CHASSIS oder AUTO"
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

    args = parser.parse_args()

    if args.list:
        print_available_ports()
        return

    display_mode = resolve_display_mode(args.mode)

    if display_mode == "AUTO":
        display_mode = "ODOM"

    if args.csv is not None:
        csv_path = args.csv
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = f"robot_{display_mode.lower()}_{timestamp}.csv"

    store = make_store(MAX_POINTS)
    stop_event = threading.Event()

    csv_file = open(csv_path, "w", newline="", encoding="utf-8")

    thread = threading.Thread(
        target=serial_thread,
        args=(
            args.port,
            args.baud,
            store,
            stop_event,
            csv_file,
            True
        ),
        daemon=True
    )

    try:
        thread.start()

        if args.no_plot:
            print("Kein Plot aktiv. STRG+C zum Beenden.")
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

        csv_file.close()


if __name__ == "__main__":
    main()