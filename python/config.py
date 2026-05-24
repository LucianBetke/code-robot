# ============================================================
# Datei: config.py
# Zweck:
#   - Zentrale Einstellungen fuer den Robot Monitor
#   - COM-Port, Baudrate, Plotmodi
#   - Spaltennamen fuer WHEELS / ODOM / CHASSIS
#   - Layout- und ODOM-Konstanten
#
# Diese Datei enthaelt keine Logik.
# Hier stehen nur feste Einstellungen.
# ============================================================


# ============================================================
# Serial-Einstellungen
# ============================================================

DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200


# ============================================================
# Datenpuffer / Aktualisierung
# ============================================================

MAX_POINTS = 3000
UPDATE_MS = 200


# ============================================================
# Standard-Plotmodus
# ============================================================

DEFAULT_DISPLAY_MODE = "ODOM"

PLOT_MODES = (
    "WHEELS",
    "ODOM",
    "CHASSIS",
)


# ============================================================
# ODOM-Erkennung fuer neue CMDP-Teilstuecke
# ============================================================

# Wenn path_cm um mehr als diesen Wert kleiner wird,
# erkennt Python: Arduino hat bei einem neuen CMDP wieder bei 0 angefangen.
ODOM_RESET_DROP_CM = 10.0

# Wenn ms um mehr als diesen Wert kleiner wird,
# erkennt Python ebenfalls einen neuen Fahrabschnitt.
ODOM_RESET_TIME_DROP_MS = 500.0

# Ab wie vielen Messpunkten ODOM ueberhaupt gezeichnet wird.
ODOM_MIN_RUN_POINTS = 2

# Anzahl der Beschriftungen auf der oberen Zeitachse im ODOM-Plot.
ODOM_TIME_AXIS_TICKS = 6


# ============================================================
# Zeitachsen-Erkennung fuer allgemeine Plotzeit
# ============================================================

# Wenn der Arduino-ms-Wert deutlich zurueckspringt,
# setzt Python t_plot_ms fortlaufend zusammen.
PLOT_TIME_RESET_DROP_MS = 250.0


# ============================================================
# ODOM-Statuszeile
# ============================================================

ODOM_STATUS_TEXT_Y = 0.925
ODOM_STATUS_TEXT_FONTSIZE = 10


# ============================================================
# Plot-Fenstergroessen
# ============================================================

FIGSIZE_ODOM = (11, 8)
FIGSIZE_WHEELS = (11, 6)
FIGSIZE_CHASSIS = (11, 6)


# ============================================================
# Standard-Spalten fuer WHEELS
# ============================================================

DEFAULT_WHEELS_COLS = [
    "ms",

    "VoLi_s",
    "VoLi_i",
    "VoLi_pwm",

    "VoRe_s",
    "VoRe_i",
    "VoRe_pwm",

    "HiLi_s",
    "HiLi_i",
    "HiLi_pwm",

    "HiRe_s",
    "HiRe_i",
    "HiRe_pwm",
]


# ============================================================
# Standard-Spalten fuer ODOM
# ============================================================

DEFAULT_ODOM_COLS = [
    "ms",
    "x_cm",
    "y_cm",
    "path_cm",
    "phi_deg",
]


# ============================================================
# Standard-Spalten fuer CHASSIS
# ============================================================
#
# Aktuell leer.
# CHASSIS kann spaeter ergaenzt werden, wenn dein Arduino
# dafuer ein festes Format liefert.
# ============================================================

DEFAULT_CHASSIS_COLS = []


# ============================================================
# Zeilen, die im Serial-Parser nicht als Messdaten gelten
# ============================================================

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