# ============================================================
# Datei: config.py
# Zweck: Zentrale Einstellungen fuer den Robot Monitor
# ============================================================

# Serial-Verbindung
DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200

# Datenpuffer und Plot-Aktualisierung
MAX_POINTS = 3000
UPDATE_MS  = 200

# Startansicht: "ODOM" oder "WHEELS"
DEFAULT_DISPLAY_MODE = "ODOM"