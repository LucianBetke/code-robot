# plot_us.py
# ============================================================
# Aufgabe:
#  - US-Ansicht (Ultraschall-Abstaende Front / Links / Rechts)
#  - X-Achse: Weg [cm], durchlaufend ueber mehrere Laeufe
#  - Y-Achse: Abstand [cm]
#
# Diese Datei enthaelt nur die US-spezifische Zeichenlogik.
# Fenster, Button und Statuszeile werden von plotter.py gebaut;
# hier wird - wie bei plot_odom.py - nur update_us_plot(axes, store)
# aufgerufen.
# ============================================================

from __future__ import annotations

import math


# ============================================================
# Anzeige-Konstanten
# ============================================================

AXIS_LABEL_FONTSIZE = 17
TICK_LABEL_FONTSIZE = 14
LEGEND_FONTSIZE = 20

US_LINEWIDTH = 3.4

# Feste Farben pro Sensor, damit die Zuordnung immer gleich ist.
US_COLORS = {
    "front": "tab:blue",
    "left":  "tab:orange",
    "right": "tab:green",
}


# ============================================================
# Datenaufbereitung
# ============================================================

def _build_us_arrays(us_rows: list):
    """Baut die Plotlisten aus den US-Frames, X-Achse = Weg [cm].

    Jeder US-Frame traegt den Weg (path_cm) des zuletzt empfangenen
    #ODOM. Frames ohne Weg (noch kein ODOM) werden uebersprungen.

    Der Weg jedes Laufs beginnt am Arduino wieder bei 0. Damit die
    X-Achse durchlaeuft statt bei jedem Lauf zurueckzuspringen, wird
    - wie in der WHEELS-Ansicht - ein kumulativer Offset addiert:
    beim Laufwechsel kommt der letzte Weg des vorherigen Laufs oben
    drauf. An der Nahtstelle wird zusaetzlich eine NaN-Luecke gesetzt,
    damit keine Linie ueber den Sprung gezogen wird.

    Die Kanalwerte werden von mm in cm umgerechnet (Faktor 0.1).
    Ungueltige Kanalwerte sind als None gefuehrt und werden zu NaN.
    """
    s_cm  = []
    front = []
    left  = []
    right = []

    last_cmd_id = None
    last_raw_path = None      # roher path_cm des vorigen Frames
    cum_offset = 0.0          # aufaddierter Weg abgeschlossener Laeufe

    def as_nan_cm(v):
        return math.nan if v is None else float(v) * 0.1

    for row in us_rows:
        if row.path_cm is None:
            continue

        raw = float(row.path_cm)

        new_run = False
        if last_cmd_id is not None and row.cmd_id != last_cmd_id:
            new_run = True
        # Weg-Ruecksprung ohne cmd-Wechsel (z. B. Neustart) ebenfalls
        # als Lauftrenner behandeln.
        if last_raw_path is not None and raw < last_raw_path - 0.5:
            new_run = True

        if new_run:
            # Den bis hierhin gelaufenen Weg des vorigen Laufs
            # dauerhaft aufaddieren, dann Luecke setzen.
            if last_raw_path is not None:
                cum_offset += last_raw_path

            s_cm.append(math.nan)
            front.append(math.nan)
            left.append(math.nan)
            right.append(math.nan)

        s_cm.append(cum_offset + raw)
        front.append(as_nan_cm(row.front_mm))
        left.append(as_nan_cm(row.left_mm))
        right.append(as_nan_cm(row.right_mm))

        last_cmd_id = row.cmd_id
        last_raw_path = raw

    return s_cm, front, left, right


def _count_valid(values) -> int:
    return sum(1 for v in values if v is not None and not math.isnan(v))


def _last_valid(values):
    for v in reversed(values):
        if v is not None and not math.isnan(v):
            return v
    return None


# ============================================================
# Statuszeile
# ============================================================

def _update_us_status(fig, s_cm, front, left, right) -> None:
    status_text = getattr(fig, "_robot_status_text", None)
    if status_text is None:
        return

    # Statustext groesser und fett.
    status_text.set_fontsize(18)
    status_text.set_fontweight("bold")

    if not s_cm:
        status_text.set_text("US: warte auf #US + #ODOM ...")
        return

    def fmt(v):
        return "---" if v is None else f"{v:.1f}cm"

    status_text.set_text(
        f"US   "
        f"Front={fmt(_last_valid(front))}   "
        f"Links={fmt(_last_valid(left))}   "
        f"Rechts={fmt(_last_valid(right))}"
    )


# ============================================================
# Update
# ============================================================

def update_us_plot(axes, store) -> None:
    ax = axes[0]
    ax.clear()

    snap = store.snapshot()
    us_rows = snap.get("us_rows", [])

    fig = ax.figure

    s_cm, front, left, right = _build_us_arrays(us_rows)

    if not s_cm:
        _update_us_status(fig, [], [], [], [])
        ax.set_xlabel("Weg [cm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
        ax.set_ylabel("Abstand [cm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
        ax.grid(True)
        ax.text(
            0.5, 0.5, "warte auf #US + #ODOM ...",
            transform=ax.transAxes,
            ha="center", va="center", fontsize=16,
        )
        return

    _update_us_status(fig, s_cm, front, left, right)

    ax.plot(s_cm, front, label="Front", color=US_COLORS["front"],
            marker="o", markersize=3, linewidth=US_LINEWIDTH)
    ax.plot(s_cm, left, label="Links", color=US_COLORS["left"],
            marker="s", markersize=3, linewidth=US_LINEWIDTH)
    ax.plot(s_cm, right, label="Rechts", color=US_COLORS["right"],
            marker="^", markersize=3, linewidth=US_LINEWIDTH)

    ax.set_xlabel("Weg [cm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
    ax.set_ylabel("Abstand [cm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
    ax.grid(True)
    ax.tick_params(axis="both", labelsize=TICK_LABEL_FONTSIZE, pad=4)
    ax.margins(x=0.01)

    ax.legend(
        loc="upper right",
        fontsize=LEGEND_FONTSIZE,
        framealpha=0.90,
        borderpad=0.6,
        labelspacing=0.45,
        handlelength=2.8,
        prop={"size": LEGEND_FONTSIZE, "weight": "bold"},
    )