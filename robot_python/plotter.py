# plotter.py
# ============================================================
# Aufgabe:
#  - Matplotlib-Fenster koordinieren
#  - WHEELS-, ODOM- und SPUR-Ansicht anzeigen
#
# WHEELS:
#  - X-Achse: Zeit [s]
#  - Rad-Soll/Ist bleiben in cm/s
#  - PWM separat unten
#  - braucht KEIN #ODOM
#
# ODOM:
#  - X-Achse: Weg [cm]
#  - nutzt #CMDP_BEGIN + #ODOM ueber plot_odom.py
#  - alte Fehlerauswertung bleibt erhalten
#
# SPUR:
#  - oben: XY-Fahrweg
#  - mitte: Querfehler
#  - unten: Verdrehwinkel
#
# Matplotlib-Toolbar unten bleibt aktiv
# eigener kompakter Button "Kopieren" oben rechts
# ============================================================

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import math

import matplotlib

# Klassische Matplotlib-Toolbar unten bleibt aktiv.
matplotlib.rcParams["toolbar"] = "toolbar2"

# Globale Schriftgroessen
matplotlib.rcParams["font.size"]        = 14
matplotlib.rcParams["axes.labelsize"]   = 17
matplotlib.rcParams["xtick.labelsize"]  = 14
matplotlib.rcParams["ytick.labelsize"]  = 14
matplotlib.rcParams["figure.titlesize"] = 18
matplotlib.rcParams["legend.fontsize"]  = 14

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from plot_odom import update_odom_plot
from plot_odom import update_spur_plot


# ============================================================
# Anzeige-Konstanten
# ============================================================

WHEEL_LINEWIDTH = 2.4
WHEEL_SETPOINT_LINEWIDTH = 1.8

LEGEND_FONTSIZE = 14
AXIS_LABEL_FONTSIZE = 17
TICK_LABEL_FONTSIZE = 14
STATUS_FONTSIZE = 12


# ============================================================
# Bildschirmgroesse / Fenster
# ============================================================

def _screen_size() -> tuple[int, int]:
    """Liefert die Bildschirmgroesse in logischen Pixeln."""
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        w = root.winfo_screenwidth()
        h = root.winfo_screenheight()
        root.destroy()
        return int(w), int(h)
    except Exception:
        return 1920, 1080


def _maximize_window(fig) -> None:
    """Setzt das Fenster in den echten Maximized-State."""
    try:
        window = fig.canvas.manager.window
    except Exception:
        return

    # TkAgg unter Windows
    try:
        window.state("zoomed")
        return
    except Exception:
        pass

    # QtAgg / Qt5Agg
    try:
        window.showMaximized()
        return
    except Exception:
        pass

    # TkAgg unter Linux
    try:
        window.attributes("-zoomed", True)
    except Exception:
        pass


# ============================================================
# Zwischenablage
# ============================================================

def _copy_figure_to_clipboard(fig) -> None:
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".png") as tmp:
            tmp_path = tmp.name

        fig.savefig(tmp_path, dpi=150)

        if sys.platform.startswith("win"):
            ps_script = (
                "Add-Type -AssemblyName System.Windows.Forms; "
                "Add-Type -AssemblyName System.Drawing; "
                f"$img=[System.Drawing.Image]::FromFile('{tmp_path}'); "
                "[System.Windows.Forms.Clipboard]::SetImage($img); "
                "$img.Dispose();"
            )
            subprocess.run(
                ["powershell", "-NoProfile", "-STA", "-Command", ps_script],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("Bild wurde in die Zwischenablage kopiert.")
        else:
            print(f"Bild wurde gespeichert: {tmp_path}")
            tmp_path = None

    except Exception as exc:
        print(f"Bild konnte nicht in die Zwischenablage kopiert werden: {exc}")

    finally:
        if tmp_path is not None:
            try:
                os.remove(tmp_path)
            except OSError:
                pass


# ============================================================
# Figure-Elemente
# ============================================================

def _add_copy_button(fig) -> None:
    button_ax = fig.add_axes([0.910, 0.940, 0.075, 0.038])
    button = Button(button_ax, "Kopieren")
    button.label.set_fontsize(12)
    button.on_clicked(lambda _event: _copy_figure_to_clipboard(fig))
    fig._robot_copy_button = button


def _add_status_text(fig) -> None:
    fig._robot_status_text = fig.text(
        0.025, 0.950, "",
        ha="left",
        va="center",
        fontsize=STATUS_FONTSIZE,
    )


def _make_base_figure(title: str):
    w, h = _screen_size()

    fig = plt.figure(figsize=(w / 100, max(760, h - 95) / 100), dpi=100)
    fig.canvas.manager.set_window_title(title)
    fig.suptitle(title, fontsize=18, y=0.992)

    _maximize_window(fig)
    _add_status_text(fig)
    _add_copy_button(fig)

    return fig


# ============================================================
# WHEELS
# ============================================================

# Reihenfolge muss zur Arduino-Ausgabe passen:
# Pro Rad: Soll, Ist, PWM
WHEEL_NAMES = ("VoLi", "VoRe", "HiLi", "HiRe")
WHEEL_VALUE_COUNT = 12

# Feste Farben pro Motor.
# Diese Farben gelten oben fuer Soll/Ist und unten fuer PWM gleich.
WHEEL_COLORS = {
    "VoLi": "tab:blue",
    "VoRe": "tab:orange",
    "HiLi": "tab:green",
    "HiRe": "tab:red",
}


def _build_wheels_time_plot_arrays(
    wheels_rows: list,
) -> tuple[list[float], list[list[float]]]:
    """Baut WHEELS-Plotdaten mit Zeitachse.

    Rueckgabe:
        t_s:             Zeit [s], inklusive NaN-Trenner bei Laufwechsel
        values_by_index: 12 Listen passend zur Arduino-Reihenfolge

    Vorteil:
        Diese Darstellung braucht kein #ODOM.
        Sie nutzt nur #WHEELS,ms,...
    """
    t_s: list[float] = []
    values_by_index: list[list[float]] = [
        [] for _ in range(WHEEL_VALUE_COUNT)
    ]

    if not wheels_rows:
        return t_s, values_by_index

    cum_offset_s = 0.0

    last_cmd_id: int | None = None
    last_ms: float | None = None

    for row in wheels_rows:
        if len(row.values) < WHEEL_VALUE_COUNT:
            continue

        current_cmd_id = row.cmd_id
        current_ms = float(row.ms)

        new_run = False

        if last_cmd_id is not None and current_cmd_id != last_cmd_id:
            new_run = True

        if last_ms is not None and current_ms < last_ms - 1.0:
            new_run = True

        if new_run:
            if last_ms is not None:
                cum_offset_s += last_ms * 0.001

            t_s.append(math.nan)
            for values in values_by_index:
                values.append(math.nan)

        t_s.append(cum_offset_s + current_ms * 0.001)

        for index in range(WHEEL_VALUE_COUNT):
            values_by_index[index].append(float(row.values[index]))

        last_cmd_id = current_cmd_id
        last_ms = current_ms

    return t_s, values_by_index


def _update_wheels_status(fig, n_frames: int, t_s: list[float]) -> None:
    status_text = getattr(fig, "_robot_status_text", None)
    if status_text is None:
        return

    if not t_s:
        status_text.set_text(f"WHEELS: {n_frames} Frames")
        return

    valid_t = [x for x in t_s if not math.isnan(x)]
    if not valid_t:
        status_text.set_text(f"WHEELS: {n_frames} Frames")
        return

    status_text.set_text(
        f"WHEELS: {n_frames} Frames   t={valid_t[-1]:.2f} s"
    )


def _format_wheels_axis(ax, ylabel: str, xlabel: str = "") -> None:
    ax.set_ylabel(ylabel, fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
    ax.set_xlabel(xlabel, fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
    ax.grid(True)
    ax.tick_params(
        axis="both",
        labelsize=TICK_LABEL_FONTSIZE,
        pad=4,
    )


def _update_wheels_plot(axes, store) -> None:
    if len(axes) < 2:
        return

    ax_speed, ax_pwm = axes[0], axes[1]

    for ax in axes:
        ax.clear()

    snap = store.snapshot()
    wheels_rows = snap.get("wheels_rows", [])

    fig = ax_speed.figure

    if not wheels_rows:
        _format_wheels_axis(ax_speed, "v [cm/s]")
        _format_wheels_axis(ax_pwm, "PWM", "Zeit [s]")
        _update_wheels_status(fig, 0, [])
        ax_speed.text(
            0.5, 0.5, "warte auf #WHEELS ...",
            transform=ax_speed.transAxes,
            ha="center",
            va="center",
            fontsize=16,
        )
        return

    t_s, values_by_index = _build_wheels_time_plot_arrays(wheels_rows)

    _update_wheels_status(fig, len(wheels_rows), t_s)

    if not t_s:
        _format_wheels_axis(ax_speed, "v [cm/s]")
        _format_wheels_axis(ax_pwm, "PWM", "Zeit [s]")
        ax_speed.text(
            0.5, 0.5, "keine gueltigen #WHEELS-Daten ...",
            transform=ax_speed.transAxes,
            ha="center",
            va="center",
            fontsize=16,
        )
        return

    for wheel_index, name in enumerate(WHEEL_NAMES):
        base = wheel_index * 3

        soll = values_by_index[base + 0]
        ist  = values_by_index[base + 1]
        pwm  = values_by_index[base + 2]

        color = WHEEL_COLORS[name]

        # Gleicher Motor = gleiche Farbe.
        # Sollwert: gestrichelt und etwas transparenter.
        # Istwert: durchgezogen.
        # PWM: unten gleiche Farbe wie oben.
        ax_speed.plot(
            t_s,
            soll,
            label=f"{name}_s",
            color=color,
            linestyle="--",
            alpha=0.55,
            linewidth=WHEEL_SETPOINT_LINEWIDTH,
        )

        ax_speed.plot(
            t_s,
            ist,
            label=f"{name}_i",
            color=color,
            linestyle="-",
            linewidth=WHEEL_LINEWIDTH,
        )

        ax_pwm.plot(
            t_s,
            pwm,
            label=f"{name}_pwm",
            color=color,
            linestyle="-",
            linewidth=WHEEL_LINEWIDTH,
        )

    _format_wheels_axis(ax_speed, "v [cm/s]")
    _format_wheels_axis(ax_pwm, "PWM", "Zeit [s]")

    ax_speed.legend(
        loc="upper right",
        fontsize=LEGEND_FONTSIZE,
        framealpha=0.90,
        borderpad=0.6,
        labelspacing=0.45,
        handlelength=2.8,
    )

    ax_pwm.legend(
        loc="upper right",
        fontsize=LEGEND_FONTSIZE,
        framealpha=0.90,
        borderpad=0.6,
        labelspacing=0.45,
        handlelength=2.8,
    )

    ax_speed.margins(x=0.01)
    ax_pwm.margins(x=0.01)


# ============================================================
# US - Ultraschall-Abstaende
# ============================================================

# Feste Farben pro Sensor, damit die Zuordnung immer gleich ist.
US_COLORS = {
    "front": "tab:blue",
    "left":  "tab:orange",
    "right": "tab:green",
}

US_LINEWIDTH = 2.2


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

    Ungueltige Kanalwerte sind als None gefuehrt und werden zu NaN.
    """
    s_cm  = []
    front = []
    left  = []
    right = []

    last_cmd_id = None
    last_raw_path = None      # roher path_cm des vorigen Frames
    cum_offset = 0.0          # aufaddierter Weg abgeschlossener Laeufe

    def as_nan(v):
        return math.nan if v is None else float(v)

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
        front.append(as_nan(row.front_mm))
        left.append(as_nan(row.left_mm))
        right.append(as_nan(row.right_mm))

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


def _update_us_status(fig, s_cm, front, left, right) -> None:
    status_text = getattr(fig, "_robot_status_text", None)
    if status_text is None:
        return

    if not s_cm:
        status_text.set_text("US: warte auf #US + #ODOM ...")
        return

    def fmt(v):
        return "---" if v is None else f"{int(v)}mm"

    valid_s = [x for x in s_cm if not math.isnan(x)]
    weg = f"{valid_s[-1]:.1f} cm" if valid_s else "-"

    status_text.set_text(
        f"US   Weg={weg}   "
        f"Front={fmt(_last_valid(front))}   "
        f"Links={fmt(_last_valid(left))}   "
        f"Rechts={fmt(_last_valid(right))}"
    )


def _update_us_plot(axes, store) -> None:
    ax = axes[0]
    ax.clear()

    snap = store.snapshot()
    us_rows = snap.get("us_rows", [])

    fig = ax.figure

    s_cm, front, left, right = _build_us_arrays(us_rows)

    if not s_cm:
        _update_us_status(fig, [], [], [], [])
        ax.set_xlabel("Weg [cm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
        ax.set_ylabel("Abstand [mm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
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
    ax.set_ylabel("Abstand [mm]", fontsize=AXIS_LABEL_FONTSIZE, labelpad=8)
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
    )


# ============================================================
# Figure-Erzeugung
# ============================================================

def _make_us_figure():
    fig = _make_base_figure("Robot Monitor - US")

    # Ein grosses Diagramm fuer die drei Abstaende.
    L, W = 0.060, 0.925
    ax = fig.add_axes([L, 0.100, W, 0.820])
    return fig, [ax]


def _make_odom_figure():
    fig = _make_base_figure("Robot Monitor - ODOM")

    # [left, bottom, width, height]
    # Alte ODOM-Fehlerauswertung bleibt unveraendert.
    L, W = 0.060, 0.925
    axes = [
        fig.add_axes([L, 0.690, W, 0.235]),
        fig.add_axes([L, 0.395, W, 0.235]),
        fig.add_axes([L, 0.100, W, 0.235]),
    ]
    return fig, axes


def _make_spur_figure():
    fig = _make_base_figure("Robot Monitor - SPUR")

    # [left, bottom, width, height]
    # Oben mehr Platz fuer die XY-Fahrspur.
    L, W = 0.070, 0.895

    axes = [
        fig.add_axes([L, 0.565, W, 0.360]),
        fig.add_axes([L, 0.335, W, 0.165]),
        fig.add_axes([L, 0.100, W, 0.165]),
    ]
    return fig, axes


def _make_wheels_figure():
    fig = _make_base_figure("Robot Monitor - WHEELS")

    # [left, bottom, width, height]
    # Groessere Diagramme, weniger Luft zwischen den beiden Achsen.
    L, W = 0.060, 0.925

    ax1 = fig.add_axes([L, 0.545, W, 0.380])
    ax2 = fig.add_axes([L, 0.090, W, 0.380], sharex=ax1)

    return fig, [ax1, ax2]


# ============================================================
# Start
# ============================================================

def start_plot(store, mode: str = "ODOM", interval_ms: int = 200):
    mode = (mode or "ODOM").upper()

    if mode == "WHEELS":
        fig, axes = _make_wheels_figure()

        def animate(_frame):
            _update_wheels_plot(axes, store)

    elif mode == "SPUR":
        fig, axes = _make_spur_figure()

        def animate(_frame):
            update_spur_plot(axes, store)

    elif mode == "US":
        fig, axes = _make_us_figure()

        def animate(_frame):
            _update_us_plot(axes, store)

    else:
        fig, axes = _make_odom_figure()

        def animate(_frame):
            update_odom_plot(axes, store)

    ani = FuncAnimation(fig, animate, interval=interval_ms, cache_frame_data=False)
    plt.show()
    return ani