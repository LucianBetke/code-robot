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
from plot_wheels import update_wheels_plot
from plot_us import update_us_plot


# ============================================================
# Anzeige-Konstanten
# ============================================================

WHEEL_LINEWIDTH = 2.4
WHEEL_SETPOINT_LINEWIDTH = 1.8

LEGEND_FONTSIZE = 14
AXIS_LABEL_FONTSIZE = 17
TICK_LABEL_FONTSIZE = 14
STATUS_FONTSIZE = 15


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
            update_wheels_plot(axes, store)

    elif mode == "SPUR":
        fig, axes = _make_spur_figure()

        def animate(_frame):
            update_spur_plot(axes, store)

    elif mode == "US":
        fig, axes = _make_us_figure()

        def animate(_frame):
            update_us_plot(axes, store)

    else:
        fig, axes = _make_odom_figure()

        def animate(_frame):
            update_odom_plot(axes, store)

    ani = FuncAnimation(fig, animate, interval=interval_ms, cache_frame_data=False)
    plt.show()
    return ani