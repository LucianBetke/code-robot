# plotter.py
# ============================================================
# Aufgabe:
#  - Matplotlib-Fenster koordinieren
#  - WHEELS- und ODOM-Ansicht anzeigen
#  - ODOM nutzt #CMDP_BEGIN + #ODOM2 ueber plot_odom.py
#  - Matplotlib-Toolbar unten bleibt aktiv
#  - eigener kompakter Button "Kopieren" oben rechts
# ============================================================

from __future__ import annotations

import os
import subprocess
import sys
import tempfile

import matplotlib

# Klassische Matplotlib-Toolbar unten bleibt aktiv.
matplotlib.rcParams["toolbar"] = "toolbar2"

# Globale Schriftgroessen
matplotlib.rcParams["font.size"]        = 12
matplotlib.rcParams["axes.labelsize"]   = 13
matplotlib.rcParams["xtick.labelsize"]  = 11
matplotlib.rcParams["ytick.labelsize"]  = 11
matplotlib.rcParams["figure.titlesize"] = 16

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from plot_odom import update_odom_plot


# ============================================================
# Bildschirmgroesse / Fenster
# ============================================================

def _screen_size() -> tuple[int, int]:
    """Liefert die Bildschirmgroesse in logischen Pixeln.

    Tk-basiert, weil der Wert mit der DPI-Skalierung uebereinstimmen muss,
    die auch das Matplotlib-Fenster (TkAgg) verwendet. Ctypes/WinAPI liefert
    sonst physische Pixel und das Canvas wird groesser als das Fenster.
    """
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
    button_ax = fig.add_axes([0.915, 0.942, 0.065, 0.032])
    button = Button(button_ax, "Kopieren")
    button.on_clicked(lambda _event: _copy_figure_to_clipboard(fig))
    fig._robot_copy_button = button


def _add_status_text(fig) -> None:
    fig._robot_status_text = fig.text(
        0.025, 0.948, "",
        ha="left", va="center", fontsize=11,
    )


def _make_base_figure(title: str):
    w, h = _screen_size()

    # figsize passend zur Bildschirmaufloesung (logisch),
    # damit Canvas und Fenster den gleichen Massstab haben.
    # h - 95: Platz fuer Titelleiste und Toolbar.
    fig = plt.figure(figsize=(w / 100, max(760, h - 95) / 100), dpi=100)
    fig.canvas.manager.set_window_title(title)
    fig.suptitle(title, fontsize=16, y=0.990)

    _maximize_window(fig)
    _add_status_text(fig)
    _add_copy_button(fig)

    return fig


# ============================================================
# WHEELS
# ============================================================

WHEEL_NAMES = ("VoLi", "VoRe", "HiLi", "HiRe")  # 4 Raeder x 3 Werte (soll, ist, pwm) = 12 values


def _update_wheels_plot(axes, store) -> None:
    if len(axes) < 2:
        return

    ax_speed, ax_pwm = axes[0], axes[1]

    for ax in axes:
        ax.clear()
        ax.grid(True)

    ax_speed.set_ylabel("v [m/s]")
    ax_pwm.set_xlabel("Zeit [s]")
    ax_pwm.set_ylabel("PWM")

    rows = [r for r in store.snapshot().get("wheels_rows", []) if len(r.values) >= 12]

    if not rows:
        ax_speed.text(
            0.5, 0.5, "warte auf #WHEELS ...",
            transform=ax_speed.transAxes, ha="center", va="center",
        )
        return

    t = [r.ms * 0.001 for r in rows]

    for wheel_index, name in enumerate(WHEEL_NAMES):
        base = wheel_index * 3
        soll = [r.values[base + 0] for r in rows]
        ist  = [r.values[base + 1] for r in rows]
        pwm  = [r.values[base + 2] for r in rows]

        ax_speed.plot(t, soll, label=f"{name}_s", linewidth=1.7)
        ax_speed.plot(t, ist,  label=f"{name}_i", linewidth=1.7)
        ax_pwm.plot(t, pwm,    label=f"{name}_pwm", linewidth=1.7)

    ax_speed.legend(loc="upper right", fontsize=9)
    ax_pwm.legend(loc="upper right", fontsize=9)


# ============================================================
# Figure-Erzeugung
# ============================================================

def _make_odom_figure():
    fig = _make_base_figure("Robot Monitor - ODOM2")

    # [left, bottom, width, height]
    L, W, H = 0.070, 0.910, 0.245
    axes = [
        fig.add_axes([L, 0.645, W, H]),
        fig.add_axes([L, 0.365, W, H]),
        fig.add_axes([L, 0.085, W, H]),
    ]
    return fig, axes


def _make_wheels_figure():
    fig = _make_base_figure("Robot Monitor - WHEELS")

    L, W = 0.070, 0.910
    ax1 = fig.add_axes([L, 0.555, W, 0.335])
    ax2 = fig.add_axes([L, 0.110, W, 0.335], sharex=ax1)
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
    else:
        fig, axes = _make_odom_figure()
        def animate(_frame):
            update_odom_plot(axes, store)

    ani = FuncAnimation(fig, animate, interval=interval_ms, cache_frame_data=False)
    plt.show()
    return ani