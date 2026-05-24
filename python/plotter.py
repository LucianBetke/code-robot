# plotter.py
# ============================================================
# Aufgabe:
#  - Matplotlib-Fenster koordinieren
#  - WHEELS- und ODOM-Ansicht anzeigen
#  - ODOM nutzt #CMDP_BEGIN + #ODOM2 ueber plot_odom.py
#  - Matplotlib-Toolbar unten bleibt aktiv
#  - eigener kompakter Button "Kopieren" oben rechts
#  - stabiles manuelles Layout
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
matplotlib.rcParams["font.size"] = 12
matplotlib.rcParams["axes.labelsize"] = 13
matplotlib.rcParams["xtick.labelsize"] = 11
matplotlib.rcParams["ytick.labelsize"] = 11
matplotlib.rcParams["figure.titlesize"] = 16

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from plot_odom import update_odom_plot


# ============================================================
# Fenster / Bildschirm
# ============================================================

def _get_screen_size_fallback() -> tuple[int, int]:
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()

        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()

        root.destroy()

        return int(width), int(height)

    except Exception:
        return 1600, 900


def _set_window_size(fig, width: int, height: int) -> None:
    try:
        manager = fig.canvas.manager
        window = manager.window
    except Exception:
        return

    try:
        window.geometry(f"{width}x{height}+0+0")
        return
    except Exception:
        pass

    try:
        window.resize(width, height)
        window.move(0, 0)
        return
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
                [
                    "powershell",
                    "-NoProfile",
                    "-STA",
                    "-Command",
                    ps_script,
                ],
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
    # Sehr kompakter Button oben rechts.
    button_ax = fig.add_axes([0.915, 0.942, 0.065, 0.032])
    button = Button(button_ax, "Kopieren")

    def on_click(_event) -> None:
        _copy_figure_to_clipboard(fig)

    button.on_clicked(on_click)

    # Referenz behalten, sonst kann Matplotlib den Button entsorgen.
    fig._robot_copy_button = button


def _add_status_text(fig) -> None:
    status = fig.text(
        0.025,
        0.948,
        "",
        ha="left",
        va="center",
        fontsize=11,
    )

    fig._robot_status_text = status


def _make_base_figure(title: str):
    screen_w, screen_h = _get_screen_size_fallback()

    # Taskleiste + Matplotlib-Toolbar unten beruecksichtigen.
    win_w = screen_w
    win_h = max(760, screen_h - 95)

    dpi = 100
    fig_w = win_w / dpi
    fig_h = win_h / dpi

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
    fig.canvas.manager.set_window_title(title)

    fig.suptitle(title, fontsize=16, y=0.990)

    _set_window_size(fig, win_w, win_h)

    _add_status_text(fig)
    _add_copy_button(fig)

    return fig


# ============================================================
# WHEELS
# ============================================================

def _latest_wheel_rows(snapshot: dict):
    return snapshot.get("wheels_rows", [])


def _wheel_value(row, index: int, default: float = 0.0) -> float:
    values = getattr(row, "values", None)

    if values is None and isinstance(row, dict):
        values = row.get("values", [])

    if not values or index >= len(values):
        return default

    return float(values[index])


def _wheel_ms(row) -> float:
    if hasattr(row, "ms"):
        return float(getattr(row, "ms"))

    if isinstance(row, dict):
        return float(row.get("ms", 0.0))

    return 0.0


def _update_wheels_plot(axes, store) -> None:
    snapshot = store.snapshot()
    rows = _latest_wheel_rows(snapshot)

    for ax in axes:
        ax.clear()
        ax.grid(True)

    if len(axes) < 2:
        return

    ax_speed = axes[0]
    ax_pwm = axes[1]

    ax_speed.set_ylabel("v [m/s]")
    ax_pwm.set_xlabel("Zeit [s]")
    ax_pwm.set_ylabel("PWM")

    if not rows:
        ax_speed.text(
            0.5,
            0.5,
            "warte auf #WHEELS ...",
            transform=ax_speed.transAxes,
            ha="center",
            va="center",
        )
        return

    t = [_wheel_ms(r) * 0.001 for r in rows]

    names = ["VoLi", "VoRe", "HiLi", "HiRe"]

    for wheel_index, name in enumerate(names):
        base = wheel_index * 3

        soll = [_wheel_value(r, base + 0) for r in rows]
        ist = [_wheel_value(r, base + 1) for r in rows]
        pwm = [_wheel_value(r, base + 2) for r in rows]

        ax_speed.plot(t, soll, label=f"{name}_s", linewidth=1.7)
        ax_speed.plot(t, ist, label=f"{name}_i", linewidth=1.7)
        ax_pwm.plot(t, pwm, label=f"{name}_pwm", linewidth=1.7)

    ax_speed.legend(loc="upper right", fontsize=9)
    ax_pwm.legend(loc="upper right", fontsize=9)


# ============================================================
# Figure-Erzeugung
# ============================================================

def _make_odom_figure():
    fig = _make_base_figure("Robot Monitor - ODOM2")

    # Manuelles Layout:
    # [left, bottom, width, height]
    #
    # Kompakter als vorher:
    #   hoehe vorher: 0.205
    #   hoehe jetzt:  0.245
    #
    # Zwischenraeume werden kleiner, ohne dass Zeit/Weg-Achsen kollidieren.
    left = 0.070
    width = 0.910
    height = 0.245

    ax1 = fig.add_axes([left, 0.645, width, height])
    ax2 = fig.add_axes([left, 0.365, width, height])
    ax3 = fig.add_axes([left, 0.085, width, height])

    axes = [ax1, ax2, ax3]

    return fig, axes


def _make_wheels_figure():
    fig = _make_base_figure("Robot Monitor - WHEELS")

    left = 0.070
    width = 0.910

    ax1 = fig.add_axes([left, 0.555, width, 0.335])
    ax2 = fig.add_axes([left, 0.110, width, 0.335], sharex=ax1)

    axes = [ax1, ax2]

    return fig, axes


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

    ani = FuncAnimation(
        fig,
        animate,
        interval=interval_ms,
        cache_frame_data=False,
    )

    plt.show()
    return ani


run_plot = start_plot