# plotter.py
# ============================================================
# Aufgabe:
#  - Matplotlib-Fenster koordinieren
#  - WHEELS- und ODOM-Ansicht anzeigen
#  - ODOM nutzt #CMDP_BEGIN + #ODOM2 ueber plot_odom.py
#  - Matplotlib-Toolbar unten bleibt aktiv
#  - eigener kompakter Button "Kopieren" oben rechts
#
# WHEELS-Plot (neu):
#  - X-Achse ist Weg [cm], analog zum ODOM-Plot.
#  - Pro WHEELS-Frame wird der zugehoerige path_cm aus dem ODOM2-Frame
#    mit gleicher cmd_id und gleichem ms geholt.
#  - Settle-Frames zwischen zwei CMDP_BEGIN-Bloecken (kein passender
#    ODOM2-Eintrag) werden weggefiltert.
#  - Beim Wechsel der cmd_id wird der Weg kumuliert und eine
#    NaN-Luecke in alle Linien eingefuegt.
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
matplotlib.rcParams["font.size"]        = 12
matplotlib.rcParams["axes.labelsize"]   = 13
matplotlib.rcParams["xtick.labelsize"]  = 11
matplotlib.rcParams["ytick.labelsize"]  = 11
matplotlib.rcParams["figure.titlesize"] = 16

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from plot_odom import update_odom_plot


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

WHEEL_NAMES = ("VoLi", "VoRe", "HiLi", "HiRe")
WHEEL_VALUE_COUNT = 12


def _filter_settle_frames(wheels_rows: list) -> list:
    """Filtert Settle-Frames zwischen zwei CMDP_BEGIN-Bloecken raus.

    Ein WHEELS-Frame gilt als Settle, sobald innerhalb der gleichen cmd_id
    ein ms-Rueckwaertssprung beobachtet wird. Alle weiteren Frames mit
    dieser cmd_id bis zum naechsten cmd_id-Wechsel werden ebenfalls
    verworfen.
    """
    result: list = []
    current_cmd: int | None = None
    current_max_ms = -1.0
    is_settle = False

    for row in wheels_rows:
        if row.cmd_id != current_cmd:
            current_cmd = row.cmd_id
            current_max_ms = row.ms
            is_settle = False
            result.append(row)
            continue

        if is_settle:
            continue

        if row.ms < current_max_ms - 1.0:
            is_settle = True
            continue

        current_max_ms = row.ms
        result.append(row)

    return result


def _build_wheels_plot_arrays(
    wheels_rows: list,
    odom2_rows: list,
    cmdp_order: list[int],
) -> tuple[list[float], list[float], list[list[float]]]:
    """Baut Weg-basierte WHEELS-Plotdaten mit NaN-Trennung zwischen Laeufen.

    Verknuepfung:
        Pro WHEELS-Frame wird (cmd_id, ms) im ODOM2-Mapping nachgeschlagen.

    Lauf-Wechsel:
        Wenn die cmd_id im Stream wechselt, werden der bis dahin erreichte
        path_cm bzw. die Zeit als Offset uebernommen und in alle Listen ein
        NaN eingefuegt, damit Matplotlib die Linien nicht ueber die Luecke
        hinweg verbindet.

    Rueckgabe:
        s_cm:             fortlaufender Weg [cm] inkl. NaN-Trenner
        t_s:              fortlaufende Zeit [s] inkl. NaN-Trenner (gleiche Laenge)
        values_by_index:  12 Listen passend zur Arduino-Reihenfolge
    """
    s_cm: list[float] = []
    t_s:  list[float] = []
    values_by_index: list[list[float]] = [
        [] for _ in range(WHEEL_VALUE_COUNT)
    ]

    if not wheels_rows:
        return s_cm, t_s, values_by_index

    # Settle-Frames vor dem Mapping entfernen, sonst matchen sie auf
    # ODOM2-Eintraege vom Anfang des gleichen Laufs.
    wheels_rows = _filter_settle_frames(wheels_rows)

    if not wheels_rows:
        return s_cm, t_s, values_by_index

    # Mapping (cmd_id, ms) -> path_cm aus ODOM2
    path_by_key: dict[tuple[int, float], float] = {}
    for row in odom2_rows:
        path_by_key[(row.cmd_id, row.ms)] = row.path_cm

    if not path_by_key:
        return s_cm, t_s, values_by_index

    # Endpunkt pro cmd_id (Weg und Zeit), um Lauf-Wechsel kumulativ zu offset-en
    last_path_by_id: dict[int, float] = {}
    last_ms_by_id:   dict[int, float] = {}
    for row in odom2_rows:
        last_path_by_id[row.cmd_id] = row.path_cm   # ueberschreibt -> Endwert
        last_ms_by_id[row.cmd_id]   = row.ms

    cum_offset_cm = 0.0
    cum_offset_s  = 0.0
    last_cmd_id: int | None = None

    for row in wheels_rows:
        if len(row.values) < WHEEL_VALUE_COUNT:
            continue

        key = (row.cmd_id, row.ms)
        local_path_cm = path_by_key.get(key)
        if local_path_cm is None:
            # Kein passender ODOM2-Frame -> ueberspringen
            continue

        if last_cmd_id is not None and row.cmd_id != last_cmd_id:
            # Lauf-Wechsel: Offsets um Endwerte des vorherigen Laufs erhoehen,
            # NaN-Luecke einfuegen.
            cum_offset_cm += last_path_by_id.get(last_cmd_id, 0.0)
            cum_offset_s  += last_ms_by_id.get(last_cmd_id, 0.0) * 0.001
            s_cm.append(math.nan)
            t_s.append(math.nan)
            for values in values_by_index:
                values.append(math.nan)

        s_cm.append(cum_offset_cm + local_path_cm)
        t_s.append(cum_offset_s + row.ms * 0.001)
        for index in range(WHEEL_VALUE_COUNT):
            values_by_index[index].append(float(row.values[index]))

        last_cmd_id = row.cmd_id

    return s_cm, t_s, values_by_index


def _update_wheels_status(fig, n_frames: int, s_cm: list[float]) -> None:
    status_text = getattr(fig, "_robot_status_text", None)
    if status_text is None:
        return

    if not s_cm:
        status_text.set_text(f"WHEELS: {n_frames} Frames")
        return

    valid_s = [x for x in s_cm if not math.isnan(x)]
    if not valid_s:
        status_text.set_text(f"WHEELS: {n_frames} Frames")
        return

    status_text.set_text(
        f"WHEELS: {n_frames} Frames   s={valid_s[-1]:.2f} cm"
    )


# ============================================================
# Achsenformat / Zeit-Sekundaerachse (analog plot_odom)
# ============================================================

def _strictly_increasing_arrays(
    x_values: list[float],
    y_values: list[float],
) -> tuple[np.ndarray, np.ndarray]:
    xs: list[float] = []
    ys: list[float] = []

    for x, y in zip(x_values, y_values):
        xf, yf = float(x), float(y)
        if xf != xf or yf != yf:  # NaN ueberspringen
            continue
        if not xs:
            xs.append(xf)
            ys.append(yf)
        elif xf > xs[-1] + 1.0e-9:
            xs.append(xf)
            ys.append(yf)
        else:
            ys[-1] = yf

    if len(xs) < 2:
        return np.array([0.0, 1.0]), np.array([0.0, 1.0])

    return np.array(xs), np.array(ys)


def _add_time_axis(ax, s_cm, t_s, show_label: bool = True) -> None:
    # Alte Sekundaerachsen entfernen, sonst stapeln sie sich bei Live-Animation.
    for child in list(ax.child_axes):
        child.remove()

    s_arr, t_arr = _strictly_increasing_arrays(s_cm, t_s)
    t_for_inverse, s_for_inverse = _strictly_increasing_arrays(t_s, s_cm)

    def s_to_t(x):
        return np.interp(x, s_arr, t_arr)

    def t_to_s(t):
        return np.interp(t, t_for_inverse, s_for_inverse)

    secax = ax.secondary_xaxis("top", functions=(s_to_t, t_to_s))
    secax.set_xlabel("Zeit [s]" if show_label else "", fontsize=15, labelpad=6)
    secax.tick_params(axis="x", labelsize=13, pad=3)


def _format_wheels_axis(ax, ylabel: str, xlabel: str = "") -> None:
    ax.set_ylabel(ylabel, fontsize=15, labelpad=6)
    ax.set_xlabel(xlabel, fontsize=15, labelpad=6)
    ax.grid(True)
    ax.tick_params(axis="both", labelsize=13, pad=3)


def _update_wheels_plot(axes, store) -> None:
    if len(axes) < 2:
        return

    ax_speed, ax_pwm = axes[0], axes[1]

    for ax in axes:
        ax.clear()

    snap = store.snapshot()
    wheels_rows = snap.get("wheels_rows", [])
    odom2_rows  = snap.get("odom2_rows", [])
    cmdp_order  = snap.get("cmdp_order", [])

    fig = ax_speed.figure

    if not wheels_rows:
        _format_wheels_axis(ax_speed, "v [m/s]")
        _format_wheels_axis(ax_pwm, "PWM", "Weg [cm]")
        _update_wheels_status(fig, 0, [])
        ax_speed.text(
            0.5, 0.5, "warte auf #WHEELS ...",
            transform=ax_speed.transAxes, ha="center", va="center", fontsize=13,
        )
        return

    s_cm, t_s, values_by_index = _build_wheels_plot_arrays(
        wheels_rows, odom2_rows, cmdp_order
    )

    _update_wheels_status(fig, len(wheels_rows), s_cm)

    if not s_cm:
        _format_wheels_axis(ax_speed, "v [m/s]")
        _format_wheels_axis(ax_pwm, "PWM", "Weg [cm]")
        ax_speed.text(
            0.5, 0.5, "warte auf passende #ODOM2 ...",
            transform=ax_speed.transAxes, ha="center", va="center", fontsize=13,
        )
        return

    for wheel_index, name in enumerate(WHEEL_NAMES):
        base = wheel_index * 3

        soll = values_by_index[base + 0]
        ist  = values_by_index[base + 1]
        pwm  = values_by_index[base + 2]

        ax_speed.plot(s_cm, soll, label=f"{name}_s", linewidth=1.7)
        ax_speed.plot(s_cm, ist,  label=f"{name}_i", linewidth=1.7)
        ax_pwm.plot(s_cm,   pwm,  label=f"{name}_pwm", linewidth=1.7)

    _format_wheels_axis(ax_speed, "v [m/s]")
    _format_wheels_axis(ax_pwm, "PWM", "Weg [cm]")

    ax_speed.legend(loc="upper right", fontsize=11)
    ax_pwm.legend(loc="upper right", fontsize=11)

    ax_speed.margins(x=0.01)
    ax_pwm.margins(x=0.01)

    # Zeit-Sekundaerachse oben, Label nur auf der oberen Achse.
    _add_time_axis(ax_speed, s_cm, t_s, show_label=True)
    _add_time_axis(ax_pwm,   s_cm, t_s, show_label=False)


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