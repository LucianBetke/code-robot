# ============================================================
# Datei: plotter.py
# Zweck:
#   - Live-Plot-Fenster fuer WHEELS / ODOM / CHASSIS
#   - matplotlib-Fenster, Animation, Layout
#   - Clipboard-Button
#   - WHEELS-Plot
#   - CHASSIS-Plot
#
# ODOM ist ausgelagert nach:
#   plot_odom.py
#
# Voraussetzung:
#   - config.py enthaelt alle Konstanten
#   - serial_io.py enthaelt die globale Variable store
#   - plot_odom.py enthaelt update_odom_plot()
# ============================================================

import os
import io

import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from config import *
from serial_io import store
from plot_odom import update_odom_plot


# ============================================================
# Farben
# ============================================================

COLORS_RAD = {
    "VoLi": "#1f77b4",
    "VoRe": "#ff7f0e",
    "HiLi": "#2ca02c",
    "HiRe": "#d62728",
}

COLORS_VEH = {
    "vx_i": "#9467bd",
    "vy_i": "#8c564b",
    "wz_i": "#e377c2",
}


# ============================================================
# WHEELS-Hilfsfunktion
# ============================================================

def build_command_boundary_series(t_sec, values, cmd_indices, local_ms):
    n = min(len(t_sec), len(values), len(cmd_indices))

    if n == 0:
        return [], []

    x = [t_sec[0]]
    y = [values[0]]

    for i in range(1, n):
        cmd_changed = cmd_indices[i] != cmd_indices[i - 1]

        if cmd_changed:
            boundary = t_sec[i]

            x.append(boundary)
            y.append(y[-1])

            x.append(boundary)
            y.append(values[i])

        x.append(t_sec[i])
        y.append(values[i])

    return x, y


# ============================================================
# Meldungen
# ============================================================

def show_message(title, text):
    print(f"[{title}] {text}")

    try:
        import tkinter.messagebox as messagebox
        messagebox.showinfo(title, text)
    except Exception:
        pass


# ============================================================
# Plot als Bild erzeugen
# ============================================================

def figure_to_pil_image(fig, dpi=150):
    from PIL import Image

    fig.canvas.draw()

    buffer = io.BytesIO()

    fig.savefig(
        buffer,
        format="png",
        dpi=dpi,
        facecolor="white"
    )

    buffer.seek(0)

    image = Image.open(buffer).convert("RGB")
    return image


# ============================================================
# Bild in Zwischenablage kopieren
# ============================================================

def copy_plot_to_clipboard(fig):
    if os.name != "nt":
        show_message(
            "Clipboard",
            "Bild-Zwischenablage ist hier nur fuer Windows eingebaut."
        )
        return

    try:
        import win32clipboard
        import win32con
    except ImportError:
        show_message(
            "Clipboard",
            "Fehlende Pakete. Bitte ausfuehren:\n\npy -m pip install --upgrade pillow pywin32"
        )
        return

    try:
        image = figure_to_pil_image(fig, dpi=150)

        bmp_buffer = io.BytesIO()
        image.save(bmp_buffer, "BMP")

        dib_data = bmp_buffer.getvalue()[14:]

        win32clipboard.OpenClipboard()

        try:
            win32clipboard.EmptyClipboard()
            win32clipboard.SetClipboardData(win32con.CF_DIB, dib_data)
        finally:
            win32clipboard.CloseClipboard()

        show_message(
            "Clipboard",
            "Plot wurde als Bild in die Zwischenablage kopiert."
        )

    except Exception as e:
        show_message(
            "Clipboard Fehler",
            f"Kopieren fehlgeschlagen:\n\n{e}"
        )


def add_extra_plot_buttons(fig):
    try:
        import tkinter as tk

        manager = plt.get_current_fig_manager()
        window = manager.window

        button_frame = tk.Frame(window, bd=1, relief=tk.GROOVE)

        btn_copy = tk.Button(
            button_frame,
            text="Bild kopieren",
            command=lambda: copy_plot_to_clipboard(fig),
            width=18
        )

        btn_copy.pack(side=tk.LEFT, padx=4, pady=3)

        button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        print("[Plot] Zusatzbutton geladen: Bild kopieren")

    except Exception as e:
        print(f"[Plot] Zusatzbutton konnte nicht geladen werden: {e}")


# ============================================================
# Layout-Auswahl
# ============================================================

def get_plot_layout(display_mode: str):
    mode = display_mode.upper()

    if mode == "WHEELS":
        return {
            "rows": 2,
            "figsize": FIGSIZE_WHEELS,
            "top": 0.91,
            "bottom": 0.085,
            "hspace": 0.10,
            "sharex": True,
        }

    if mode == "CHASSIS":
        return {
            "rows": 2,
            "figsize": FIGSIZE_CHASSIS,
            "top": 0.91,
            "bottom": 0.085,
            "hspace": 0.10,
            "sharex": True,
        }

    return {
        "rows": 3,
        "figsize": FIGSIZE_ODOM,
        "top": 0.82,
        "bottom": 0.075,
        "hspace": 0.72,
        "sharex": False,
    }


# ============================================================
# Live-Plot
# ============================================================

def start_plot(display_mode: str):
    initial_mode = display_mode.upper()
    layout = get_plot_layout(initial_mode)

    fig, axes = plt.subplots(
        layout["rows"],
        1,
        figsize=layout["figsize"],
        sharex=layout["sharex"]
    )

    if layout["rows"] == 1:
        axes = [axes]
    else:
        axes = list(axes)

    fig.suptitle("Robot Monitor", fontsize=13, y=0.985)

    status_text = fig.text(
        0.5,
        ODOM_STATUS_TEXT_Y,
        "",
        ha="center",
        va="top",
        fontsize=ODOM_STATUS_TEXT_FONTSIZE
    )

    if layout["rows"] >= 3:
        time_axes = [ax.twiny() for ax in axes]
    else:
        time_axes = []

    def apply_layout():
        fig.subplots_adjust(
            left=0.075,
            right=0.985,
            top=layout["top"],
            bottom=layout["bottom"],
            hspace=layout["hspace"]
        )

    apply_layout()

    add_extra_plot_buttons(fig)

    def maximize_and_fit_to_window():
        try:
            manager = plt.get_current_fig_manager()
            window = manager.window

            window.state("zoomed")
            window.update_idletasks()

            widget = fig.canvas.get_tk_widget()
            width_px = widget.winfo_width()
            height_px = widget.winfo_height()

            dpi = fig.get_dpi()

            if width_px > 200 and height_px > 200:
                fig.set_size_inches(
                    width_px / dpi,
                    height_px / dpi,
                    forward=True
                )

            apply_layout()
            fig.canvas.draw_idle()

            print("[Plot] Fenster maximiert und Figure an sichtbare Flaeche angepasst")

        except Exception as e:
            print(f"[Plot] Maximieren nicht moeglich: {e}")

    try:
        manager = plt.get_current_fig_manager()
        manager.window.after(300, maximize_and_fit_to_window)
    except Exception as e:
        print(f"[Plot] Maximieren nicht vorbereitet: {e}")

    def update(_):
        if not store.ready:
            return

        d, mode = store.snapshot()

        if mode == "ODOM":
            if len(axes) < 3:
                status_text.set_text(
                    "ODOM-Daten empfangen, aber Plot wurde als 2-Achsen-WHEELS-Fenster gestartet."
                )
                return

            update_odom_plot(d, axes, time_axes, status_text)

        elif mode == "WHEELS":
            status_text.set_text("")
            hide_time_axes(time_axes)
            update_wheels_plot(d, axes)

        elif mode == "CHASSIS":
            status_text.set_text("")
            hide_time_axes(time_axes)
            update_chassis_plot(d, axes)

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=UPDATE_MS,
        cache_frame_data=False
    )

    plt.show()
    return ani


# ============================================================
# Gemeinsame Plot-Helfer
# ============================================================

def hide_time_axes(time_axes):
    for ax in time_axes:
        ax.set_visible(False)
        ax.set_xlabel("")
        ax.set_xticks([])

        ax.tick_params(
            axis="x",
            which="both",
            top=False,
            labeltop=False,
            bottom=False,
            labelbottom=False
        )

        ax.tick_params(
            axis="y",
            which="both",
            left=False,
            right=False,
            labelleft=False,
            labelright=False
        )


def clear_axes(axes):
    for ax in axes:
        ax.cla()
        ax.grid(True, linestyle="--", alpha=0.5)


# ============================================================
# WHEELS-Plot
# ============================================================

def update_wheels_plot(d, axes):
    axes[0].set_visible(True)
    axes[1].set_visible(True)

    t_ms = d.get("t_plot_ms", [])

    if not t_ms:
        return

    t = [x / 1000.0 for x in t_ms]

    clear_axes(axes[:2])

    ax_v = axes[0]
    ax_pwm = axes[1]

    ax_v.set_ylabel("Geschwindigkeit [m/s]")
    ax_pwm.set_ylabel("PWM")
    ax_pwm.set_xlabel("Zeit [s]")

    cmd_indices = d.get("cmd_index", [])
    local_ms = d.get("ms", [])

    for name, col in COLORS_RAD.items():
        s_key = f"{name}_s"
        i_key = f"{name}_i"
        pwm_key = f"{name}_pwm"

        s = d.get(s_key, [])
        ist = d.get(i_key, [])
        pwm = d.get(pwm_key, [])

        n = min(
            len(t),
            len(s),
            len(ist),
            len(cmd_indices),
            len(local_ms)
        )

        if n:
            x_soll, y_soll = build_command_boundary_series(
                t[:n],
                s[:n],
                cmd_indices[:n],
                local_ms[:n]
            )

            ax_v.plot(
                x_soll,
                y_soll,
                linestyle="--",
                color=col,
                alpha=0.6,
                label=f"{name} Soll"
            )

            ax_v.plot(
                t[:n],
                ist[:n],
                linestyle="-",
                color=col,
                label=f"{name} Ist"
            )

        n_p = min(
            len(t),
            len(pwm),
            len(cmd_indices),
            len(local_ms)
        )

        if n_p:
            x_pwm, y_pwm = build_command_boundary_series(
                t[:n_p],
                pwm[:n_p],
                cmd_indices[:n_p],
                local_ms[:n_p]
            )

            ax_pwm.plot(
                x_pwm,
                y_pwm,
                linestyle="-",
                color=col,
                label=f"{name} PWM"
            )

    ax_v.legend(loc="lower right", fontsize=7, ncol=4)
    ax_pwm.legend(loc="lower right", fontsize=7, ncol=4)

    ax_pwm.set_ylim(-255, 255)

    n_pts = len(t)

    ax_v.set_title(
        f"WHEELS - {n_pts} Messpunkte - t = {t[-1]:.1f} s",
        fontsize=10,
        pad=6
    )


# ============================================================
# CHASSIS-Plot
# ============================================================

def update_chassis_plot(d, axes):
    axes[0].set_visible(True)
    axes[1].set_visible(True)

    t_ms = d.get("t_plot_ms", [])

    if not t_ms:
        return

    t = [x / 1000.0 for x in t_ms]

    clear_axes(axes[:2])

    ax_rad = axes[0]
    ax_veh = axes[1]

    ax_rad.set_ylabel("Radgeschwindigkeit Ist [m/s]")
    ax_veh.set_ylabel("Fahrzeug [m/s / rad/s]")
    ax_veh.set_xlabel("Zeit [s]")

    for name, col in COLORS_RAD.items():
        vals = d.get(f"{name}_i", [])
        n = min(len(t), len(vals))

        if n:
            ax_rad.plot(
                t[:n],
                vals[:n],
                color=col,
                label=f"{name}"
            )

    for key, col in COLORS_VEH.items():
        vals = d.get(key, [])
        n = min(len(t), len(vals))

        if n:
            ax_veh.plot(
                t[:n],
                vals[:n],
                color=col,
                label=key
            )

    ax_rad.legend(loc="lower right", fontsize=8, ncol=4)
    ax_veh.legend(loc="lower right", fontsize=8, ncol=3)

    n_pts = len(t)

    ax_rad.set_title(
        f"CHASSIS - {n_pts} Messpunkte - t = {t[-1]:.1f} s",
        fontsize=10,
        pad=6
    )