# ============================================================
# Datei: plot_odom.py
# Zweck:
#   - ODOM-spezifische Rechenfunktionen
#   - mehrere CMDP-Teilstuecke fortlaufend zusammensetzen
#   - Weltkoordinaten fuer den Plot berechnen
#   - obere Zeitachse fuer ODOM erzeugen
#   - ODOM-Plot aktualisieren
#
# Diese Datei kennt keinen Serial-Port.
# Diese Datei startet kein Fenster.
# Sie wird von plotter.py benutzt.
# ============================================================

import math

from config import *


# ============================================================
# Kleine Plot-Helfer fuer ODOM
# ============================================================

def clear_axes(axes):
    for ax in axes:
        ax.cla()
        ax.grid(True, linestyle="--", alpha=0.5)


def show_time_axes(time_axes):
    for ax in time_axes:
        ax.set_visible(True)


# ============================================================
# ODOM-Hilfsfunktionen
# ============================================================

def reset_delta(current_value: float, previous_value: float, reset_detected: bool) -> float:
    if not reset_detected:
        return current_value - previous_value

    return current_value


def build_continuous_odom(path_cm, x_cm, y_cm, phi_deg, raw_ms, plot_ms):
    """
    Setzt mehrere CMDP-Teilstuecke zu EINER fortlaufenden Odometrie-Strecke zusammen.

    Problem:
      Arduino gibt bei jedem neuen CMDP wieder lokale Werte aus:
        ms      : 0,100,200,...
        path_cm : 0,...,100
        x_cm    : lokal zum CMDP
        y_cm    : lokal zum CMDP
        phi_deg : lokal zum CMDP

    Der Plot soll daraus machen:
        fortlaufender Weg
        fortlaufende x/y/phi-Werte
    """

    n = min(len(path_cm), len(x_cm), len(y_cm), len(phi_deg), len(plot_ms))

    if n == 0:
        return {
            "path": [],
            "x": [],
            "y": [],
            "phi": [],
            "t_ms": [],
        }

    result = {
        "path": [],
        "x": [],
        "y": [],
        "phi": [],
        "t_ms": [],
    }

    cum_path = 0.0
    cum_x = 0.0
    cum_y = 0.0
    cum_phi = 0.0

    prev_path = None
    prev_x = None
    prev_y = None
    prev_phi = None
    prev_raw_ms = None

    for i in range(n):
        p = float(path_cm[i])
        x = float(x_cm[i])
        y = float(y_cm[i])
        phi = float(phi_deg[i])
        t_plot = float(plot_ms[i])

        if i < len(raw_ms):
            t_raw = float(raw_ms[i])
        else:
            t_raw = t_plot

        if prev_path is None:
            cum_path = p
            cum_x = x
            cum_y = y
            cum_phi = phi
        else:
            path_reset = p < (prev_path - ODOM_RESET_DROP_CM)
            time_reset = t_raw < (prev_raw_ms - ODOM_RESET_TIME_DROP_MS)
            reset_detected = path_reset or time_reset

            if reset_detected:
                dp = p
            else:
                dp = p - prev_path

            if dp < 0.0:
                dp = 0.0

            dx = reset_delta(x, prev_x, reset_detected)
            dy = reset_delta(y, prev_y, reset_detected)
            dphi = reset_delta(phi, prev_phi, reset_detected)

            cum_path += dp
            cum_x += dx
            cum_y += dy
            cum_phi += dphi

        result["path"].append(cum_path)
        result["x"].append(cum_x)
        result["y"].append(cum_y)
        result["phi"].append(cum_phi)
        result["t_ms"].append(t_plot)

        prev_path = p
        prev_x = x
        prev_y = y
        prev_phi = phi
        prev_raw_ms = t_raw

    return result


def compute_world_coordinates(run):
    x_body = run["x"]
    y_body = run["y"]
    phi_deg = run["phi"]

    n = min(len(x_body), len(y_body), len(phi_deg))

    if n == 0:
        return [], []

    x_world = [0.0]
    y_world = [0.0]

    for i in range(1, n):
        dx_body = x_body[i] - x_body[i - 1]
        dy_body = y_body[i] - y_body[i - 1]

        phi_prev = math.radians(phi_deg[i - 1])
        phi_now = math.radians(phi_deg[i])
        dphi = phi_now - phi_prev
        phi_mid = phi_prev + 0.5 * dphi

        c = math.cos(phi_mid)
        s = math.sin(phi_mid)

        dx_world = dx_body * c - dy_body * s
        dy_world = dx_body * s + dy_body * c

        x_world.append(x_world[-1] + dx_world)
        y_world.append(y_world[-1] + dy_world)

    return x_world, y_world


# ============================================================
# Obere Zeitachse fuer ODOM
# ============================================================

def interpolate_time_for_path(path_values, time_values_ms, path_target):
    n = min(len(path_values), len(time_values_ms))

    if n == 0:
        return 0.0

    if path_target <= path_values[0]:
        return time_values_ms[0] / 1000.0

    for i in range(1, n):
        p0 = path_values[i - 1]
        p1 = path_values[i]

        if p0 <= path_target <= p1:
            t0 = time_values_ms[i - 1]
            t1 = time_values_ms[i]

            dp = p1 - p0

            if abs(dp) < 1e-9:
                return t1 / 1000.0

            alpha = (path_target - p0) / dp
            return (t0 + alpha * (t1 - t0)) / 1000.0

    return time_values_ms[n - 1] / 1000.0


def make_even_ticks(start_value, end_value, count):
    if count <= 1:
        return [start_value]

    step = (end_value - start_value) / (count - 1)
    return [start_value + i * step for i in range(count)]


def update_time_axis(time_ax, path_values, time_values_ms, max_path):
    time_ax.set_visible(True)

    x_max = max_path * 1.03 if max_path > 0.0 else 1.0
    time_ax.set_xlim(0.0, x_max)

    time_ax.grid(False)

    time_ax.tick_params(
        axis="y",
        which="both",
        left=False,
        right=False,
        labelleft=False,
        labelright=False
    )

    time_ax.xaxis.set_ticks_position("top")
    time_ax.xaxis.set_label_position("top")

    time_ax.tick_params(
        axis="x",
        which="both",
        top=True,
        labeltop=True,
        bottom=False,
        labelbottom=False,
        pad=3
    )

    time_ax.set_xlabel("Zeit [s]", labelpad=8)

    time_ax.spines["bottom"].set_visible(False)
    time_ax.spines["left"].set_visible(False)
    time_ax.spines["right"].set_visible(False)
    time_ax.spines["top"].set_visible(True)

    if not path_values or not time_values_ms or max_path <= 0.0:
        time_ax.set_xticks([])
        return

    ticks = make_even_ticks(0.0, max_path, ODOM_TIME_AXIS_TICKS)

    labels = [
        f"{interpolate_time_for_path(path_values, time_values_ms, tick):.1f}"
        for tick in ticks
    ]

    time_ax.set_xticks(ticks)
    time_ax.set_xticklabels(labels)


# ============================================================
# ODOM-Plot
# ============================================================

def update_odom_plot(d, axes, time_axes, status_text=None):
    for ax in axes:
        ax.set_visible(True)

    show_time_axes(time_axes)

    raw_path_cm = d.get("path_cm", [])
    raw_y_body_cm = d.get("y_cm", [])
    raw_phi_deg = d.get("phi_deg", [])
    raw_x_body_cm = d.get("x_cm", [])
    raw_ms = d.get("ms", [])
    plot_ms = d.get("t_plot_ms", raw_ms)

    run = build_continuous_odom(
        raw_path_cm,
        raw_x_body_cm,
        raw_y_body_cm,
        raw_phi_deg,
        raw_ms,
        plot_ms
    )

    if len(run["path"]) < ODOM_MIN_RUN_POINTS:
        return

    clear_axes(axes)

    ax_y_world = axes[0]
    ax_y_body = axes[1]
    ax_phi = axes[2]

    path = run["path"]
    y_body = run["y"]
    phi = run["phi"]

    x_world, y_world = compute_world_coordinates(run)

    max_path = max(path) if path else 0.0

    n_world = min(len(path), len(y_world))

    if n_world:
        ax_y_world.plot(
            path[:n_world],
            y_world[:n_world],
            linestyle="-",
            label="y_world"
        )

    n_body = min(len(path), len(y_body))

    if n_body:
        ax_y_body.plot(
            path[:n_body],
            y_body[:n_body],
            linestyle="-",
            label="y_body"
        )

    n_phi = min(len(path), len(phi))

    if n_phi:
        ax_phi.plot(
            path[:n_phi],
            phi[:n_phi],
            linestyle="-",
            label="phi"
        )

    ax_y_world.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)
    ax_y_body.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)
    ax_phi.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.7)

    ax_y_world.set_xlabel("Weg path fortlaufend [cm]")
    ax_y_world.set_ylabel("y_world [cm]")

    ax_y_body.set_xlabel("Weg path fortlaufend [cm]")
    ax_y_body.set_ylabel("y_body [cm]")

    ax_phi.set_xlabel("Weg path fortlaufend [cm]")
    ax_phi.set_ylabel("phi [deg]")

    if max_path > 0.0:
        x_max = max_path * 1.03

        ax_y_world.set_xlim(0.0, x_max)
        ax_y_body.set_xlim(0.0, x_max)
        ax_phi.set_xlim(0.0, x_max)

    ax_y_world.tick_params(axis="y", which="both", labelleft=True, left=True)
    ax_y_body.tick_params(axis="y", which="both", labelleft=True, left=True)
    ax_phi.tick_params(axis="y", which="both", labelleft=True, left=True)

    ax_y_world.legend(loc="lower left", fontsize=8, ncol=2)
    ax_y_body.legend(loc="lower left", fontsize=8, ncol=2)
    ax_phi.legend(loc="lower left", fontsize=8, ncol=2)

    for time_ax in time_axes:
        update_time_axis(
            time_ax,
            run["path"],
            run["t_ms"],
            max_path
        )

    last_path = run["path"][-1]
    last_x_body = run["x"][-1]
    last_y_body = run["y"][-1]
    last_phi = run["phi"][-1]
    last_t = run["t_ms"][-1] / 1000.0

    if x_world and y_world:
        last_x_world = x_world[-1]
        last_y_world = y_world[-1]
    else:
        last_x_world = 0.0
        last_y_world = 0.0

    total_points = len(run["path"])

    status_line = (
        f"ODOM - fortlaufende Strecke - {total_points} Messpunkte - "
        f"path = {last_path:.2f} cm - "
        f"x_body = {last_x_body:.2f} cm - "
        f"y_body = {last_y_body:.2f} cm - "
        f"x_world = {last_x_world:.2f} cm - "
        f"y_world = {last_y_world:.2f} cm - "
        f"phi = {last_phi:.2f} deg - t = {last_t:.1f} s"
    )

    if status_text is not None:
        status_text.set_text(status_line)
        ax_y_world.set_title("")
    else:
        ax_y_world.set_title(status_line, fontsize=10, pad=28)