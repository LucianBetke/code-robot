# plot_wheels.py
# ============================================================
# Aufgabe:
#  - WHEELS-Ansicht (Rad-Soll/Ist in cm/s + PWM)
#  - X-Achse: Zeit [s], durchlaufend ueber mehrere Laeufe
#
# Diese Datei enthaelt nur die WHEELS-spezifische Zeichenlogik.
# Fenster/Button/Status baut plotter.py; hier wird nur
# update_wheels_plot(axes, store) aufgerufen - wie bei plot_odom.py.
# ============================================================

from __future__ import annotations

import math


# ============================================================
# Anzeige-Konstanten
# ============================================================

WHEEL_LINEWIDTH = 2.4
WHEEL_SETPOINT_LINEWIDTH = 1.8

LEGEND_FONTSIZE = 14
AXIS_LABEL_FONTSIZE = 17
TICK_LABEL_FONTSIZE = 14


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


def update_wheels_plot(axes, store) -> None:
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