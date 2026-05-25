# plot_odom.py
# ============================================================
# Aufgabe:
#  - Neues ODOM2-Protokoll auswerten
#  - Sollbewegung aus #CMDP_BEGIN berechnen
#  - Istbewegung aus #ODOM2 zusammensetzen
#  - Laengsfehler und Querfehler berechnen
#
# Neues Arduino-Format:
#   #CMDP_BEGIN,id,vx,vy,wz,target
#   #ODOM2,id,ms,path_cm,x_body_cm,y_body_cm,phi_deg
#
# Darstellung:
#   1. e_quer
#   2. e_laengs
#   3. e_verdrehen
#
# Keine Diagrammtitel.
#
# Unten an jedem Diagramm:
#   Weg [cm]
#
# Oben an jedem Diagramm:
#   Zeit [s]
#
# Mehrfachlauf-Unterstuetzung:
#   Werden mehrere Laeufe mit gleichen cmd_ids aufgezeichnet, erkennt
#   _split_rows_by_run() den Reset am ms-Ruecksprung und trennt die
#   Laeufe. Jeder Lauf wird unabhaengig berechnet; im Plot erscheinen
#   die Kurven uebereinander mit NaN-Luecke dazwischen.
# ============================================================

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

import numpy as np


@dataclass
class OdomPlotData:
    s_cm: list[float]
    t_s: list[float]

    x_ist_cm: list[float]
    y_ist_cm: list[float]

    x_soll_cm: list[float]
    y_soll_cm: list[float]

    e_parallel_cm: list[float]
    e_cross_cm: list[float]

    phi_deg: list[float]
    cmd_id: list[int]

    text: str


def _cmd_value(cmd, name: str, default: float = 0.0) -> float:
    if hasattr(cmd, name):
        return float(getattr(cmd, name))

    if isinstance(cmd, dict):
        return float(cmd.get(name, default))

    return default


def _row_value(row, name: str, default: float = 0.0) -> float:
    if hasattr(row, name):
        return float(getattr(row, name))

    if isinstance(row, dict):
        return float(row.get(name, default))

    return default


def _row_id(row) -> int:
    if hasattr(row, "cmd_id"):
        return int(getattr(row, "cmd_id"))

    if isinstance(row, dict):
        return int(row.get("cmd_id", 0))

    return 0


def _safe_unit(vx: float, vy: float) -> tuple[float, float, float]:
    v_abs = sqrt(vx * vx + vy * vy)

    if v_abs <= 1.0e-9:
        return 0.0, 0.0, 0.0

    return vx / v_abs, vy / v_abs, v_abs


# ============================================================
# Lauf-Trennung
# ============================================================

def _split_rows_by_run(rows: list, threshold_ms: float = 1000.0) -> list[list]:
    """Teilt eine Zeilenliste anhand eines ms-Resets in einzelne Laeufe auf.

    Wenn ms von einem hohen Wert auf nahe 0 springt (Differenz > threshold_ms),
    beginnt ein neuer Lauf. Fuer Einzellaeufe gibt die Funktion eine Liste
    mit einem einzigen Element zurueck.
    """
    if not rows:
        return []

    runs: list[list] = [[]]
    prev_ms: float | None = None

    for row in rows:
        ms = _row_value(row, "ms")

        if prev_ms is not None and ms < prev_ms - threshold_ms:
            runs.append([])

        runs[-1].append(row)
        prev_ms = ms

    return runs


# ============================================================
# Datenaufbau pro Lauf
# ============================================================

def _build_single_run_data(
    rows_by_id_run: dict[int, list],
    cmdp_by_id: dict,
    ordered_ids: list[int],
) -> OdomPlotData:
    """Berechnet OdomPlotData fuer einen einzelnen Lauf.

    rows_by_id_run enthaelt nur die Zeilen des jeweiligen Laufs
    (bereits aufgeteilt durch _split_rows_by_run).
    """
    # Erster Durchlauf: kumulative Versaetze pro cmd_id berechnen
    ideal_start_by_id: dict[int, tuple[float, float, float, float]] = {}
    actual_start_by_id: dict[int, tuple[float, float, float, float, float]] = {}

    ideal_x = 0.0
    ideal_y = 0.0
    ideal_s = 0.0
    ideal_t = 0.0

    actual_x = 0.0
    actual_y = 0.0
    actual_s = 0.0
    actual_t = 0.0
    actual_phi = 0.0

    for cmd_id in ordered_ids:
        ideal_start_by_id[cmd_id] = (ideal_x, ideal_y, ideal_s, ideal_t)
        actual_start_by_id[cmd_id] = (
            actual_x,
            actual_y,
            actual_s,
            actual_t,
            actual_phi,
        )

        cmd = cmdp_by_id.get(cmd_id)

        if cmd is not None:
            vx = _cmd_value(cmd, "vx_cms")
            vy = _cmd_value(cmd, "vy_cms")
            target = _cmd_value(cmd, "target")
            ux, uy, v_abs = _safe_unit(vx, vy)

            ideal_x += ux * target
            ideal_y += uy * target
            ideal_s += target

            if v_abs > 1.0e-9:
                ideal_t += target / v_abs

        seg_rows = rows_by_id_run.get(cmd_id, [])

        if seg_rows:
            last = seg_rows[-1]

            actual_x += _row_value(last, "x_body_cm")
            actual_y += _row_value(last, "y_body_cm")
            actual_s += _row_value(last, "path_cm")
            actual_t += _row_value(last, "ms") * 0.001
            actual_phi += _row_value(last, "phi_deg")

    # Zweiter Durchlauf: Ausgabelisten befuellen
    s_cm: list[float] = []
    t_s: list[float] = []

    x_ist_cm: list[float] = []
    y_ist_cm: list[float] = []

    x_soll_cm: list[float] = []
    y_soll_cm: list[float] = []

    e_parallel_cm: list[float] = []
    e_cross_cm: list[float] = []

    phi_deg: list[float] = []
    cmd_id_out: list[int] = []

    for cmd_id in ordered_ids:
        cmd = cmdp_by_id.get(cmd_id)
        seg_rows = rows_by_id_run.get(cmd_id, [])

        if not seg_rows:
            continue

        if cmd is None:
            ux = 1.0
            uy = 0.0
            v_abs = 0.0
            target = 0.0
        else:
            vx = _cmd_value(cmd, "vx_cms")
            vy = _cmd_value(cmd, "vy_cms")
            target = _cmd_value(cmd, "target")
            ux, uy, v_abs = _safe_unit(vx, vy)

        ideal_x0, ideal_y0, ideal_s0, ideal_t0 = ideal_start_by_id.get(
            cmd_id,
            (0.0, 0.0, 0.0, 0.0),
        )

        actual_x0, actual_y0, actual_s0, actual_t0, actual_phi0 = actual_start_by_id.get(
            cmd_id,
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )

        for row in seg_rows:
            ms = _row_value(row, "ms")
            path_local = _row_value(row, "path_cm")
            x_local = _row_value(row, "x_body_cm")
            y_local = _row_value(row, "y_body_cm")
            phi_local = _row_value(row, "phi_deg")

            t_local = ms * 0.001
            t_global = actual_t0 + t_local

            if v_abs > 1.0e-9:
                s_soll_local = v_abs * t_local

                if s_soll_local > target:
                    s_soll_local = target
            else:
                s_soll_local = 0.0

            x_soll_local = ux * s_soll_local
            y_soll_local = uy * s_soll_local

            x_actual_global = actual_x0 + x_local
            y_actual_global = actual_y0 + y_local

            x_ideal_global = ideal_x0 + x_soll_local
            y_ideal_global = ideal_y0 + y_soll_local

            error_x = x_actual_global - x_ideal_global
            error_y = y_actual_global - y_ideal_global

            if v_abs > 1.0e-9:
                e_parallel = error_x * ux + error_y * uy
                e_cross = -error_x * uy + error_y * ux
            else:
                e_parallel = 0.0
                e_cross = 0.0

            phi_global = actual_phi0 + phi_local

            s_cm.append(actual_s0 + path_local)
            t_s.append(t_global)

            x_ist_cm.append(x_actual_global)
            y_ist_cm.append(y_actual_global)

            x_soll_cm.append(x_ideal_global)
            y_soll_cm.append(y_ideal_global)

            e_parallel_cm.append(e_parallel)
            e_cross_cm.append(e_cross)

            phi_deg.append(phi_global)
            cmd_id_out.append(cmd_id)

    last_text = ""

    if s_cm:
        last_text = (
            f"s={s_cm[-1]:.2f} cm   "
            f"e_laengs={e_parallel_cm[-1]:.2f} cm   "
            f"e_quer={e_cross_cm[-1]:.2f} cm   "
            f"e_verdrehen={phi_deg[-1]:.2f} deg"
        )

    return OdomPlotData(
        s_cm=s_cm,
        t_s=t_s,
        x_ist_cm=x_ist_cm,
        y_ist_cm=y_ist_cm,
        x_soll_cm=x_soll_cm,
        y_soll_cm=y_soll_cm,
        e_parallel_cm=e_parallel_cm,
        e_cross_cm=e_cross_cm,
        phi_deg=phi_deg,
        cmd_id=cmd_id_out,
        text=last_text,
    )


# ============================================================
# Oeffentliche Hauptfunktion
# ============================================================

def build_odom_plot_data(snapshot: dict) -> OdomPlotData:
    cmdp_by_id = snapshot.get("cmdp_by_id", {})
    cmdp_order = snapshot.get("cmdp_order", [])
    odom2_rows = snapshot.get("odom2_rows", [])

    if not odom2_rows:
        return OdomPlotData(
            [], [], [], [], [], [], [], [], [], [],
            "warte auf #ODOM2 ...",
        )

    # Alle Zeilen nach cmd_id gruppieren
    rows_by_id: dict[int, list] = {}

    for row in odom2_rows:
        rows_by_id.setdefault(_row_id(row), []).append(row)

    # Reihenfolge der cmd_ids bestimmen
    ordered_ids = [int(i) for i in cmdp_order if int(i) in rows_by_id]

    for cmd_id in rows_by_id.keys():
        if cmd_id not in ordered_ids:
            ordered_ids.append(cmd_id)

    # Jeden cmd_id-Bucket in einzelne Laeufe aufteilen (ms-Reset-Erkennung)
    runs_by_id: dict[int, list[list]] = {
        cmd_id: _split_rows_by_run(rows)
        for cmd_id, rows in rows_by_id.items()
    }

    # Anzahl Laeufe = Maximum der Teillistenlaengen
    num_runs = max(len(runs) for runs in runs_by_id.values()) if runs_by_id else 1

    # Pro Lauf unabhaengig berechnen
    run_data_list: list[OdomPlotData] = []

    for run_idx in range(num_runs):
        run_rows_by_id = {
            cmd_id: runs[run_idx] if run_idx < len(runs) else []
            for cmd_id, runs in runs_by_id.items()
        }

        run_data = _build_single_run_data(run_rows_by_id, cmdp_by_id, ordered_ids)
        run_data_list.append(run_data)

    # Laeufe mit NaN-Trennpunkt zusammenfuehren
    NAN = float("nan")

    s_cm: list[float] = []
    t_s: list[float] = []
    x_ist_cm: list[float] = []
    y_ist_cm: list[float] = []
    x_soll_cm: list[float] = []
    y_soll_cm: list[float] = []
    e_parallel_cm: list[float] = []
    e_cross_cm: list[float] = []
    phi_deg: list[float] = []
    cmd_id_out: list[int] = []

    for idx, rd in enumerate(run_data_list):
        if not rd.s_cm:
            continue

        if s_cm:
            # NaN-Trennpunkt zwischen Laeufen einfuegen
            s_cm.append(NAN)
            t_s.append(NAN)
            x_ist_cm.append(NAN)
            y_ist_cm.append(NAN)
            x_soll_cm.append(NAN)
            y_soll_cm.append(NAN)
            e_parallel_cm.append(NAN)
            e_cross_cm.append(NAN)
            phi_deg.append(NAN)
            cmd_id_out.append(-1)

        s_cm.extend(rd.s_cm)
        t_s.extend(rd.t_s)
        x_ist_cm.extend(rd.x_ist_cm)
        y_ist_cm.extend(rd.y_ist_cm)
        x_soll_cm.extend(rd.x_soll_cm)
        y_soll_cm.extend(rd.y_soll_cm)
        e_parallel_cm.extend(rd.e_parallel_cm)
        e_cross_cm.extend(rd.e_cross_cm)
        phi_deg.extend(rd.phi_deg)
        cmd_id_out.extend(rd.cmd_id)

    # Statustext vom letzten Lauf mit Daten
    last_text = next(
        (rd.text for rd in reversed(run_data_list) if rd.text),
        "",
    )

    return OdomPlotData(
        s_cm=s_cm,
        t_s=t_s,
        x_ist_cm=x_ist_cm,
        y_ist_cm=y_ist_cm,
        x_soll_cm=x_soll_cm,
        y_soll_cm=y_soll_cm,
        e_parallel_cm=e_parallel_cm,
        e_cross_cm=e_cross_cm,
        phi_deg=phi_deg,
        cmd_id=cmd_id_out,
        text=last_text,
    )


def build_continuous_odom(snapshot: dict) -> OdomPlotData:
    return build_odom_plot_data(snapshot)


# ============================================================
# Plot-Hilfsroutinen
# ============================================================

def _draw_command_boundaries(ax, s_cm: list[float], cmd_id: list[int]) -> None:
    if not s_cm or not cmd_id:
        return

    last_id = cmd_id[0]

    for index in range(1, len(cmd_id)):
        current_id = cmd_id[index]

        # cmd_id == -1 ist ein NaN-Trennpunkt zwischen Laeufen -> ignorieren
        if current_id != last_id and current_id != -1 and last_id != -1:
            ax.axvline(s_cm[index], linestyle="--", linewidth=1.1)

        if current_id != -1:
            last_id = current_id


def _update_status_text(fig, text: str) -> None:
    status_text = getattr(fig, "_robot_status_text", None)

    if status_text is not None:
        status_text.set_text(text)


def _strictly_increasing_arrays(x_values: list[float], y_values: list[float]) -> tuple[np.ndarray, np.ndarray]:
    xs: list[float] = []
    ys: list[float] = []

    for x, y in zip(x_values, y_values):
        xf = float(x)
        yf = float(y)

        if xf != xf or yf != yf:  # NaN ueberspringen
            continue

        if not xs:
            xs.append(xf)
            ys.append(yf)
            continue

        if xf > xs[-1] + 1.0e-9:
            xs.append(xf)
            ys.append(yf)
        else:
            ys[-1] = yf

    if len(xs) < 2:
        return np.array([0.0, 1.0]), np.array([0.0, 1.0])

    return np.array(xs), np.array(ys)


def _add_time_axis(ax, data: OdomPlotData, show_label: bool = True) -> None:
    # Alte Sekundaerachsen entfernen, sonst stapeln sie sich bei Live-Animation.
    for child in list(ax.child_axes):
        child.remove()

    s_arr, t_arr = _strictly_increasing_arrays(data.s_cm, data.t_s)
    t_for_inverse, s_for_inverse = _strictly_increasing_arrays(data.t_s, data.s_cm)

    def s_to_t(x):
        return np.interp(x, s_arr, t_arr)

    def t_to_s(t):
        return np.interp(t, t_for_inverse, s_for_inverse)

    secax = ax.secondary_xaxis("top", functions=(s_to_t, t_to_s))

    if show_label:
        secax.set_xlabel("Zeit [s]", fontsize=15, labelpad=6)
    else:
        secax.set_xlabel("")

    secax.tick_params(axis="x", labelsize=13, pad=3)


def _format_axis(ax, ylabel: str, show_xlabel: bool = True) -> None:
    ax.set_ylabel(ylabel, fontsize=15, labelpad=6)

    if show_xlabel:
        ax.set_xlabel("Weg [cm]", fontsize=15, labelpad=6)
    else:
        ax.set_xlabel("")

    ax.grid(True)
    ax.tick_params(axis="both", labelsize=13, pad=3)


# ============================================================
# Plot-Update
# ============================================================

def update_odom_plot(axes, store) -> None:
    snapshot = store.snapshot()
    data = build_odom_plot_data(snapshot)

    if len(axes) < 3:
        return

    fig = axes[0].figure

    ax_cross = axes[0]
    ax_parallel = axes[1]
    ax_phi = axes[2]

    ax_cross.clear()
    ax_parallel.clear()
    ax_phi.clear()

    _update_status_text(fig, data.text)

    _format_axis(ax_cross,    "e_quer [cm]",       show_xlabel=False)
    _format_axis(ax_parallel, "e_laengs [cm]",     show_xlabel=False)
    _format_axis(ax_phi,      "e_verdrehen [deg]", show_xlabel=True)

    if not data.s_cm:
        ax_cross.text(
            0.5,
            0.5,
            data.text,
            transform=ax_cross.transAxes,
            ha="center",
            va="center",
            fontsize=13,
        )
        return

    for i, ax in enumerate((ax_cross, ax_parallel, ax_phi)):
        ax.axhline(0.0, linewidth=1.2)
        _draw_command_boundaries(ax, data.s_cm, data.cmd_id)
        ax.margins(x=0.01)
        _add_time_axis(ax, data, show_label=(i == 0))

    # NaN-Trennpunkte sind bereits in data enthalten (zwischen Laeufen).
    # Direktes Plotten genuegt.
    ax_cross.plot(data.s_cm, data.e_cross_cm, linewidth=1.8)
    ax_parallel.plot(data.s_cm, data.e_parallel_cm, linewidth=1.8)
    ax_phi.plot(data.s_cm, data.phi_deg, linewidth=1.8)