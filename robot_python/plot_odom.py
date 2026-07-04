# plot_odom.py
# ============================================================
# Aufgabe:
#  - ODOM2-Protokoll auswerten
#  - Sollbewegung aus #CMDP_BEGIN berechnen
#  - Istbewegung aus #ODOM2 zusammensetzen
#  - Laengsfehler, Querfehler und Verdrehfehler bestimmen
#
# Arduino-Format:
#   #CMDP_BEGIN,id,vx,vy,wz,target
#   #ODOM2,id,ms,path_cm,x_body_cm,y_body_cm,phi_deg
#
# Darstellung:  1. e_quer   2. e_laengs   3. e_verdrehen
#
# Mehrfachlauf-Unterstuetzung:
#   Werden mehrere Laeufe mit gleichen cmd_ids aufgezeichnet, erkennt
#   _split_rows_by_run() den Reset am ms-Ruecksprung und trennt die Laeufe.
#   Im Plot erscheinen die Kurven mit NaN-Luecke dazwischen.
# ============================================================

from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt

import numpy as np


@dataclass
class OdomPlotData:
    s_cm:          list[float] = field(default_factory=list)
    t_s:           list[float] = field(default_factory=list)
    x_ist_cm:      list[float] = field(default_factory=list)
    y_ist_cm:      list[float] = field(default_factory=list)
    x_soll_cm:     list[float] = field(default_factory=list)
    y_soll_cm:     list[float] = field(default_factory=list)
    e_parallel_cm: list[float] = field(default_factory=list)
    e_cross_cm:    list[float] = field(default_factory=list)
    phi_deg:       list[float] = field(default_factory=list)
    cmd_id:        list[int]   = field(default_factory=list)
    text:          str         = ""


def _safe_unit(vx: float, vy: float) -> tuple[float, float, float]:
    v_abs = sqrt(vx * vx + vy * vy)
    if v_abs <= 1.0e-9:
        return 0.0, 0.0, 0.0
    return vx / v_abs, vy / v_abs, v_abs


# ============================================================
# Lauf-Trennung
# ============================================================

def _split_rows_by_run(rows: list, threshold_ms: float = 1000.0) -> list[list]:
    """Teilt Zeilen anhand eines ms-Resets in einzelne Laeufe auf.

    Wenn ms von einem hohen Wert auf nahe 0 springt (Differenz > threshold_ms),
    beginnt ein neuer Lauf.
    """
    if not rows:
        return []

    runs: list[list] = [[]]
    prev_ms: float | None = None

    for row in rows:
        if prev_ms is not None and row.ms < prev_ms - threshold_ms:
            runs.append([])
        runs[-1].append(row)
        prev_ms = row.ms

    return runs


# ============================================================
# Datenaufbau pro Lauf
# ============================================================

def _build_single_run_data(
    rows_by_id_run: dict[int, list],
    cmdp_by_id: dict,
    ordered_ids: list[int],
) -> OdomPlotData:
    """Berechnet OdomPlotData fuer einen einzelnen Lauf."""

    # Erster Durchlauf: kumulative Startversaetze pro cmd_id berechnen
    ideal_start_by_id:  dict[int, tuple[float, float, float, float]] = {}
    actual_start_by_id: dict[int, tuple[float, float, float, float, float]] = {}

    ideal_x = ideal_y = ideal_s = ideal_t = 0.0
    actual_x = actual_y = actual_s = actual_t = actual_phi = 0.0

    for cmd_id in ordered_ids:
        ideal_start_by_id[cmd_id]  = (ideal_x, ideal_y, ideal_s, ideal_t)
        actual_start_by_id[cmd_id] = (actual_x, actual_y, actual_s, actual_t, actual_phi)

        cmd = cmdp_by_id.get(cmd_id)
        if cmd is not None:
            ux, uy, v_abs = _safe_unit(cmd.vx_cms, cmd.vy_cms)
            ideal_x += ux * cmd.target
            ideal_y += uy * cmd.target
            ideal_s += cmd.target
            if v_abs > 1.0e-9:
                ideal_t += cmd.target / v_abs

        seg_rows = rows_by_id_run.get(cmd_id, [])
        if seg_rows:
            last = seg_rows[-1]
            actual_x   += last.x_body_cm
            actual_y   += last.y_body_cm
            actual_s   += last.path_cm
            actual_t   += last.ms * 0.001
            actual_phi += last.phi_deg

    # Zweiter Durchlauf: Ausgabelisten befuellen
    out = OdomPlotData()

    for cmd_id in ordered_ids:
        seg_rows = rows_by_id_run.get(cmd_id, [])
        if not seg_rows:
            continue

        cmd = cmdp_by_id.get(cmd_id)
        if cmd is None:
            ux, uy, v_abs, target = 1.0, 0.0, 0.0, 0.0
        else:
            ux, uy, v_abs = _safe_unit(cmd.vx_cms, cmd.vy_cms)
            target = cmd.target

        ideal_x0,  ideal_y0,  _,  _           = ideal_start_by_id[cmd_id]
        actual_x0, actual_y0, actual_s0, actual_t0, actual_phi0 = actual_start_by_id[cmd_id]

        for row in seg_rows:
            t_local  = row.ms * 0.001
            t_global = actual_t0 + t_local

            if v_abs > 1.0e-9:
                s_soll_local = min(v_abs * t_local, target)
            else:
                s_soll_local = 0.0

            x_soll_local = ux * s_soll_local
            y_soll_local = uy * s_soll_local

            x_actual_global = actual_x0 + row.x_body_cm
            y_actual_global = actual_y0 + row.y_body_cm

            x_ideal_global = ideal_x0 + x_soll_local
            y_ideal_global = ideal_y0 + y_soll_local

            error_x = x_actual_global - x_ideal_global
            error_y = y_actual_global - y_ideal_global

            if v_abs > 1.0e-9:
                e_parallel =  error_x * ux + error_y * uy
                e_cross    = -error_x * uy + error_y * ux
            else:
                e_parallel = 0.0
                e_cross    = 0.0

            out.s_cm.append(actual_s0 + row.path_cm)
            out.t_s.append(t_global)
            out.x_ist_cm.append(x_actual_global)
            out.y_ist_cm.append(y_actual_global)
            out.x_soll_cm.append(x_ideal_global)
            out.y_soll_cm.append(y_ideal_global)
            out.e_parallel_cm.append(e_parallel)
            out.e_cross_cm.append(e_cross)
            out.phi_deg.append(actual_phi0 + row.phi_deg)
            out.cmd_id.append(cmd_id)

    if out.s_cm:
        out.text = (
            f"s={out.s_cm[-1]:.2f} cm   "
            f"e_laengs={out.e_parallel_cm[-1]:.2f} cm   "
            f"e_quer={out.e_cross_cm[-1]:.2f} cm   "
            f"e_verdrehen={out.phi_deg[-1]:.2f} deg"
        )

    return out


# ============================================================
# Oeffentliche Hauptfunktion
# ============================================================

# Listenfelder von OdomPlotData (ohne `text` und `cmd_id`), fuer den NaN-Merge.
_MERGE_FIELDS_FLOAT = (
    "s_cm", "t_s",
    "x_ist_cm", "y_ist_cm",
    "x_soll_cm", "y_soll_cm",
    "e_parallel_cm", "e_cross_cm",
    "phi_deg",
)


def build_odom_plot_data(snapshot: dict) -> OdomPlotData:
    cmdp_by_id = snapshot.get("cmdp_by_id", {})
    cmdp_order = snapshot.get("cmdp_order", [])
    odom2_rows = snapshot.get("odom2_rows", [])

    if not odom2_rows:
        return OdomPlotData(text="warte auf #ODOM2 ...")

    # Alle Zeilen nach cmd_id gruppieren
    rows_by_id: dict[int, list] = {}
    for row in odom2_rows:
        rows_by_id.setdefault(row.cmd_id, []).append(row)

    # Reihenfolge: erst die aus #CMDP_BEGIN, dann der Rest
    ordered_ids = [int(i) for i in cmdp_order if int(i) in rows_by_id]
    for cmd_id in rows_by_id:
        if cmd_id not in ordered_ids:
            ordered_ids.append(cmd_id)

    # Jeden cmd_id-Bucket per ms-Reset in einzelne Laeufe aufteilen
    runs_by_id = {cid: _split_rows_by_run(rows) for cid, rows in rows_by_id.items()}
    num_runs = max((len(r) for r in runs_by_id.values()), default=1)

    # Pro Lauf unabhaengig berechnen
    run_data_list = [
        _build_single_run_data(
            {cid: runs[i] if i < len(runs) else [] for cid, runs in runs_by_id.items()},
            cmdp_by_id,
            ordered_ids,
        )
        for i in range(num_runs)
    ]

    # Laeufe mit NaN-Trennpunkt zusammenfuehren
    NAN = float("nan")
    merged = OdomPlotData()

    for rd in run_data_list:
        if not rd.s_cm:
            continue

        if merged.s_cm:
            for f in _MERGE_FIELDS_FLOAT:
                getattr(merged, f).append(NAN)
            merged.cmd_id.append(-1)

        for f in _MERGE_FIELDS_FLOAT:
            getattr(merged, f).extend(getattr(rd, f))
        merged.cmd_id.extend(rd.cmd_id)

    # Statustext vom letzten nicht-leeren Lauf
    merged.text = next((rd.text for rd in reversed(run_data_list) if rd.text), "")

    return merged


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
    secax.set_xlabel("Zeit [s]" if show_label else "", fontsize=15, labelpad=6)
    secax.tick_params(axis="x", labelsize=13, pad=3)


def _format_axis(ax, ylabel: str, show_xlabel: bool = True) -> None:
    ax.set_ylabel(ylabel, fontsize=15, labelpad=6)
    ax.set_xlabel("Weg [cm]" if show_xlabel else "", fontsize=15, labelpad=6)
    ax.grid(True)
    ax.tick_params(axis="both", labelsize=13, pad=3)


# ============================================================
# Plot-Update
# ============================================================

def update_odom_plot(axes, store) -> None:
    if len(axes) < 3:
        return

    data = build_odom_plot_data(store.snapshot())

    fig = axes[0].figure
    ax_cross, ax_parallel, ax_phi = axes[0], axes[1], axes[2]

    for ax in (ax_cross, ax_parallel, ax_phi):
        ax.clear()

    _update_status_text(fig, data.text)

    _format_axis(ax_cross,    "e_quer [cm]",       show_xlabel=False)
    _format_axis(ax_parallel, "e_laengs [cm]",     show_xlabel=False)
    _format_axis(ax_phi,      "e_verdrehen [deg]", show_xlabel=True)

    if not data.s_cm:
        ax_cross.text(
            0.5, 0.5, data.text,
            transform=ax_cross.transAxes,
            ha="center", va="center", fontsize=13,
        )
        return

    for i, ax in enumerate((ax_cross, ax_parallel, ax_phi)):
        ax.axhline(0.0, linewidth=1.2)
        _draw_command_boundaries(ax, data.s_cm, data.cmd_id)
        ax.margins(x=0.01)
        _add_time_axis(ax, data, show_label=(i == 0))

    ax_cross.plot(data.s_cm, data.e_cross_cm, linewidth=1.8)
    ax_parallel.plot(data.s_cm, data.e_parallel_cm, linewidth=1.8)
    ax_phi.plot(data.s_cm, data.phi_deg, linewidth=1.8)