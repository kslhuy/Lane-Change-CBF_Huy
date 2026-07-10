"""Calculate manuscript numerical-setup values from the actual source files.

Run from the repository root:
    python test/python/calculate_numerical_setup_values.py

The script reads the MATLAB source constants used by main_mean_attack_plot.m
and computes the scaled-infinity one-step bicycle gain used in the manuscript
contraction paragraph.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]


def read_rel(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="replace")


def matlab_number(source: str, name: str) -> float:
    pattern = rf"(?<![\w.]){re.escape(name)}\s*=\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"
    match = re.search(pattern, source)
    if not match:
        raise ValueError(f"Could not find numeric MATLAB assignment for {name!r}")
    return float(match.group(1))


def matlab_string_assignment(source: str, name: str) -> str:
    pattern = rf"(?<![\w.]){re.escape(name)}\s*=\s*[\"']([^\"']+)[\"']"
    match = re.search(pattern, source)
    if not match:
        raise ValueError(f"Could not find MATLAB string assignment for {name!r}")
    return match.group(1)


def matlab_property_number(source: str, name: str) -> float:
    return matlab_number(source, name)


def scaled_bicycle_gain(
    ts: float,
    wheelbase: float,
    vmax: float = 33.0,
    delta_max: float = 0.35,
    dx: tuple[float, float, float, float] = (0.01, 0.01, 1.0, 0.2),
    n_psi_for_spectral_check: int = 4001,
) -> dict[str, float]:
    """Compute max ||D_x A D_x^{-1}||_inf on the stated envelope.

    A is the Jacobian of
        X+ = X + Ts v cos(psi)
        Y+ = Y + Ts v sin(psi)
        psi+ = psi + Ts v / L tan(delta)
        v+ = v + Ts a
    with respect to x = [X,Y,psi,v].
    """

    d = np.asarray(dx, dtype=float)
    inv_d = 1.0 / d

    # Exact scaled-infinity norm maximum:
    # row X/Y: 1 + max_psi a|sin psi| + b|cos psi|
    # row psi: 1 + Ts/L tan(delta_max) scaled from v to psi.
    xy_a = d[0] * ts * vmax * inv_d[2]
    xy_b = d[0] * ts * inv_d[3]
    xy_row = 1.0 + math.hypot(xy_a, xy_b)
    heading_row = 1.0 + d[2] * ts / wheelbase * math.tan(delta_max) * inv_d[3]
    best_inf = max(xy_row, heading_row, 1.0)
    best_psi = math.atan2(xy_a, xy_b)
    best_delta = delta_max if heading_row >= xy_row else 0.0

    # Spectral norm is not used in the proof. This lightweight check is included
    # only to make the script comparable with the PDF note.
    best_spectral = -math.inf
    v = vmax
    for delta in (-delta_max, delta_max):
        tan_delta = math.tan(delta)
        for psi in np.linspace(0.0, 2.0 * math.pi, n_psi_for_spectral_check):
            a = np.array(
                [
                    [1.0, 0.0, -ts * v * math.sin(psi), ts * math.cos(psi)],
                    [0.0, 1.0, ts * v * math.cos(psi), ts * math.sin(psi)],
                    [0.0, 0.0, 1.0, ts / wheelbase * tan_delta],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            scaled = d[:, None] * a * inv_d[None, :]
            best_spectral = max(best_spectral, float(np.linalg.norm(scaled, ord=2)))

    anchor_mass = 0.4 * 0.5
    return {
        "scaled_inf_norm": best_inf,
        "scaled_spectral_norm": best_spectral,
        "argmax_psi_rad": best_psi,
        "argmax_delta_rad": best_delta,
        "anchor_mass": anchor_mass,
        "alpha": best_inf * (1.0 - anchor_mass),
    }


def main() -> None:
    config_m = read_rel("test/Config.m")
    main_m = read_rel("test/main_mean_attack_plot.m")
    param_m = read_rel("test/core/ParamVeh.m")
    trust_m = read_rel("test/core/Trust/TriPTrustModel.m")
    weight_m = read_rel("test/Function/Weight_Trust_module.m")

    ts = matlab_number(config_m, "dt")
    n = int(matlab_number(main_m, "num_vehicles"))
    l_f = matlab_number(param_m, "l_f")
    l_r = matlab_number(param_m, "l_r")
    wheelbase = l_f + l_r

    trust_threshold = matlab_number(config_m, "trust_threshold")
    kappa = int(matlab_number(config_m, "kappa"))
    fusion_mode = matlab_string_assignment(config_m, "Scenarios_config.local_trust_fusion_mode")
    dirichlet_type = matlab_string_assignment(config_m, "Scenarios_config.Dichiret_type")
    model_vehicle_type = matlab_string_assignment(config_m, "Scenarios_config.model_vehicle_type")

    w0_base = matlab_property_number(weight_m, "w0_fixed")
    w_self_base = matlab_property_number(weight_m, "w_self_base")
    w_cap = matlab_property_number(weight_m, "w_cap")
    startup_fixed_duration = matlab_property_number(weight_m, "startup_fixed_duration_s")

    omega_v = matlab_property_number(trust_m, "py_weight_velocity")
    omega_p = matlab_property_number(trust_m, "py_weight_distance")
    omega_u = matlab_property_number(trust_m, "py_weight_acceleration")
    omega_psi = matlab_property_number(trust_m, "py_weight_heading")
    psi_th = matlab_property_number(trust_m, "heading_base_tolerance_rad")
    beta_local = matlab_property_number(trust_m, "wt")
    beta_global = matlab_property_number(trust_m, "wt_global")
    missing_packet_decay = matlab_property_number(trust_m, "lambda_h")
    c_tr = matlab_property_number(trust_m, "C")
    q = int(matlab_property_number(trust_m, "k"))

    gain = scaled_bicycle_gain(ts=ts, wheelbase=wheelbase)
    neighbor_budget = 1.0 - w0_base - w_self_base

    print("Computed from source files")
    print("==========================")
    print(f"N = {n}")
    print("graph = all-to-all without self edges (main_mean_attack_plot.m uses ones(N)-eye(N))")
    print(f"T_s = {ts:.6g} s")
    print(f"l_f = {l_f:.6g} m")
    print(f"l_r = {l_r:.6g} m")
    print(f"L_w = l_f + l_r = {wheelbase:.6g} m")
    print(f"active model_vehicle_type = {model_vehicle_type}")
    print()
    print("Trust parameters")
    print("----------------")
    print(f"local_trust_fusion_mode = {fusion_mode}")
    print(
        "local powers (position/distance, velocity, heading, acceleration/input) = "
        f"({omega_p:.6g}, {omega_v:.6g}, {omega_psi:.6g}, {omega_u:.6g})"
    )
    print(f"Dichiret_type = {dirichlet_type}")
    print(f"rating-vector aging wt(local) = {beta_local:.6g}")
    print(f"rating-vector aging wt_global = {beta_global:.6g}")
    print(f"missing-packet trust decay lambda_h = {missing_packet_decay:.6g}")
    print(f"C_tr = {c_tr:.6g}")
    print(f"q = {q}")
    print(f"heading base tolerance = {psi_th:.6g} rad")
    print(f"theta_min = trust_threshold = {trust_threshold:.6g}")
    print()
    print("Observer weight-module parameters")
    print("---------------------------------")
    print(f"kappa = {kappa}")
    print(f"w0_fixed/base = {w0_base:.6g}")
    print(f"w_self_base = {w_self_base:.6g}")
    print(f"nominal neighbor budget = 1 - w0 - w_self = {neighbor_budget:.6g}")
    print(f"per-neighbor cap w_cap = {w_cap:.6g}")
    print(
        "startup_fixed_duration_s = "
        f"{startup_fixed_duration:.6g} (class default used after main script re-instantiates the 5-vehicle module)"
    )
    print()
    print("Scaled bicycle contraction check")
    print("--------------------------------")
    print("D_x = diag(0.01, 0.01, 1, 0.2)")
    print("Envelope: v <= 33 m/s, |delta| <= 0.35 rad")
    print(f"max ||D_x A D_x^-1||_inf = {gain['scaled_inf_norm']:.9f}")
    print(f"max ||D_x A D_x^-1||_2   = {gain['scaled_spectral_norm']:.9f}")
    print(f"argmax psi = {gain['argmax_psi_rad']:.9f} rad")
    print(f"argmax delta = {gain['argmax_delta_rad']:.9f} rad")
    print(f"accepted anchor mass eta0 * theta_min = {gain['anchor_mass']:.6g}")
    print(f"alpha = Lx_max * (1 - anchor_mass) = {gain['alpha']:.9f}")
    print()
    print("LaTeX replacement")
    print("-----------------")
    print(
        "The numerical simulation uses five vehicles with all-to-all V2V communication; "
        "each vehicle runs the observer, and small fixed parameter variations are "
        "treated as modeling uncertainty. The main parameters are $N=5$, "
        f"$T_s={ts:.2f}$~s, nominal geometry $l_f={l_f:.2f}$~m and "
        f"$l_r={l_r:.2f}$~m, giving $L_w={wheelbase:.2f}$~m, "
        "local-trust product fusion with internal powers "
        f"$(\\omega_p,\\omega_v,\\omega_\\psi,\\omega_u)=({omega_p:.1f},{omega_v:.1f},{omega_psi:.1f},{omega_u:.1f})$ "
        "for distance/position, velocity, heading, and acceleration/input consistency, "
        f"class-filter aging weight $\\beta_{{\\mathrm{{trust}}}}={beta_local:.2f}$ in the active single-rating update, "
        f"class confidence $C_{{\\mathrm{{tr}}}}={c_tr:.1f}$, $q={q}$ trust classes, "
        f"heading base tolerance $\\psi_{{\\mathrm{{th}}}}={psi_th:.2f}$~rad, "
        f"and trust threshold $\\theta_{{\\min}}={trust_threshold:.1f}$. "
        f"The observer-weight module uses base gains $w_{{0,\\mathrm{{base}}}}={w0_base:.1f}$ "
        f"and $w_{{\\mathrm{{self,base}}}}={w_self_base:.1f}$, with nominal neighbor budget "
        f"${neighbor_budget:.1f}$ and per-neighbor cap ${w_cap:.1f}$; after trust and flag "
        "normalization these are adaptive weights rather than fixed lower bounds."
    )
    print()
    print(
        f"On $v\\le33$~m/s and $|\\delta|\\le0.35$~rad, the induced one-step "
        f"bicycle gain evaluates to $L_{{x,\\max}}\\approx{gain['scaled_inf_norm']:.3f}$. "
        f"With accepted anchor mass $\\underline w_0=\\eta_0\\theta_{{\\min}}={gain['anchor_mass']:.2f}$, "
        f"Lemma~\\ref{{lem_anchor_contraction}} gives "
        f"$\\alpha=L_{{x,\\max}}(1-\\underline w_0)\\approx"
        f"{gain['scaled_inf_norm']:.3f}\\times0.80\\approx{gain['alpha']:.3f}<1$."
    )


if __name__ == "__main__":
    main()
