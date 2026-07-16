"""Emit golden per-target weights from the authoritative Python module.

The MATLAB cross-language test invokes this file and compares the numeric
channel vectors directly.  Keeping the fixture executable avoids hand-copying
formula results into a second implementation.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np


MODULE_PATH = Path(__file__).with_name("weight_trust_module.py")
SPEC = importlib.util.spec_from_file_location("weight_reference", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def config(weight_type: str = "trust_based", kappa: int = 3):
    return MODULE.WeightConfig(
        weight_type=weight_type,
        w0_fixed=0.4,
        w_self_base=0.2,
        w_cap=0.4,
        kappa=kappa,
        trust_threshold=0.5,
        eta=0.15,
        enable_smoothing=False,
        startup_fixed_duration_s=0.5,
        use_gamma_self_weight_adaptation=True,
        gamma_self_weight_floor=0.25,
        include_target_self_fleet_estimate=True,
        local_bad_zero_w0_neighbor_total_cap=0.05,
        flag_w0_target_attack_factor=0.25,
        flag_w0_global_est_check_factor=1.25,
        flag_w0_local_est_check_factor=0.5,
    )


def trust(local: float, gamma_self: float, *, target=False, local_bad=False, global_bad=False):
    return SimpleNamespace(
        local_trust_sample=local,
        gamma_self=gamma_self,
        flag_target_attack=target,
        flag_local_est_check=local_bad,
        flag_global_est_check=global_bad,
    )


def vector(weights: dict, host_id: int, fleet_size: int):
    result = np.zeros(fleet_size + 1, dtype=float)
    result[0] = weights["w0"]
    result[host_id + 1] = weights["w_self"]
    for source_id, value in weights["neighbors"].items():
        result[int(source_id) + 1] = value
    return result.tolist()


def main() -> None:
    host_id = 0
    target_id = 1
    fleet_size = 4
    trust_scores = {0: 1.0, 1: 0.9, 2: 0.8, 3: 0.7}
    available = {1: {1: {}}, 2: {1: {}}, 3: {1: {}}}
    direct = np.zeros(5)

    module = MODULE.WeightTrustModule(host_id, fleet_size, config())
    normal = module.calculate_weights_for_target(
        target_id, trust_scores, available, direct, trust(0.8, 0.6)
    )
    local_bad = module.calculate_weights_for_target(
        target_id,
        trust_scores,
        available,
        None,
        trust(0.1, 0.4, local_bad=True),
    )
    global_bad = module.calculate_weights_for_target(
        target_id,
        trust_scores,
        available,
        direct,
        trust(1.0, 1.0, global_bad=True),
    )
    startup = module.calculate_startup_weights_for_target(
        target_id, available, direct
    )

    paper_module = MODULE.WeightTrustModule(
        host_id, fleet_size, config(weight_type="paper", kappa=2)
    )
    paper = paper_module.calculate_paper_weights_for_target(
        target_id=target_id,
        opinion_scores=trust_scores,
        target_local_trust=0.9,
        neighbor_fleet_estimates=available,
        direct_measurement=direct,
    )

    payload = {
        "normal": vector(normal, host_id, fleet_size),
        "local_bad": vector(local_bad, host_id, fleet_size),
        "global_bad": vector(global_bad, host_id, fleet_size),
        "startup": vector(startup, host_id, fleet_size),
        "paper": vector(paper, host_id, fleet_size),
    }
    print(json.dumps(payload, sort_keys=True))


if __name__ == "__main__":
    main()
