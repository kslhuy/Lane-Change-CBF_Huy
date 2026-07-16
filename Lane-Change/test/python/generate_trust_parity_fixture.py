"""Emit trust-kernel golden values from the authoritative Python model."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("trust_model.py")
SPEC = importlib.util.spec_from_file_location("trust_reference", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def main() -> None:
    config = MODULE.TrustConfig(
        dirichlet_method="matlab",
        dirichlet_type="Dual",
        dirichlet_C=0.2,
        dirichlet_wt_local=0.4,
        dirichlet_wt_global=0.5,
        ema_alpha=0.5,
        trust_threshold=0.5,
    )
    model = MODULE.TriPTrustModel(vehicle_id=0, config=config)

    score = MODULE.TrustScore(
        vehicle_id=1,
        velocity_score=0.8,
        distance_score=0.6,
        acceleration_score=0.4,
        heading_score=0.9,
        beacon_score=1.0,
        quality_factor=0.7,
    )
    local_sample = model._compute_local_trust_sample(score)

    sample_sequence = [(1.0, 1.0), (0.2, 0.8), (0.1, 0.1), (0.9, 0.3)]
    final_scores = []
    flag_rows = []
    previous = None
    for local_value, global_value in sample_sequence:
        final = model._dirichlet_matlab(1, local_value, global_value)
        if previous is not None:
            final = config.ema_alpha * final + (1.0 - config.ema_alpha) * previous
        previous = final
        final_scores.append(float(final))

        flags = MODULE.TrustScore(
            vehicle_id=1,
            local_trust_sample=local_value,
            global_trust_sample=global_value,
        )
        model._set_attack_flags(flags)
        flag_rows.append(
            [
                bool(flags.flag_target_attack),
                bool(flags.flag_global_est_check),
                bool(flags.flag_local_est_check),
            ]
        )

    print(
        json.dumps(
            {
                "local_sample": float(local_sample),
                "sample_sequence": sample_sequence,
                "final_scores": final_scores,
                "flags": flag_rows,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
