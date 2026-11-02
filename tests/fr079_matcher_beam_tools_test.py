#!/usr/bin/env python3
"""No-external-data unit checks for FR079 matcher/beam benchmark driver."""
from __future__ import annotations
from argparse import Namespace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from fr079_matcher_beam_benchmark import audit_frames, command, first_divergence


def make_frame(index, *, accepted=False, candidate=True, beam=90, blocks=65):
    return {
        "index": index, "attempted": True, "accepted": accepted,
        "status": "accepted" if accepted else "no_improvement",
        "match_input_beams": beam, "solver_residual_blocks": blocks,
        "candidate_evaluated": candidate, "seed_supported_hits": 65,
        "candidate_supported_hits": 66, "seed_strict_cost": 120.,
        "candidate_strict_cost": 110., "seed_finite_cost": 160.,
        "candidate_finite_cost": 150., "candidate_translation_m": 0.02,
        "candidate_rotation_rad": 0.01,
        "ceres_invalid_residual_evaluations": 0,
        "ceres_continuous_unknown_evaluations": 22,
    }


def main():
    a = Namespace(log=Path("/tmp/fr079.log.gz"),
                  out_dir=Path("/tmp/fr079-beams"),
                  docker=True, binary=None,
                  field_every=15, map_stride=1, distance_truncation=0.5)
    cmd = command(a, "trial", "finite-seed-legacy", 4, 1)
    assert "--continuous-cost" in cmd
    assert cmd[cmd.index("--ablation")+1] == "finite-seed-legacy"
    assert cmd[cmd.index("--match-stride")+1] == "4"
    assert cmd[cmd.index("--match-phase")+1] == "1"
    assert cmd[cmd.index("--map-stride")+1] == "1"
    cmd2 = command(a, "baseline", "legacy", 4, 0)
    assert "--ablation" not in cmd2 and "--continuous-cost" not in cmd2
    poses = [{"index": 0, "prior": [0.,0.,0.], "estimated": [0.,0.,0.]},
             {"index": 1, "prior": [1.,0.,0.], "estimated": [1.,0.,0.]}]
    frames = [make_frame(0), make_frame(1, accepted=True)]
    audit = audit_frames("trial","finite-seed-legacy",4,0,frames,poses)
    assert len(audit) == 2
    assert audit[1]["strict_delta"] == -10
    assert audit[1]["finite_delta"] == -10
    assert first_divergence(audit, audit) == (None, "identical_trajectory")
    changed = [dict(f) for f in audit]
    changed[1]["accepted"] = 0
    assert first_divergence(audit, changed) == (1, "first_correction_divergence")
    shifted = [dict(f) for f in audit]
    shifted[1]["prior"] = [1.1, 0., 0.]
    assert first_divergence(audit, shifted) == (1, "different_prior")
    try:
        audit_frames("bad","continuous",4,0,
                     [make_frame(0,beam=91)], [poses[0]])
    except ValueError:
        pass
    else:
        raise AssertionError("raw-beam upper bound not enforced")
    print("FR079 matcher/beam benchmark driver checks passed")


if __name__ == "__main__":
    main()
