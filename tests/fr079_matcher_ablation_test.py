#!/usr/bin/env python3
"""Small deterministic Ceres ablation and original-beam phase regression."""
from __future__ import annotations
import json
from pathlib import Path
import sys
import tempfile

from fr079_offline_test import carmen_fixture, run


def main():
    binary = Path(sys.argv[1]).resolve(strict=True)
    modes = ("legacy", "finite-seed-legacy", "finite-all-legacy",
             "finite-seed-continuous", "continuous")
    with tempfile.TemporaryDirectory(prefix="fr079-match-ablation-") as temp:
        root = Path(temp)
        dataset = root / "fixture.log"
        dataset.write_text(carmen_fixture())
        for stride in (1, 4, 8):
            for mode in modes:
                tag = f"{mode}-s{stride}"
                flags = ["--match-every", "1", "--field-every", "15",
                         "--map-stride", "1", "--match-stride", str(stride),
                         "--match-phase", "0", "--distance-truncation", "0.5"]
                if mode != "legacy":
                    flags.append("--continuous-cost")
                    if mode != "continuous":
                        flags.extend(("--ablation", mode))
                prefix = root / tag
                run(binary, dataset, prefix, *flags)
                report = json.loads(prefix.with_suffix(".report.json").read_text())
                matches = json.loads(prefix.with_suffix(".matches.json").read_text())["frames"]
                assert report["frames"] == 55
                assert report["match_beam_stride"] == stride
                assert report["match_beam_phase"] == 0
                assert report["map_beam_stride"] == 1
                assert report["ablation_mode"] == (
                    mode if mode not in ("legacy", "continuous") else "none")
                assert report["accepted"] + report["rejected"] == report["attempts"]
                assert len(matches) == 55
                for frame in matches:
                    n, blocks = frame["match_input_beams"], frame["solver_residual_blocks"]
                    assert 0 <= blocks <= n <= (360 + stride - 1)//stride
                    if frame["accepted"]:
                        assert frame["candidate_evaluated"]
                        assert frame["candidate_supported_hits"] >= frame["seed_supported_hits"]
                        assert frame["candidate_translation_m"] <= 0.4 + 1e-8
                    if mode != "legacy":
                        assert frame["ceres_invalid_residual_evaluations"] == 0
                    if frame["candidate_evaluated"]:
                        assert frame["candidate_translation_m"] >= 0
                        assert frame["candidate_rotation_rad"] >= 0
                assert sum(f["ceres_invalid_residual_evaluations"] for f in matches) == (
                    report["ceres_invalid_residual_evaluations"])
                if mode == "continuous":
                    assert sum(f["solver_residual_blocks"] for f in matches) == sum(
                        f["match_input_beams"] for f in matches if f["attempted"])
        # Verify phase selection changes the raw sampled indices without
        # modifying mapping-beam stride or the CARMEN input.
        run(binary, dataset, root/"shifted", "--continuous-cost",
            "--match-every", "1", "--field-every", "15",
            "--map-stride", "1", "--match-stride", "4",
            "--match-phase", "1", "--distance-truncation", "0.5")
        shifted = json.loads((root/"shifted.report.json").read_text())
        assert shifted["match_beam_phase"] == 1
        run(binary, dataset, root/"invalid-ablation", "--ablation",
            "finite-all-legacy", expected_success=False)
        run(binary, dataset, root/"invalid-phase",
            "--match-stride", "4", "--match-phase", "4",
            expected_success=False)
        run(binary, dataset, root/"invalid-combination",
            "--continuous-cost", "--support-aware",
            "--ablation", "finite-seed-legacy", expected_success=False)
        print("FR079 factorial matcher/beam/phase fixture passed")


if __name__ == "__main__":
    main()
