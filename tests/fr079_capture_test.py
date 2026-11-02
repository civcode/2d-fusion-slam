#!/usr/bin/env python3
"""Exercise the FR079 truncation controls and frozen-map perturbation CSV."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import tempfile

from fr079_offline_test import carmen_fixture, run


def main():
    binary = Path(sys.argv[1]).resolve(strict=True)
    with tempfile.TemporaryDirectory(prefix="fr079-capture-test-") as temp:
        root = Path(temp)
        log = root / "synthetic.log"
        log.write_text(carmen_fixture())
        prefix = root / "default"
        run(binary, log, prefix, "--reference", "--match-every", "1",
            "--field-every", "15")
        default = json.loads(prefix.with_suffix(".report.json").read_text())
        assert default["distance_truncation_m"] == 0.7
        assert default["ceres_invalid_residual_evaluations"] == 0
        assert sum(default["ceres_invalid_" + name] for name in
                   ("unknown_cells", "truncated", "outside_grid", "other")) == 0

        exact = root / "explicit"
        run(binary, log, exact, "--reference", "--match-every", "1",
            "--field-every", "15", "--distance-truncation", "0.7")
        explicit = json.loads(exact.with_suffix(".report.json").read_text())
        assert explicit["distance_truncation_m"] == 0.7
        assert prefix.with_suffix(".slammap").read_bytes() == \
            exact.with_suffix(".slammap").read_bytes(), (
                "explicit 0.7 m changed the existing default map")

        capture = root / "capture"
        run(binary, log, capture, "--reference", "--match-every", "1",
            "--field-every", "15", "--distance-truncation", "1.0",
            "--capture", "--capture-every", "1", "--capture-max-frames", "1")
        report = json.loads(capture.with_suffix(".report.json").read_text())
        assert report["distance_truncation_m"] == 1.0
        csv_path = Path(str(capture) + ".capture.csv")
        with csv_path.open(newline="") as stream:
            records = list(csv.DictReader(stream))
        assert len(records) == 6 * 2 * 6 * 6 * 2, len(records)
        assert {float(row["truncation_m"]) for row in records} == \
               {0.35, 0.5, 0.7, 1.0, 1.5, 2.0}
        assert {float(row["search_bound_xy_m"]) for row in records} == \
               {0.12, 0.5}
        assert {float(row["initial_xy_offset_m"]) for row in records} == \
               {0., 0.1, 0.2, 0.4, 0.7, 1.}
        assert {float(row["initial_yaw_offset_deg"]) for row in records} == \
               {0., 1., 2., 3., 5., 8.}
        assert {row["source_sha256"] for row in records} == \
               {report["source_sha256"]}
        assert {row["anchor_kind"] for row in records} == \
               {"same_run_accepted_pose_proxy_not_ground_truth"}
        assert all(int(row["source_revision"]) < int(row["frame"]) + 1
                   for row in records)
        assert {row["matcher"] for row in records} == {"reference"}
        assert {row["ceres_invalid_residual_evaluations"] for row in records} == {"0"}
        assert {row["ceres_termination"] for row in records} == {"not_run"}
        assert any(int(row["initial_truncated_hits"]) > 0 for row in records)
        assert any(float(row["initial_overlap"]) >= 0.5 for row in records)
        assert any(float(row["initial_overlap"]) < 0.5 for row in records)
        assert all(0 <= float(row["initial_overlap"]) <= 1 for row in records)
        assert all(row["match_status"] in {
            "accepted", "insufficient_support", "degenerate",
            "no_improvement", "invalid_input"} for row in records)
        assert all(row["recovered_proxy"] in ("0", "1") for row in records)
        assert all(float(row["trial_ms"]) >= 0 for row in records)
        run(binary, log, root / "invalid-low", "--reference",
            "--distance-truncation", "0.19", expected_success=False)
        run(binary, log, root / "invalid-high", "--reference",
            "--distance-truncation", "3.01", expected_success=False)
        run(binary, log, root / "invalid-nan", "--reference",
            "--distance-truncation", "nan", expected_success=False)
        run(binary, log, capture, "--reference", "--capture",
            expected_success=False)
        print("FR079 0.7 default, explicit cutoff, controlled perturbations, "
              "support classification and CLI fail-closed checks passed")


if __name__ == "__main__":
    main()
