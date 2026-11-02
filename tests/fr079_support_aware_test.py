#!/usr/bin/env python3
"""End-to-end support-aware FR079 Ceres regression on deterministic fixture."""
from __future__ import annotations
import json
from pathlib import Path
import sys
import tempfile

from fr079_offline_test import carmen_fixture, run


def main():
    binary=Path(sys.argv[1]).resolve(strict=True)
    with tempfile.TemporaryDirectory(prefix="fr079-support-aware-") as temp:
        root=Path(temp)
        dataset=root/"fixture.log"
        dataset.write_text(carmen_fixture())
        standard=root/"legacy"
        guarded=root/"guarded"
        continuous=root/"continuous"
        flags=("--match-every","1","--field-every","15",
               "--map-stride","3","--match-stride","4")
        run(binary,dataset,standard,*flags)
        run(binary,dataset,guarded,*flags,"--support-aware")
        run(binary,dataset,continuous,*flags,"--continuous-cost")
        old=json.loads(standard.with_suffix(".report.json").read_text())
        new=json.loads(guarded.with_suffix(".report.json").read_text())
        smooth=json.loads(continuous.with_suffix(".report.json").read_text())
        assert old["matcher"]=="ceres"
        assert new["matcher"]=="ceres_support_aware"
        assert smooth["matcher"]=="ceres_continuous_cost"
        assert smooth["frames"]==old["frames"]==55
        assert smooth["source_sha256"]==old["source_sha256"]
        assert smooth["map_beam_stride"]==old["map_beam_stride"]==3
        assert smooth["match_beam_stride"]==old["match_beam_stride"]==4
        assert smooth["inserted_scans"]==55
        assert smooth["accepted"]+smooth["rejected"]==smooth["attempts"]
        assert smooth["ceres_invalid_residual_evaluations"]==0, (
            "continuous cost solver must not fail at finite missing field cells")
        for category in ("unknown", "truncated", "outside", "other"):
            assert smooth["ceres_continuous_"+category+"_evaluations"]>=0
        smooth_frames=json.loads(
            continuous.with_suffix(".matches.json").read_text())["frames"]
        for category in ("unknown", "truncated", "outside", "other"):
            key="ceres_continuous_"+category+"_evaluations"
            assert sum(f[key] for f in smooth_frames)==smooth[key]
        assert sum(f["ceres_invalid_residual_evaluations"]
                   for f in smooth_frames)==smooth["ceres_invalid_residual_evaluations"]
        assert new["frames"]==old["frames"]==55
        assert new["source_sha256"]==old["source_sha256"]
        assert new["distance_truncation_m"]==old["distance_truncation_m"]==0.7
        assert new["match_every"]==old["match_every"]==1
        assert new["field_rebuild_every"]==old["field_rebuild_every"]==15
        assert new["map_beam_stride"]==old["map_beam_stride"]==3
        assert new["match_beam_stride"]==old["match_beam_stride"]==4
        assert new["inserted_scans"]==old["inserted_scans"]==55
        assert new["accepted"]+new["rejected"]==new["attempts"]
        assert new["support_aware_accepted_stages"]<=new["support_aware_stages"]
        assert new["ceres_invalid_residual_evaluations"]==sum(
            new["ceres_invalid_"+key] for key in
            ("unknown_cells","truncated","outside_grid","other"))
        assert new["ceres_invalid_residual_evaluations"]==0, (
            "support-aware certified Ceres evaluated an invalid EDT residual")
        decisions=json.loads(guarded.with_suffix(".matches.json").read_text())
        assert len(decisions["frames"])==55
        assert sum(f["support_aware_stages"] for f in decisions["frames"])==new[
            "support_aware_stages"]
        assert sum(f["support_aware_accepted_stages"] for f in decisions["frames"])==new[
            "support_aware_accepted_stages"]
        assert sum(f["ceres_invalid_residual_evaluations"]
                   for f in decisions["frames"])==0
        run(binary,dataset,root/"invalid","--reference","--support-aware",
            expected_success=False)
        run(binary,dataset,root/"conflict","--support-aware","--continuous-cost",
            expected_success=False)
        run(binary,dataset,root/"bad-reference","--reference","--continuous-cost",
            expected_success=False)
        print("FR079 legacy, support-aware and finite-cost Ceres fixture passed")


if __name__=="__main__":
    main()
