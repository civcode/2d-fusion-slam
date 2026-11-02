#!/usr/bin/env python3
"""Fusion sweep command construction and argument-validation regression."""
from __future__ import annotations

from argparse import Namespace
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fusion_match_beam_sweep import command, validate, parser
from fusion_bootstrap_compare import compare
from fusion_beam_generalization import summarize
from contextlib import redirect_stdout
from io import StringIO


def main():
    with tempfile.TemporaryDirectory(prefix="fusion-sweep-tool-") as tmp:
        options = Namespace(
            repo=Path(__file__).resolve().parents[1],
            out_dir=Path(tmp)/"unused", docker=True, binary=None,
            scenes=["apartment","warehouse"], matchers=["legacy","continuous"],
            match_strides=[1,2,4,8], map_stride=1,
            match_every=1, field_every=1, distance_truncation=.7,
            lidar_only_bootstrap=False, lidar_only_predict_first=False,
            match_offsets=[0,1,2,3])
        validate(options, parser())
        for scene in options.scenes:
            for mode in options.matchers:
                cmd=command(options,scene,mode,4,scene+"-"+mode)
                assert cmd[:4]==["docker","compose","run","--rm"]
                assert cmd[cmd.index("--map-stride")+1]=="1"
                assert cmd[cmd.index("--match-stride")+1]=="4"
                assert cmd[cmd.index("--match-offset")+1]=="0"
                shifted=command(options,scene,mode,4,scene+"-"+mode,2)
                assert shifted[shifted.index("--match-offset")+1]=="2"
                assert cmd[cmd.index("--field-every")+1]=="1"
                assert cmd[cmd.index("--match-every")+1]=="1"
                assert cmd[-1]=="--continuous-cost" if mode=="continuous" else (
                    "--continuous-cost" not in cmd)
                assert "--lidar-only-bootstrap" not in cmd
                options.lidar_only_bootstrap=True
                boot=command(options,scene,mode,4,scene+"-"+mode)
                assert "--lidar-only-bootstrap" in boot
                options.lidar_only_predict_first=True
                predicted=command(options,scene,mode,4,scene+"-"+mode)
                assert predicted[-2:]==[
                    "--lidar-only-bootstrap","--lidar-only-predict-first"]
                options.lidar_only_predict_first=False
                options.lidar_only_bootstrap=False
                assert scene in cmd
        # Duplicate effective phases must be skipped by the sweep:
        # stride=1 has one, stride=2 has two, stride=4 has four.
        from fusion_match_beam_sweep import SCENES
        assert {"office","loft","atrium","industrial",
                "warehouse_asymmetric"} <= set(SCENES)
        options.match_offsets=[0,1,1]
        try:
            validate(options,parser())
        except SystemExit:
            pass
        else:
            raise AssertionError("duplicate match phase accepted")
        options.match_offsets=[0,1,2,3]
        options.lidar_only_predict_first=True
        try:
            validate(options,parser())
        except SystemExit:
            pass
        else:
            raise AssertionError("predict-first without bootstrap accepted")
        options.lidar_only_predict_first=False
        # Compare tool must fail if any non-LiDAR-only fingerprint changes.
        old={}
        new={}
        for method in ("wheel_only","gyro_wheel","fused_scan_matched","lidar_only"):
            for stride in (1,2):
                key=("apartment","continuous",stride,method)
                row={
                    "scene":"apartment","matcher":"continuous",
                    "nominal_beams":str(240//stride),
                    "match_stride":str(stride),
                    "map_fingerprint":f"fixed-{method}",
                    "translation_rmse_m":"0.2",
                    "occupied_iou":"0.3",
                    "lidar_only_accepted":"2",
                }
                old[key]=row.copy()
                new[key]=row.copy()
        compare(old,new)
        broken={key:value.copy() for key,value in new.items()}
        bad=("apartment","continuous",1,"wheel_only")
        broken[bad]["map_fingerprint"]="unexpected"
        try:
            compare(old,broken)
        except ValueError as exc:
            assert "control map changed" in str(exc)
        else:
            raise AssertionError("modified wheel-only map was not rejected")
        # Cross-scene robustness comparisons use paired phase-zero
        # beam densities, not misleading pooled unpaired averages.
        evidence=[
            {"scene":"warehouse","matcher":"continuous","match_stride":"4",
             "match_offset":"0","map_method":"lidar_only",
             "translation_rmse_m":"4.0","occupied_iou":"0.04"},
            {"scene":"warehouse","matcher":"continuous","match_stride":"1",
             "match_offset":"0","map_method":"lidar_only",
             "translation_rmse_m":"1.0","occupied_iou":"0.1"},
            {"scene":"warehouse","matcher":"continuous","match_stride":"4",
             "match_offset":"1","map_method":"lidar_only",
             "translation_rmse_m":"5.0","occupied_iou":"0.03"},
        ]
        with StringIO() as out, redirect_stdout(out):
            result=summarize(evidence)
            output=out.getvalue()
        assert ("warehouse","continuous",0) in result
        assert ("warehouse","continuous",1) in result
        assert "scene-level 1/1 improved" in output
        assert "stride  4 ->  1: 0/1 worse" in output
        options.map_stride=0
        try:
            validate(options,parser())
        except SystemExit:
            pass
        else:
            raise AssertionError("invalid map stride accepted")
    print("fusion beam sweep tool checks passed")


if __name__ == "__main__":
    main()
