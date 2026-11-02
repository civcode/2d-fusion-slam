#!/usr/bin/env python3
"""Fusion sweep command construction and argument-validation regression."""
from __future__ import annotations

from argparse import Namespace
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fusion_match_beam_sweep import command, validate, parser


def main():
    with tempfile.TemporaryDirectory(prefix="fusion-sweep-tool-") as tmp:
        options = Namespace(
            repo=Path(__file__).resolve().parents[1],
            out_dir=Path(tmp)/"unused", docker=True, binary=None,
            scenes=["apartment","warehouse"], matchers=["legacy","continuous"],
            match_strides=[1,2,4,8], map_stride=1,
            match_every=1, field_every=1, distance_truncation=.7)
        validate(options, parser())
        for scene in options.scenes:
            for mode in options.matchers:
                cmd=command(options,scene,mode,4,scene+"-"+mode)
                assert cmd[:4]==["docker","compose","run","--rm"]
                assert cmd[cmd.index("--map-stride")+1]=="1"
                assert cmd[cmd.index("--match-stride")+1]=="4"
                assert cmd[cmd.index("--field-every")+1]=="1"
                assert cmd[cmd.index("--match-every")+1]=="1"
                assert cmd[-1]=="--continuous-cost" if mode=="continuous" else (
                    "--continuous-cost" not in cmd)
                assert scene in cmd
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
