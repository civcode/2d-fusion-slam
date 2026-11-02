#!/usr/bin/env python3
"""Smoke-test each C++ command-line frontend without invoking its workload.

Exercises cxxopts help/unknown-flag handling and requires the normal help
path to return successfully, without creating files or touching datasets.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def invoke(binary: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(binary), *args], text=True, capture_output=True,
        timeout=15, check=False,
    )


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: test_cli_cxxopts.py EXECUTABLE")
    binary = Path(sys.argv[1]).resolve(strict=True)
    help_result = invoke(binary, "--help")
    assert help_result.returncode == 0, (
        f"{binary.name} --help failed with {help_result.returncode}: "
        f"{help_result.stdout}\n{help_result.stderr}"
    )
    assert binary.name in (help_result.stdout + help_result.stderr) or (
        "Usage:" in help_result.stdout or "Usage:" in help_result.stderr
    ), f"{binary.name}: empty or unhelpful usage output"
    invalid = invoke(binary, "--definitely-not-a-real-option-8362")
    assert invalid.returncode != 0, (
        f"{binary.name} accepted an unrecognised command line option"
    )
    if binary.name == "fusion_slam_demo":
        # An explicitly false boolean must never authorize public binding.
        denied = invoke(binary, "--data-dir", "/tmp/irrelevant-for-network-gate",
                        "--host", "0.0.0.0", "--allow-network=false")
        assert denied.returncode != 0
        assert "explicit --allow-network" in denied.stderr, denied.stderr
    print(f"{binary.name}: cxxopts --help and unsupported-option checks passed")


if __name__ == "__main__":
    main()
