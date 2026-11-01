#!/usr/bin/env python3
"""Launch the local FUSMAP2D dashboard from a repository checkout."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server import run

if __name__ == "__main__":
    run()
