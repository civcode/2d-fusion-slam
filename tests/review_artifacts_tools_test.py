#!/usr/bin/env python3
"""Review manifest paths, uploads, immutability and legacy FR079 indexing."""
from __future__ import annotations

import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import review_artifacts as review


class ReviewArtifactsTest(unittest.TestCase):
    def test_docker_path_mapping_and_exclusive_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            file=root/"container"/"fr079.report.json"
            file.parent.mkdir()
            file.write_text("{}")
            manifest=file.parent/"review.json"
            with mock.patch.dict(os.environ, {
                "SLAM_CONTAINER_OUTPUT_DIR": str(file.parent),
                "SLAM_HOST_OUTPUT_DIR": "/host/results",
            }):
                printed=io.StringIO()
                with contextlib.redirect_stdout(printed):
                    inventory=review.review_files(
                        manifest,"fr079_offline",[("report",file,True)])
                self.assertEqual(inventory["format"],"SLAM_REVIEW1")
                self.assertEqual(inventory["files"][0]["host_path"],
                                 "/host/results/fr079.report.json")
                self.assertIn("/host/results/fr079.report.json", printed.getvalue())
            self.assertEqual(json.loads(manifest.read_text()), inventory)
            with self.assertRaises(FileExistsError):
                review.review_files(manifest,"fr079_offline",[("report",file,True)])

    def test_unmapped_container_path_is_never_guessed(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(review.host_path(Path("/output/experiment/foo.json")))
            self.assertEqual(review.host_path(Path("/tmp/experiment/foo.json")),
                             "/tmp/experiment/foo.json")

    def test_refuse_missing_artifact_without_writing_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing=Path(tmp)/"missing.json"
            manifest=Path(tmp)/"review.json"
            with self.assertRaises(FileNotFoundError):
                review.review_files(manifest,"missing",[("report",missing,True)])
            self.assertFalse(manifest.exists())

    def test_cli_indexes_existing_fr079_without_rerun(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            prefix=root/"fr079-continuous-noloop-001"
            for suffix in (".report.json",".trajectory.json",".matches.json",
                           ".slamreplay.json",".slammap",".pgm",".yaml"):
                Path(str(prefix)+suffix).write_text("{}")
            review_file=root/"review-manifest.json"
            with mock.patch.object(sys,"argv",["review_artifacts.py","fr079",
                                               str(prefix),"--manifest",
                                               str(review_file)]):
                with contextlib.redirect_stdout(io.StringIO()):
                    review.cli()
            payload=json.loads(review_file.read_text())
            self.assertEqual([a["role"] for a in payload["files"] if a["upload"]],
                             ["report","trajectory","matching_decisions","replay"])
            with mock.patch.object(sys,"argv",["review_artifacts.py","fr079",
                                               str(prefix),"--manifest",
                                               str(review_file)]):
                with contextlib.redirect_stdout(io.StringIO()):
                    review.cli()
            self.assertTrue(review_file.is_file())


if __name__ == "__main__":
    unittest.main()
