"""Synthetic fallback replay does not affect any existing user evidence."""
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.example import data_dir_or_example
from web.replay import create_synthetic_replay
from web.server import Dashboard

class SyntheticReplayTests(unittest.TestCase):
    def test_synthetic_session_and_timeline(self):
        data=create_synthetic_replay()
        self.assertEqual(data['format'],'FUSREPLAY1')
        self.assertEqual(len(data['frames']),56)
        self.assertEqual(len(data['frames'][0]['points']),64)
        self.assertEqual(data['source']['pose_kind'],'synthetic_exact')
        self.assertFalse(data['source']['map_alignment_verified'])

    def test_example_mode_serves_virtual_session_without_user_file(self):
        with tempfile.TemporaryDirectory() as root:
            with data_dir_or_example(Path(root),True) as generated:
                self.assertNotEqual(generated,Path(root))
                viewer=Dashboard(generated)
                viewer.synthetic_replay=create_synthetic_replay()
                self.assertEqual(len(viewer.replays()['sessions']),1)
                self.assertEqual(len(viewer.replay(viewer.replays()['sessions'][0])['frames']),56)
            self.assertEqual(list(Path(root).iterdir()),[])

    def test_existing_recordings_never_hidden_by_synthetic_map(self):
        with tempfile.TemporaryDirectory() as root:
            (Path(root)/'recording.slamreplay.json').write_text('corrupt')
            with data_dir_or_example(Path(root),True) as selected:
                self.assertEqual(selected,Path(root))
                viewer=Dashboard(selected)
                self.assertEqual(viewer.replays()['sessions'],['recording.slamreplay.json'])

if __name__=='__main__':unittest.main()
