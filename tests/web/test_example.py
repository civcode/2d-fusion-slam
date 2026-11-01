"""Synthetic startup demo tests; no hardware or user map modification."""
from pathlib import Path
import tempfile
import unittest

from test_dashboard import make_fixture
from web.bundle import MapBundle
from web.server import Dashboard


class ExampleFallbackTests(unittest.TestCase):
    """Demo has one synthetic example but never masks existing data."""

    def test_example_valid_native_bundle(self):
        from web.example import create_example_bundle, EXAMPLE_NAME
        a = create_example_bundle()
        b = create_example_bundle()
        self.assertEqual(a, b)
        m = MapBundle(a)
        self.assertEqual(EXAMPLE_NAME, 'sample-synthetic-workshop.slammap')
        self.assertEqual((m.width, m.height), (120, 90))
        self.assertGreater(m.known, 5000)
        self.assertGreater(m.unknown, 1000)
        self.assertGreater(m.occupied, 500)
        self.assertEqual(m.truth_class, 'synthetic_exact')
        self.assertFalse(m.metric_origin_verified)

    def test_empty_directory_uses_example_without_writing_to_user_dir(self):
        from web.example import data_dir_or_example, EXAMPLE_NAME
        with tempfile.TemporaryDirectory() as d:
            user = Path(d)
            before = set(user.iterdir())
            with data_dir_or_example(user, use_example=True) as source:
                self.assertNotEqual(source, user)
                self.assertEqual([p.name for p in source.iterdir()], [EXAMPLE_NAME])
                view = Dashboard(source)
                index = view.index()
                self.assertEqual(len(index['maps']), 1)
                self.assertEqual(index['maps'][0]['id'], EXAMPLE_NAME)
                self.assertEqual(index['registry']['status'], 'absent')
            self.assertFalse(source.exists())
            self.assertEqual(set(user.iterdir()), before)

    def test_existing_real_maps_take_precedence(self):
        from web.example import data_dir_or_example
        with tempfile.TemporaryDirectory() as d:
            user = Path(d)
            (user / 'my-real.slammap').write_bytes(make_fixture()[0])
            with data_dir_or_example(user, use_example=True) as selected:
                self.assertEqual(selected, user)
            self.assertEqual([p.name for p in user.iterdir()], ['my-real.slammap'])

    def test_corrupt_registry_does_not_get_hidden_by_example(self):
        from web.example import data_dir_or_example
        with tempfile.TemporaryDirectory() as d:
            user = Path(d)
            (user / 'CURRENT').write_text('broken')
            with data_dir_or_example(user, use_example=True) as selected:
                self.assertEqual(selected, user)
                self.assertEqual(Dashboard(selected).index()['registry']['status'], 'invalid')

    def test_sample_is_not_fallback_without_opt_in(self):
        from web.example import data_dir_or_example
        with tempfile.TemporaryDirectory() as d:
            user = Path(d)
            with data_dir_or_example(user, use_example=False) as selected:
                self.assertEqual(selected, user)
                self.assertFalse(Dashboard(selected).index()['maps'])
