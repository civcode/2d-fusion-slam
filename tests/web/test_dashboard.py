"""Hermetic dashboard tests: native map format, registry, HTTP and security."""
from __future__ import annotations

import http.server
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import unittest
from urllib import error, request
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "apps" / "demo"))
from web.bundle import (FNV_START, BundleError, MapBundle, fnv_update, fnv_word,
                        parse_registry, read_bundle)
from web.server import Dashboard, Handler


def make_fixture(width=6, height=5, revision=8):
    cells = []
    for y in range(height):
        for x in range(width):
            obs = 1 if x < 3 or (x == 3 and y == height - 1) else 0
            lo = 1.7 if x == 1 else (-1.2 if obs else 0.0)
            cells.append((obs, lo))
    config = (0.1, 2.0, -1.0, 0.3, 0.85, -0.4, -5.0, 5.0)
    h = FNV_START
    for n in (width, height):
        h = fnv_word(h, n)
    for n in config[:4]:
        h = fnv_word(h, struct.unpack("<Q", struct.pack("<d", n))[0])
    h = fnv_word(h, revision)
    for obs, lo in cells:
        h = fnv_word(h, struct.unpack("<Q", struct.pack("<d", lo))[0])
        h = fnv_word(h, obs)
    data = bytearray(b"FUSMAP2D")
    data.extend(struct.pack("<4I", 1, 0, width, height))
    data.extend(struct.pack("<8d", *config))
    data.extend(struct.pack("<IQQ", 0, revision, h))  # config flag + revision + fingerprint
    for label in (b"synthetic_test", b"independent_fixture", b"synthetic_exact"):
        data.extend(struct.pack("<I", len(label)) + label)
    data.extend(struct.pack("<Q", len(cells)))
    for obs, lo in cells:
        data.extend(struct.pack("<Bd", obs, lo))
    data.extend(struct.pack("<Q", fnv_update(FNV_START, data)))
    return bytes(data), str(h)


def registry_text(active=None, previous=None):
    def line(v):
        return "0 0\n" if v is None else f"{v[0]} {v[1]}\n"
    payload = "FUSGEN1\n" + line(active) + line(previous)
    return (payload + str(fnv_update(FNV_START, payload.encode())) + "\n").encode()


class MapTests(unittest.TestCase):
    def test_canonical_binary_and_cell_coordinates(self):
        data, fp = make_fixture()
        m = MapBundle(data)
        self.assertEqual((m.width, m.height, m.known, m.unknown), (6, 5, 16, 14))
        self.assertEqual(str(m.fingerprint), fp)
        cell = m.cell(1, 0)
        self.assertTrue(cell["observed"])
        self.assertAlmostEqual(cell["occupied_probability"], 1 / (1 + math.exp(-1.7)))
        x, y = (1.5 * .1, .5 * .1)
        self.assertAlmostEqual(cell["world_x"], 2 + math.cos(.3)*x - math.sin(.3)*y)
        self.assertFalse(m.cell(5, 3)["observed"])
        self.assertIsNone(m.cell(5, 3)["occupied_probability"])

    def test_png_layout_valid_crc_and_top_to_bottom(self):
        m = MapBundle(make_fixture()[0]); png = m.png()
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
        pos=8; compressed=b""; w=h=0
        while pos < len(png):
            n = struct.unpack_from(">I", png, pos)[0]
            tag = png[pos+4:pos+8]
            data = png[pos+8:pos+8+n]
            expected = struct.unpack_from(">I", png, pos+8+n)[0]
            self.assertEqual(expected,zlib.crc32(tag+data))
            if tag==b"IHDR":w,h=struct.unpack_from(">II", data)
            if tag==b"IDAT":compressed+=data
            pos+=n+12
        self.assertEqual((w,h),(6,5))
        raw=zlib.decompress(compressed)
        self.assertEqual(len(raw),h*(1+w*4))
        self.assertEqual(raw[0],0)  # filter None
        self.assertEqual(raw[1+1*4:1+2*4], bytes((70,211,184,255)))  # top y occupied
        self.assertEqual(raw[1+5*4:1+6*4], bytes((21,34,54,255)))  # top y unknown

    def test_corrupt_payload_rejected(self):
        data, _ = make_fixture()
        variants = [data[:-1], b"X"+data[1:], data+b"X",
                    data[:-8]+b"\x00"*8,
                    data[:-20]+b"\x80"+data[-19:]]
        for payload in variants:
            with self.subTest(payload=len(payload)):
                with self.assertRaises(BundleError):MapBundle(payload)

    def test_modified_cells_even_with_recomputed_footer_rejected(self):
        raw = bytearray(make_fixture()[0]); raw[-18] ^= 1
        raw[-8:] = struct.pack("<Q", fnv_update(FNV_START, raw[:-8]))
        with self.assertRaisesRegex(BundleError, "fingerprint|occupancy"):
            MapBundle(bytes(raw))

    def test_registry_good_bad_pointer(self):
        data, fp = make_fixture()
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"CURRENT"
            self.assertEqual(parse_registry(p)["status"],"absent")
            p.write_bytes(registry_text((2,fp),(1,44)))
            self.assertEqual(parse_registry(p)["active"]["file"],"generation-2.slammap")
            p.write_bytes(registry_text((2,fp),(2,44)))
            with self.assertRaises(BundleError):parse_registry(p)
            p.write_bytes(registry_text((2,fp))[:-3])
            with self.assertRaises(BundleError):parse_registry(p)


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.root=Path(cls.tmp.name)
        cls.data,cls.fp=make_fixture()
        (cls.root/"generation-1.slammap").write_bytes(cls.data)
        (cls.root/"CURRENT").write_bytes(registry_text((1,cls.fp)))
        cls.httpd=http.server.ThreadingHTTPServer(("127.0.0.1",0),Handler)
        cls.httpd.viewer=Dashboard(cls.root)
        cls.thread=threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        cls.url=f"http://127.0.0.1:{cls.httpd.server_port}"

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown();cls.httpd.server_close();cls.thread.join(timeout=2)
        cls.tmp.cleanup()

    def fetch(self,path, method="GET"):
        return request.urlopen(request.Request(self.url+path,method=method),timeout=5)

    def test_live_http_map_registry_and_cell(self):
        with self.fetch("/api/maps") as r:
            index=json.load(r)
            self.assertEqual(r.headers["Content-Security-Policy"].count("object-src 'none'"),1)
            self.assertEqual(index["registry"]["status"],"present")
            self.assertEqual(index["registry"]["active"]["integrity"],"valid")
            self.assertEqual(index["maps"][0]["role"],"active")
            self.assertFalse(index["live_tracking"])
        with self.fetch("/api/maps/generation-1.slammap") as r:
            m=json.load(r);self.assertEqual(m["fingerprint"],self.fp)
            self.assertFalse(m["metric_origin_verified"])
        with self.fetch("/api/maps/generation-1.slammap/cell?x=1&y=4") as r:
            cell=json.load(r);self.assertTrue(cell["observed"])
        with self.fetch("/api/maps/generation-1.slammap/image.png") as r:
            self.assertEqual(r.headers["Content-Type"],"image/png")
            self.assertTrue(r.read().startswith(b"\x89PNG"))

    def test_page_scripts_served(self):
        for endpoint, expected in [("/","text/html"),("/app.js","text/javascript"),
                                   ("/styles.css","text/css")]:
            with self.fetch(endpoint) as r:
                self.assertIn(expected,r.headers["Content-Type"])
                self.assertGreater(len(r.read()),100)

    def test_refuses_state_changes(self):
        for method in ("POST","DELETE","PUT"):
            with self.assertRaises(error.HTTPError) as cm:
                self.fetch("/api/maps",method=method)
            self.assertEqual(cm.exception.code,405)

    def test_refuses_invalid_paths_and_indices(self):
        for url in ("/api/maps/../CURRENT","/api/maps/%2E%2E/CURRENT",
                    "/api/maps/generation-1.slammap/cell?x=-1&y=1",
                    "/api/maps/generation-1.slammap/cell?x=1&y=600",
                    "/api/maps/generation-1.slammap/cell?x=NaN&y=1",
                    "/api/maps/generation-1.slammap/cell?x=1&x=2&y=1"):
            with self.subTest(url=url), self.assertRaises(error.HTTPError) as cm:
                self.fetch(url)
            self.assertIn(cm.exception.code,(404,422))

    def test_symlinks_not_listed(self):
        (self.root/"outside.slammap").symlink_to("/etc/passwd")
        try:
            with self.fetch("/api/maps") as r: self.assertNotIn("outside.slammap",json.dumps(json.load(r)))
            with self.assertRaises(error.HTTPError) as cm:self.fetch("/api/maps/outside.slammap")
            self.assertEqual(cm.exception.code,422)
        finally:(self.root/"outside.slammap").unlink()

    def test_corrupt_active_file_reported_without_recovery(self):
        path=self.root/"generation-1.slammap"
        original=path.read_bytes()
        try:
            path.write_bytes(b"corrupt")
            with self.fetch("/api/maps") as r:
                index=json.load(r)
                self.assertEqual(index["registry"]["status"],"degraded")
            with self.assertRaises(error.HTTPError) as cm:self.fetch("/api/maps/generation-1.slammap")
            self.assertEqual(cm.exception.code,422)
            self.assertEqual((self.root/"CURRENT").read_bytes(),registry_text((1,self.fp)))
        finally:
            path.write_bytes(original)


if __name__=="__main__":unittest.main()
