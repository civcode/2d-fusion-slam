"""Real HTTP and strict converter regression for recorded LiDAR replay."""
import gzip
import hashlib
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib import error, request

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.replay import ReplayError, convert_carmen, parse_flaser, read_replay, validate
from web.server import Dashboard, Handler


def carmen_line(*, x=2.5, t=20.0, bad=None):
    ranges=[1.0, 2.0, 0.0, 80.99, 120.0, 3.5]
    if bad is not None: ranges[bad] = float('nan')
    return ('FLASER 6 '+' '.join(map(str,ranges))+f' {x} -1 0.3 {x} -1 0.3 {t} magnum 8.5\n')


class ConversionTests(unittest.TestCase):
    def test_angles_match_carmen_and_invalid_beams_filtered(self):
        result=parse_flaser(carmen_line(),stride=1)
        self.assertEqual((result['beam_count'],result['hit_count'],len(result['points'])),(6,4,4))
        self.assertAlmostEqual(result['points'][0][0],0,places=4)
        self.assertAlmostEqual(result['points'][0][1],-1.0,places=4)
        self.assertAlmostEqual(result['points'][1][0],1.0,places=4)
        self.assertAlmostEqual(result['points'][1][1],-1.73205,places=4)
        self.assertIsNone(result['matching'])

    def test_convert_realistic_small_log(self):
        with tempfile.TemporaryDirectory() as d:
            log=Path(d)/'dataset.log.gz'
            with gzip.open(log,'wt') as out:
                out.write('PARAM something ignored\n'+carmen_line(t=20)+carmen_line(x=2.6,t=20.2))
            data=convert_carmen(log,stride=2,frame_id='test_frame_UNVERIFIED')
            self.assertEqual(len(data['frames']),2)
            self.assertEqual(data['frames'][1]['index'],1)
            self.assertEqual(data['source']['source_sha256'],hashlib.sha256(log.read_bytes()).hexdigest())
            self.assertEqual(data['source']['frame_id'],'test_frame_UNVERIFIED')
            self.assertFalse(data['source']['per_beam_time_known'])
            path=Path(d)/'session.slamreplay.json'
            path.write_text(json.dumps(data))
            self.assertEqual(read_replay(path),data)

    def test_fail_closed_nonfinite_reordered_and_malformed(self):
        with self.assertRaises(ReplayError):parse_flaser(carmen_line(bad=2),stride=1)
        with self.assertRaises(ReplayError):parse_flaser('FLASER 500000 '+carmen_line(),stride=1)
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'raw.log';src.write_text(carmen_line(t=20.1)+carmen_line(t=20))
            with self.assertRaisesRegex(ReplayError,'nonmonotonic'):convert_carmen(src)
        with self.assertRaises(ReplayError):validate({'format':'FUSREPLAY1','source':{},'frames':[]})

    def test_corrupt_replay_refused(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'raw.log';src.write_text(carmen_line())
            data=convert_carmen(src)
            data['frames'][0]['points'][0][0]=float('nan')
            with self.assertRaises(ReplayError):validate(data)
            file=Path(d)/'tampered.slamreplay.json'
            file.write_text('{"format":"unsupported"}')
            with self.assertRaises(ReplayError):read_replay(file)
            file.unlink();file.symlink_to(src)
            with self.assertRaises(ReplayError):read_replay(file)


class ReplayHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.root=Path(cls.tmp.name)
        source=cls.root/'raw.log';source.write_text(carmen_line(t=12.0)+carmen_line(x=2.6,t=12.2))
        cls.file=cls.root/'recording.slamreplay.json'
        cls.file.write_text(json.dumps(convert_carmen(source,stride=1)))
        cls.httpd=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        cls.httpd.viewer=Dashboard(cls.root)
        cls.thread=threading.Thread(target=cls.httpd.serve_forever,daemon=True)
        cls.thread.start()
        cls.url=f'http://127.0.0.1:{cls.httpd.server_port}'
    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown();cls.httpd.server_close();cls.thread.join(timeout=2);cls.tmp.cleanup()
    def api(self,endpoint):
        with request.urlopen(self.url+endpoint,timeout=5) as r:return json.load(r)
    def test_session_index_meta_and_frame(self):
        self.assertEqual(self.api('/api/replays')['sessions'],['recording.slamreplay.json'])
        meta=self.api('/api/replays/recording.slamreplay.json')
        self.assertEqual(meta['frame_count'],2)
        self.assertEqual(meta['trajectory'][1][1],2.6)
        self.assertFalse(meta['source']['map_alignment_verified'])
        batch=self.api('/api/replays/recording.slamreplay.json/frames')
        self.assertEqual(batch['format'], 'FUSREPLAYFRAMES1')
        self.assertEqual(batch['source_sha256'],meta['source']['source_sha256'])
        self.assertEqual(len(batch['frames']),meta['frame_count'])
        frame=self.api('/api/replays/recording.slamreplay.json/frame/1')
        self.assertEqual(batch['frames'][1],frame)
        self.assertEqual(frame['index'],1)
        self.assertEqual(frame['hit_count'],4)
        self.assertIsNone(frame['matching'])
    def test_replay_html_and_js(self):
        for path,tag in [('/replay',b'LiDAR replay'),('/replay.js',b'"use strict"'),('/replay.css',b'.replay-shell')]:
            with request.urlopen(self.url+path,timeout=5) as r:
                self.assertIn(tag,r.read());self.assertIn('object-src',r.headers['Content-Security-Policy'])
    def test_invalid_frames_symlinks_write_and_corrupt(self):
        for url in ['/api/replays/recording.slamreplay.json/frame/12',
                    '/api/replays/../raw.log','/api/replays/recording.slamreplay.json/frame/-1',
                    '/api/replays/recording.slamreplay.json/frame/0?x=0']:
            try:
                request.urlopen(self.url+url,timeout=5)
                if '?x=0' not in url:self.fail(f'invalid path accepted: {url}')
            except error.HTTPError as exc:
                self.assertIn(exc.code,(404,422))
        with self.assertRaises(error.HTTPError) as cm:
            request.urlopen(request.Request(self.url+'/api/replays',method='POST'))
        self.assertEqual(cm.exception.code,405)
        link=self.root/'outside.slamreplay.json';link.symlink_to('/etc/passwd')
        try:
            self.assertNotIn(link.name,self.api('/api/replays')['sessions'])
            with self.assertRaises(error.HTTPError) as cm:request.urlopen(self.url+'/api/replays/'+link.name)
            self.assertEqual(cm.exception.code,422)
        finally:link.unlink()
        original=self.file.read_bytes()
        try:
            self.file.write_bytes(b'corrupt')
            with self.assertRaises(error.HTTPError) as cm:request.urlopen(self.url+'/api/replays/recording.slamreplay.json')
            self.assertEqual(cm.exception.code,422)
        finally:self.file.write_bytes(original)


if __name__=='__main__':unittest.main()
