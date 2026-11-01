"""Ceres sidecar validation and live HTTP, with immutable source/map fixtures."""
from pathlib import Path
import hashlib
from http.server import ThreadingHTTPServer
import json
import shutil
import sys
import tempfile
import threading
import unittest
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.replay import convert_carmen, ReplayError
from web.server import Dashboard, Handler
from web.example import create_example_bundle
from web.bundle import read_bundle
from web.ceres_results import validate_results


def flaser(t=5.,x=6.,y=4.5):
    ranges=['2.0']*64
    return 'FLASER 64 '+ ' '.join(ranges)+f' {x} {y} 0 {x} {y} 0 {t} host {t+1}\n'

class CeresHTTP(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.recording=self.root/'source.log'
        self.recording.write_text(flaser()+flaser(t=5.1,x=6.1))
        self.session=convert_carmen(self.recording,stride=4,frame_id='synthetic_demo_frame')
        self.replay=self.root/'capture.slamreplay.json'
        self.replay.write_text(json.dumps(self.session))
        self.map=self.root/'synthetic.slammap'
        self.map.write_bytes(create_example_bundle())
        self.meta=read_bundle(self.map).metadata()
        self.meta['name']=self.map.name
        row={"index":0,"t":5.,"seed":[6.,4.5,0.],"estimate":[6.,4.5,0.],
             "status":"no_improvement","iterations":0,"input_hits":16,"supported_hits":13,
             "overlap":13/16,"initial_robust_cost":15.,"robust_cost":15.,
             "mean_distance_m":.2,"full_rank":True,"map_fingerprint":self.meta['fingerprint']}
        self.result={"format":"FUSMATCH1","matcher":"ceres_scan_to_edt",
                     "source_sha256":self.session['source']['source_sha256'],
                     "replay_sha256":hashlib.sha256(self.replay.read_bytes()).hexdigest(),
                     "map_name":self.map.name,"map_fingerprint":self.meta['fingerprint'],
                     "map_frame_id":self.meta['frame_id'],"map_revision":self.meta['revision'],
                     "map_origin_verified":False,"independent_ground_truth":False,
                     "training_independence_verified":False,"per_beam_time_verified":False,
                     "frames":[row]}
        self.sidecar=self.root/(self.replay.name+'.ceres.json')
        self.sidecar.write_text(json.dumps(self.result))
        self.httpd=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.httpd.viewer=Dashboard(self.root)
        self.thread=threading.Thread(target=self.httpd.serve_forever,daemon=True)
        self.thread.start()
        self.base=f'http://127.0.0.1:{self.httpd.server_port}'
    def tearDown(self):
        self.httpd.shutdown();self.httpd.server_close();self.thread.join(timeout=2)
        self.temp.cleanup()
    def get(self,endpoint):
        with urlopen(self.base+endpoint,timeout=5) as res:return json.load(res)
    def test_valid_sidecar_exposed_only_on_sampled_frame(self):
        data=self.get('/api/replays/'+self.replay.name)
        self.assertEqual((data['ceres']['status'],data['ceres']['processed']),("valid",1))
        matched=self.get('/api/replays/'+self.replay.name+'/frame/0')
        self.assertEqual(matched['matching']['status'],'no_improvement')
        self.assertIsNone(self.get('/api/replays/'+self.replay.name+'/frame/1')['matching'])
        self.assertIsNone(self.session['frames'][0]['matching'])
    def test_tampered_source_refused_not_loaded(self):
        self.result['source_sha256']='0'*64
        self.sidecar.write_text(json.dumps(self.result))
        self.assertEqual(self.get('/api/replays/'+self.replay.name)['ceres']['status'],'invalid')
        self.assertIsNone(self.get('/api/replays/'+self.replay.name+'/frame/0')['matching'])
    def test_changed_replay_bytes_invalidates(self):
        self.replay.write_text(json.dumps(self.session,indent=1))
        self.assertEqual(self.get('/api/replays/'+self.replay.name)['ceres']['status'],'invalid')
    def test_changed_frozen_map_refused(self):
        self.map.write_bytes(b'bad')
        self.assertEqual(self.get('/api/replays/'+self.replay.name)['ceres']['status'],'invalid')
    def test_rejected_pose_cannot_masquerade_as_solution(self):
        self.result['frames'][0]['estimate'][0]+=1
        with self.assertRaises(ReplayError):
            validate_results(self.result,self.session,self.result['replay_sha256'],self.meta)
    def test_unsupported_false_accepted_refused(self):
        self.result['frames'][0]['status']='accepted'
        with self.assertRaises(ReplayError):
            validate_results(self.result,self.session,self.result['replay_sha256'],self.meta)
    def test_sidecar_symlink_refused(self):
        self.sidecar.unlink();self.sidecar.symlink_to(self.recording)
        self.assertEqual(self.get('/api/replays/'+self.replay.name)['ceres']['status'],'invalid')
    def test_no_sidecar_retains_original_replay(self):
        self.sidecar.unlink()
        self.assertEqual(self.get('/api/replays/'+self.replay.name)['ceres']['status'],'absent')
        self.assertIsNone(self.get('/api/replays/'+self.replay.name+'/frame/0')['matching'])

if __name__=='__main__':unittest.main()
