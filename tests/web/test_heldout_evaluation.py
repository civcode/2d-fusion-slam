"""Held-out FR079 source/map split and match-report validation regressions."""
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.bundle import read_bundle
from web.ceres_results import validate_results, summarize
from web.replay import convert_carmen, ReplayError
from match_ceres import create_results


class HoldoutValidation(unittest.TestCase):
    def setUp(self):
        from web.example import create_example_bundle
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        self.log=self.root/'source.log.gz'
        with gzip.open(self.log,'wt') as f:
            for i in range(30):
                # Almost identical beams; only structure/provenance is tested.
                ranges=' '.join(['2.1']*64)
                f.write(f'FLASER 64 {ranges} {3+i*.01} 4 0 {3+i*.01} 4 0 {i*.1} host {i*.1+1}\n')
        self.replay=self.root/'record.slamreplay.json'
        self.replay.write_text(json.dumps(convert_carmen(self.log,stride=4,frame_id='fr079_logged_laser_frame_UNVERIFIED')))
        self.map=self.root/'test.slammap'
        self.map.write_bytes(create_example_bundle())
    def tearDown(self):self.tmp.cleanup()
    def test_legacy_sidecar_remains_supported(self):
        from web.ceres_results import validate_results
        session=json.loads(self.replay.read_text())
        metadata=read_bundle(self.map).metadata();metadata['name']=self.map.name
        metadata['frame_id']=session['source']['frame_id']
        row={'index':0,'t':0.,'seed':[3.,4.,0.],'estimate':[3.,4.,0.],
             'status':'no_improvement','iterations':0,'input_hits':16,'supported_hits':0,
             'overlap':0.,'initial_robust_cost':0.,'robust_cost':0.,
             'mean_distance_m':None,'full_rank':False,'map_fingerprint':metadata['fingerprint']}
        r={'format':'FUSMATCH1','matcher':'ceres_scan_to_edt',
           'source_sha256':session['source']['source_sha256'],
           'replay_sha256':hashlib.sha256(self.replay.read_bytes()).hexdigest(),
           'map_name':self.map.name,'map_fingerprint':metadata['fingerprint'],
           'map_frame_id':metadata['frame_id'],'map_revision':metadata['revision'],
           'map_origin_verified':False,'independent_ground_truth':False,
           'training_independence_verified':False,'per_beam_time_verified':False,
           'frames':[row]}
        validate_results(r,session,r['replay_sha256'],metadata)
        self.assertFalse(summarize(r)['training_split_checked'])
        r['selection']='fr079_heldout_mod5_verified'
        r['query_stride']=10;r['beam_stride']=4
        with self.assertRaisesRegex(ReplayError,'non-training map'):
            validate_results(r,session,r['replay_sha256'],metadata)
    def test_holdout_only_selects_excluded_indices(self):
        # Full native end-to-end test runs separately via CTest with proper
        # build and training map; this unit rejects untrusted sidecar claims.
        s=json.loads(self.replay.read_text())
        m=read_bundle(self.map).metadata();m['name']=self.map.name
        m['source_id']='fr079-training-index-mod5-mod6-v1'
        m['frame_id']='fr079_logged_laser_frame_UNVERIFIED'
        m['metric_origin_verified']=False
        seed=s['frames'][1]['pose']
        r={'format':'FUSMATCH1','matcher':'ceres_scan_to_edt',
           'source_sha256':s['source']['source_sha256'],
           'replay_sha256':hashlib.sha256(self.replay.read_bytes()).hexdigest(),
           'map_name':m['name'],'map_fingerprint':m['fingerprint'],
           'map_frame_id':m['frame_id'],'map_revision':m['revision'],
           'map_origin_verified':False,'independent_ground_truth':False,
           'training_independence_verified':False,'per_beam_time_verified':False,
           'selection':'fr079_heldout_mod5_verified','query_stride':10,'beam_stride':4,
           'frames':[{'index':1,'t':s['frames'][1]['t'],'seed':seed,'estimate':seed[:],
             'status':'no_improvement','iterations':0,'input_hits':16,'supported_hits':0,
             'overlap':0.,'initial_robust_cost':0.,'robust_cost':0.,
             'mean_distance_m':None,'full_rank':False,'map_fingerprint':m['fingerprint']}]}
        with self.assertRaisesRegex(ReplayError,'held-out coverage'):
            validate_results(r,s,r['replay_sha256'],m)
        r['frames'][0]['index']=0
        r['frames'][0]['t']=s['frames'][0]['t']
        r['frames'][0]['seed']=s['frames'][0]['pose']
        r['frames'][0]['estimate']=s['frames'][0]['pose'][:]
        validate_results(r,s,r['replay_sha256'],m)
        summ=summarize(r)
        self.assertTrue(summ['training_split_checked'])
        self.assertFalse(summ['ground_truth_available'])
        self.assertEqual(summ['evaluated_scans'],1)
        self.assertEqual(summ['status_counts']['no_improvement'],1)

if __name__=='__main__':unittest.main()
