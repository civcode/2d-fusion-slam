"""Run genuine Ceres exported diagnostics and verify integration end-to-end."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.example import create_example_bundle
from web.replay import convert_carmen
from web.ceres_results import validate_results
from web.bundle import read_bundle

if __name__=='__main__':
    exe=Path(sys.argv[1])
    with tempfile.TemporaryDirectory() as root:
        p=Path(root)
        (p/'sample.slammap').write_bytes(create_example_bundle())
        ranges=' '.join(['2.0']*64)
        (p/'raw.log').write_text('FLASER 64 '+ranges+' 6 4.5 0 6 4.5 0 5 host 6\n')
        session=convert_carmen(p/'raw.log',stride=4,frame_id='synthetic_demo_frame')
        (p/'sample.slamreplay.json').write_text(json.dumps(session))
        proc=subprocess.run([str(exe.resolve()),str(p/'sample.slammap'),str(p/'raw.log'),'1','--beam-stride','4'],capture_output=True,text=True,check=True)
        native=json.loads(proc.stdout)
        assert native['format']=='FUSMATCH_NATIVE1' and native['frame_count']==1
        assert len(native['matches'])==1 and native['matches'][0]['input_hits']==16
        from match_ceres import create_results
        result=create_results(p/'sample.slamreplay.json',p/'sample.slammap',p/'raw.log',exe,1,4)
        assert len(result['frames'])==1
        assert result['frames'][0]['map_fingerprint']==read_bundle(p/'sample.slammap').metadata()['fingerprint']
        print('Native Ceres -> FUSMATCH1 -> Python validation PASS')
