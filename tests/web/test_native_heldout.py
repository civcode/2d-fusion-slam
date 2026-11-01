"""C++ Ceres end-to-end held-out regression: rebuild/map fingerprint and index split."""
from pathlib import Path
import gzip
import json
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.replay import convert_carmen
from web.ceres_results import summarize
from match_ceres import create_results


def main():
    if len(sys.argv)!=3:raise SystemExit('usage: test_native_heldout.py MAP_EXPORT CERES_EXPORT')
    map_binary=Path(sys.argv[1]); ceres_binary=Path(sys.argv[2])
    with tempfile.TemporaryDirectory() as td:
        root=Path(td); recording=root/'synthetic.log.gz'
        with gzip.open(recording,'wt') as f:
            for i in range(30):
                # A simple nondegenerate obstacle pattern, not a physical truth dataset.
                ranges=' '.join(str(2.0+.17*(j%7)+.015*(i%3)) for j in range(90))
                x=4+i*.02;y=5+i*.01
                f.write(f'FLASER 90 {ranges} {x} {y} 0 {x} {y} 0 {i*.1} host {i*.1+1}\n')
        prefix=root/'training'
        subprocess.run([str(map_binary),str(recording),str(prefix)],check=True,capture_output=True,timeout=30)
        mapfile=root/'training.slammap'
        replay=root/'record.slamreplay.json'
        replay.write_text(json.dumps(convert_carmen(recording,stride=4,frame_id='fr079_logged_laser_frame_UNVERIFIED')))
        result=create_results(replay,mapfile,recording,ceres_binary,scan_stride=2,beam_stride=4,heldout_only=True)
        # CLI output is a durable report next to the source-linked sidecar.
        cmd=[sys.executable,str(ROOT/'apps'/'demo'/'match_ceres.py'),
             str(recording),str(replay),str(mapfile),
             '--executable',str(ceres_binary),'--scan-stride','2',
             '--beam-stride','4','--verified-fr079-holdout']
        subprocess.run(cmd,capture_output=True,check=True,timeout=30)
        report=json.loads((root/'record.slamreplay.json.holdout-report.json').read_text())
        assert report['format']=='FUSHOLDOUT_REPORT1'
        assert report['diagnostics']['evaluated_scans']==3
        assert report['map_fingerprint']==result['map_fingerprint']
        # Both outputs cannot be replaced unintentionally.
        rerun=subprocess.run(cmd,capture_output=True,timeout=10)
        assert rerun.returncode!=0

        assert result['selection']=='fr079_heldout_mod5_verified'
        assert [r['index'] for r in result['frames']]==[0,10,20]
        assert not result['training_independence_verified']
        assert summarize(result)['evaluated_scans']==3
        # Verify a damaged source map is rejected by native recomputation;
        # this must NOT become an alleged held-out evaluation.
        bogus=root/'wrong.slammap'
        from web.example import create_example_bundle
        bogus.write_bytes(create_example_bundle())
        try:
            create_results(replay,bogus,recording,ceres_binary,scan_stride=2,beam_stride=4,heldout_only=True)
        except Exception:
            pass
        else:raise AssertionError('non-training map must be refused')
    print('native Ceres heldout source-rebuild/index gate passed')

if __name__=='__main__':main()
