#!/usr/bin/env python3
"""Run native Ceres against an immutable map, attach diagnostics to a replay.

This intentionally runs offline and writes a separate, exclusively created
FUSMATCH1 sidecar. It never edits FUSREPLAY1, FUSMAP2D, or CURRENT.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from web.bundle import read_bundle
from web.replay import read_replay, ReplayError
from web.ceres_results import validate_results, MAX_RESULTS


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def create_results(replay_file: Path, map_file: Path, log_file: Path,
                   executable: Path, scan_stride: int = 20, beam_stride: int = 4,
                   heldout_only: bool = False):
    session=read_replay(replay_file)
    if session['source']['pose_kind'] != 'logged_laser_estimate_not_ground_truth':
        raise ReplayError('Ceres matching requires a real CARMEN session, not synthetic replay')
    if sha(log_file)!=session['source']['source_sha256']:
        raise ReplayError('raw recording SHA-256 differs from replay provenance')
    map_bundle=read_bundle(map_file)
    meta=map_bundle.metadata()
    if meta['frame_id'] != session['source']['frame_id']:
        raise ReplayError('map and replay frame labels differ; independently align them before matching')
    if map_file.name.endswith('.slammap') is False:
        raise ReplayError('source map must be an authoritative .slammap')
    if not (1 <= scan_stride <= 128 and 1 <= beam_stride <= 64):
        raise ReplayError('invalid Ceres query stride')
    args=[str(executable.resolve()),str(map_file.resolve()),str(log_file.resolve()),
          str(scan_stride),'--beam-stride',str(beam_stride)]
    if heldout_only:
        # The native executable recomputes the actual map from the source
        # recording before emitting any held-out diagnostics.
        args.append('--verified-fr079-holdout')
    try:
        native=subprocess.run(args,capture_output=True,timeout=300,check=True)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
        raise ReplayError(f'Ceres export failed: {exc}') from exc
    if len(native.stdout)>MAX_RESULTS:
        raise ReplayError('native match results exceed 4 MiB')
    try:
        decoded=json.loads(native.stdout,parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite')))
    except ValueError as exc:
        raise ReplayError('malformed native Ceres results') from exc
    if decoded.get('format')!='FUSMATCH_NATIVE1' or decoded.get('matcher')!='ceres_scan_to_edt' or \
       decoded.get('frame_count')!=len(session['frames']) or decoded.get('map_fingerprint')!=meta['fingerprint'] or \
       decoded.get('map_frame_id')!=meta['frame_id'] or decoded.get('map_revision')!=meta['revision'] or \
       decoded.get('map_origin_verified')!=meta['metric_origin_verified'] or \
       decoded.get('selection')!=('fr079_heldout_mod5_verified' if heldout_only else 'sampled_unrestricted') or \
       decoded.get('query_stride')!=scan_stride or decoded.get('beam_stride')!=beam_stride:
        raise ReplayError('native Ceres output does not match map/session')
    result={'format':'FUSMATCH1','matcher':'ceres_scan_to_edt',
            'source_sha256':session['source']['source_sha256'],
            'replay_sha256':sha(replay_file),
            'map_name':map_file.name,'map_fingerprint':meta['fingerprint'],
            'map_frame_id':meta['frame_id'],'map_revision':meta['revision'],
            'map_origin_verified':meta['metric_origin_verified'],
            'independent_ground_truth':False,
            'training_independence_verified':False,
            'per_beam_time_verified':False,
            'selection':decoded['selection'],
            'query_stride':decoded['query_stride'],'beam_stride':decoded['beam_stride'],
            'frames':decoded['matches']}
    meta['name']=map_file.name
    validate_results(result,session,sha(replay_file),meta)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path,help='original .log or .log.gz source')
    p.add_argument('replay',type=Path,help='existing .slamreplay.json session')
    p.add_argument('map',type=Path,help='frozen .slammap built separately')
    p.add_argument('--executable',type=Path,default=Path('build/slam_ceres_replay_export'))
    p.add_argument('--scan-stride',type=int,default=20)
    p.add_argument('--beam-stride',type=int,default=4)
    p.add_argument('--report',type=Path,default=None,
                   help='write a separate exclusive FUSHOLDOUT_REPORT1 JSON file (verified-heldout mode only)')
    p.add_argument('--verified-fr079-holdout',action='store_true',
                   help='rebuild and fingerprint-check the train-only map, then match only excluded index%%5==0 scans')
    args=p.parse_args()
    if args.report and not args.verified_fr079_holdout:
        p.error('--report requires --verified-fr079-holdout')
    if args.replay.name.endswith('.slamreplay.json') is False:
        p.error('replay filename must end in .slamreplay.json')
    output=args.replay.with_name(args.replay.name+'.ceres.json')
    report_file=(args.report or args.replay.with_name(args.replay.name+'.holdout-report.json')) if args.verified_fr079_holdout else None
    if output.exists() or (report_file and report_file.exists()):
        p.error('result sidecar or held-out report already exists; refuse overwrite')
    try:
        data=create_results(args.replay,args.map,args.recording,args.executable,
                            args.scan_stride,args.beam_stride,args.verified_fr079_holdout)
        payload=json.dumps(data,separators=(',',':'),allow_nan=False).encode()
        if len(payload)>MAX_RESULTS:
            raise ReplayError('sidecar exceeds 4 MiB')
        report_payload=None
        if report_file:
            from web.ceres_results import summarize
            report={"format":"FUSHOLDOUT_REPORT1",
                    "source_sha256":data['source_sha256'],
                    "replay_sha256":data['replay_sha256'],
                    "map_name":data['map_name'],
                    "map_fingerprint":data['map_fingerprint'],
                    "map_revision":data['map_revision'],
                    "selection":data['selection'],
                    "query_stride":data['query_stride'],
                    "beam_stride":data['beam_stride'],
                    "diagnostics":summarize(data)}
            report_payload=json.dumps(report,indent=2,sort_keys=True,allow_nan=False).encode()+b'\n'
        try:
            with output.open('xb') as f:
                f.write(payload)
            if report_file:
                with report_file.open('xb') as f:
                    f.write(report_payload)
        except OSError:
            # Avoid leaving a misleading half-published result/report pair.
            if report_file and output.exists() and not report_file.exists():
                output.unlink()
            raise
        from collections import Counter
        counts=Counter(r['status'] for r in data['frames'])
        print(f'Ceres results: {output} | {len(data["frames"])} processed scans | {dict(counts)}')
        if args.verified_fr079_holdout:
            from web.ceres_results import summarize
            report=summarize(data)
            print('FR079 held-out split VERIFIED against rebuilt source map:')
            print(json.dumps(report,indent=2,sort_keys=True))
            print(f'Holdout report: {report_file}')
            print('No metric ground truth, validated extrinsics or per-beam timing available.')
        else:
            print('Tracking estimates only. No ground-truth or independent map training claim.')
    except (ReplayError,ValueError,OSError) as exc:
        p.error(str(exc))

if __name__=='__main__':main()
