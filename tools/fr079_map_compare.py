#!/usr/bin/env python3
"""FR079 map-to-map agreement (NOT independent ground-truth accuracy).

Input: original supplied fr079-maps.tgz and a FUSMAP2D .slammap.
Output: JSON scores, a labelled overlay PNG, and an SE(2) alignment estimate.

Requires numpy, scipy, matplotlib; reads and validates FUSMAP2D via the
repository's existing apps/demo/web/bundle.py implementation.

No generated map is used as a reference during SLAM; registration and
scoring are strictly after the recording has been processed.
"""
from __future__ import annotations

from review_artifacts import review_files

import argparse
import gzip
import json
import math
from pathlib import Path
import struct
import sys
import tarfile

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from scipy.ndimage import map_coordinates


def reference_grid(archive: Path, resolution_cm: int):
    name = f"fr079-maps/fr079-{resolution_cm:02d}cm.carmen.map"
    if archive.suffix == ".tgz" or archive.name.endswith(".tar.gz"):
        with tarfile.open(archive, "r:gz") as tar:
            member = tar.getmember(name)
            if not member.isfile() or member.size > 5_000_000:
                raise ValueError("unexpected CARMEN reference map archive member")
            stream = tar.extractfile(member)
            if stream is None:
                raise ValueError("missing reference map file")
            compressed = stream.read(5_000_001)
    else:
        compressed = archive.read_bytes()
    raw = gzip.decompress(compressed)
    magic, grid = b"CARMENMAPFILEv020 ", b"GRIDMAP   "
    if magic not in raw or grid not in raw:
        raise ValueError("unsupported CARMEN reference map format")
    pos = raw.index(grid) + len(grid)
    width, height, step = struct.unpack_from("<IIf", raw, pos)
    if not (0 < width * height <= 4_000_000 and 0.01 <= step <= .2):
        raise ValueError("unreasonable reference map bounds")
    data = np.frombuffer(raw, dtype="<f4", count=width * height, offset=pos + 12)
    # CARMEN is X-major, with Y increasing UP. No image-space Y flip here.
    probabilities = data.reshape(width, height).T.copy()
    known = (probabilities >= 0) & (probabilities <= 1)
    if not np.all((probabilities == -1) | known):
        raise ValueError("unsupported occupancy values")
    return known, known & (probabilities > .65), float(step), (width, height)


def generated_grid(slammap: Path, repository_root: Path):
    sys.path.insert(0, str(repository_root / "apps" / "demo"))
    from web.bundle import read_bundle  # import at runtime, not module load
    bundle = read_bundle(slammap)  # verifies binary footer AND map fingerprint
    dtype = np.dtype([("observed", "u1"), ("logodds", "<f8")])
    cells = np.frombuffer(bundle.cells, dtype=dtype).reshape(bundle.height, bundle.width)
    known = cells["observed"] != 0
    occupied = known & (cells["logodds"] > math.log(.65 / .35))
    return known, occupied, bundle


def wall_points(occupied: np.ndarray, spacing: float, origin=(0., 0., 0.)):
    ys, xs = np.nonzero(occupied)
    pts = np.column_stack(((xs+.5) * spacing, (ys+.5) * spacing))
    ox, oy, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + [ox, oy]


def sample_points(points: np.ndarray, maximum: int, seed: int):
    if len(points) <= maximum:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), maximum, replace=False)]


def transform_points(points: np.ndarray, param: np.ndarray, mirror: bool = False):
    angle, dx, dy = param
    pts = points
    if mirror:
        pts = points * [1., -1.]
    c, s = np.cos(angle), np.sin(angle)
    return pts @ np.array([[c, s], [-s, c]]) + [dx, dy]


def pca_yaw(pts: np.ndarray):
    cov = np.cov(pts.T)
    vals, vecs = np.linalg.eigh(cov)
    vector = vecs[:, int(np.argmax(vals))]
    return math.atan2(float(vector[1]), float(vector[0]))


def rigid_registration(reference_pts, generated_pts, *, mirror=False):
    # Wall shape alignment is a diagnostic only. No scale fitting, no warping,
    # and reflection is deliberately excluded unless explicitly diagnostic.
    r = sample_points(reference_pts, 8000, 2003)
    g = sample_points(generated_pts, 4000, 2004)
    reference_tree = cKDTree(r)
    refcenter = np.median(r, axis=0)
    genbase = g * [1., -1.] if mirror else g
    gencenter = np.median(genbase, axis=0)
    base = pca_yaw(r) - pca_yaw(genbase)
    solutions = []

    def score(param):
        projected = transform_points(g, param, mirror)
        d = reference_tree.query(projected, k=1, workers=1)[0]
        # Robust truncated error resists outliers without allowing an optimizer
        # to 'erase' missing walls. The UNTRIMMED metrics are computed below.
        return float(np.mean(np.minimum(d, 1.0)**2))

    for k in range(4):
        yaw = base + k * math.pi/2
        c, s = math.cos(yaw), math.sin(yaw)
        center = gencenter @ np.array([[c, s], [-s, c]])
        guess = np.array([yaw, *(refcenter-center)])
        fit = minimize(score, guess, method="Powell", options={
            "maxiter": 70, "xtol": .005, "ftol": .0002})
        solutions.append((float(score(fit.x)), fit.x))
    solutions.sort(key=lambda x:x[0])
    error, params = solutions[0]
    params[0] = (params[0]+math.pi) % (2*math.pi)-math.pi
    return params, error, [float(x[0]) for x in solutions]


def near_wall_metrics(ref_pts, gen_pts, alignment, *, mirror=False):
    src=transform_points(gen_pts, alignment, mirror)
    # Operate on both full wall sets rather than the optimizer's subsets.
    d_pred = cKDTree(ref_pts).query(src, k=1)[0]
    d_ref = cKDTree(src).query(ref_pts, k=1)[0]
    def metrics(d):
        return {"mean_m":float(np.mean(d)), "median_m":float(np.median(d)),
                "p95_m":float(np.percentile(d,95)),
                "within_0p15m_fraction":float(np.mean(d<=.15)),
                "within_0p30m_fraction":float(np.mean(d<=.30))}
    p15=float(np.mean(d_pred <= .15));r15=float(np.mean(d_ref <= .15))
    p30=float(np.mean(d_pred <= .30));r30=float(np.mean(d_ref <= .30))
    return {"generated_to_reference":metrics(d_pred),
            "reference_to_generated":metrics(d_ref),
            "symmetric_mean_m":float((np.mean(d_pred)+np.mean(d_ref))/2),
            "symmetric_p95_m":float(np.percentile(
                np.concatenate((d_pred,d_ref)),95)),
            "wall_precision_within_0p15m":p15,
            "wall_recall_within_0p15m":r15,
            "wall_f1_within_0p15m":float(2*p15*r15/(p15+r15)) if p15+r15 else 0.,
            "wall_precision_within_0p30m":p30,
            "wall_recall_within_0p30m":r30,
            "wall_f1_within_0p30m":float(2*p30*r30/(p30+r30)) if p30+r30 else 0.}


def reference_resample(known_pred, occ_pred, bundle, refshape, resolution, alignment,
                       *, mirror=False):
    # Evaluate each reference grid cell by a nearest-neighbor lookup in the
    # reconstructed map; unknown remains UNKNOWN, not free space.
    height,width=refshape
    yy,xx=np.indices((height,width),dtype=np.float64)
    tx=np.array([(xx+.5)*resolution,(yy+.5)*resolution])
    angle,dx,dy=alignment
    c,s=np.cos(angle),np.sin(angle)
    world=tx.reshape(2,-1).T-np.array([dx,dy])
    # inverse R(angle)
    world=world @ np.array([[c,-s],[s,c]])
    if mirror: world[:,1]*=-1
    ox,oy,yaw=bundle.origin
    world-=np.array([ox,oy])
    a,b=np.cos(yaw),np.sin(yaw)
    local=world @ np.array([[a,-b],[b,a]])
    gx=local[:,0]/bundle.resolution-.5
    gy=local[:,1]/bundle.resolution-.5
    inside=(gx>=-.5)&(gy>=-.5)&(gx<bundle.width-.5)&(gy<bundle.height-.5)
    ix=np.rint(gx).astype(np.int64);iy=np.rint(gy).astype(np.int64)
    ix=np.clip(ix,0,bundle.width-1);iy=np.clip(iy,0,bundle.height-1)
    known=(inside & known_pred[iy,ix]).reshape((height,width))
    occupied=(known.ravel() & occ_pred[iy,ix]).reshape((height,width))
    return known,occupied


def occupancy_scores(ref_known,ref_occ,gen_known,gen_occ):
    joint=ref_known & gen_known
    tp=np.sum(ref_occ & gen_occ & joint)
    union=np.sum((ref_occ|gen_occ)&joint)
    # A conservative diagnostic counts reference walls that are not covered
    # as misses. It is not the same as wall IoU over independently surveyed GT.
    coverage=np.sum(joint)/max(1,np.sum(ref_known))
    rwall_recall=np.sum(ref_occ & gen_occ)/max(1,np.sum(ref_occ))
    return {"overlap_known_fraction_of_reference_known":float(coverage),
            "occupied_iou_on_joint_known_cells":float(tp/union) if union else None,
            "reference_occupied_recall_including_unknown_as_missed":float(rwall_recall),
            "joint_known_cells":int(joint.sum()),
            "reference_known_cells":int(ref_known.sum()),
            "reference_wall_cells":int(ref_occ.sum()),
            "generated_known_on_reference_grid":int(gen_known.sum()),
            "generated_wall_on_reference_grid":int(gen_occ.sum())}


def save_plot(filename, refknown, refocc, genknown, genocc, refpts, genpts,
              alignment, scores, *, mirror, reference_resolution_m):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(15,6.3))
    view=np.full(refknown.shape,.5,dtype=float)
    view[refknown]=1.
    view[refocc]=0.
    ax[0].imshow(view,origin="lower",cmap="gray",vmin=0,vmax=1)
    ax[0].set_title("FR079 supplied scan-matched reference (not independent truth)")
    ax[0].set_xlabel(f"{100 * reference_resolution_m:g} cm grid columns")
    ax[0].set_ylabel(f"{100 * reference_resolution_m:g} cm grid rows (Y up)")
    refsample=sample_points(refpts,30000,12)
    gensample=sample_points(genpts,30000,13)
    transformed=transform_points(gensample,alignment,mirror)
    ax[1].scatter(refsample[:,0],refsample[:,1],s=.2,c="#192a62",alpha=.5,
                  label="Supplied reference walls")
    ax[1].scatter(transformed[:,0],transformed[:,1],s=.3,c="#dd4e31",alpha=.4,
                  label="Our map, rigidly aligned")
    ax[1].axis("equal");ax[1].legend(markerscale=10)
    ax[1].set_xlabel("Reference-map local X (m)")
    ax[1].set_ylabel("Reference-map local Y (m)")
    ax[1].set_title("SE(2) aligned wall overlap" +
                    (" — MIRRORED diagnostic only" if mirror else ""))
    fig.suptitle("Map-to-map geometric agreement (in-sample registration, not ATE/GT)")
    fig.tight_layout()
    fig.savefig(filename,dpi=145)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slammap",type=Path,required=True)
    parser.add_argument("--reference-archive",type=Path,required=True)
    parser.add_argument("--repo",type=Path,default=Path.cwd(),
                        help="repository root containing apps/demo/web/bundle.py")
    parser.add_argument("--reference-cm",type=int,choices=(3,5,10),default=5)
    parser.add_argument("--out",type=Path,default=Path("fr079-comparison"),
                        help="new output basename; .json/.png must not exist")
    parser.add_argument("--diagnostic-reflection",action="store_true",
                        help="try an improper mirrored mapping; do NOT treat it as robot pose")
    args=parser.parse_args()
    if args.out.with_suffix(".json").exists() or args.out.with_suffix(".png").exists():
        parser.error("refusing to overwrite existing comparison outputs")
    refknown,refocc,res,shape=reference_grid(args.reference_archive,args.reference_cm)
    genknown,genocc,bundle=generated_grid(args.slammap,args.repo)
    refpts=wall_points(refocc,res)
    genpts=wall_points(genocc,bundle.resolution,bundle.origin)
    if min(len(refpts),len(genpts)) < 100:
        raise ValueError("too few occupied walls for a meaningful alignment")
    fit, fitcost, alternatives = rigid_registration(refpts,genpts,
                                                     mirror=args.diagnostic_reflection)
    detail=near_wall_metrics(refpts,genpts,fit,mirror=args.diagnostic_reflection)
    sampled_known,sampled_occ=reference_resample(genknown,genocc,bundle,
        refknown.shape,res,fit,mirror=args.diagnostic_reflection)
    occupancy=occupancy_scores(refknown,refocc,sampled_known,sampled_occ)
    report={
        "type":"fr079_map_to_map_agreement_proxy_not_ground_truth",
        "reference":"supplied FR079 scan-matched reference (same recording)",
        "independent_ground_truth":False,
        "registration":"in_sample_SE2_rigid_wall_alignment_not_trajectory_calibration",
        "reflection_used":bool(args.diagnostic_reflection),
        "proper_robot_pose_transform":not args.diagnostic_reflection,
        "grid_origin_independently_verified":False,
        "reference_resolution_m":res,"generated_resolution_m":bundle.resolution,
        "reference_size_cells":list(shape),
        "generated_size_cells":[bundle.width,bundle.height],
        "generated_provenance":{"frame_id":bundle.frame_id,
                                "truth_class":bundle.truth_class,
                                "fingerprint":str(bundle.fingerprint)},
        "alignment": {"theta_rad":float(fit[0]),"tx_m":float(fit[1]),
                       "ty_m":float(fit[2]),"truncated_registration_cost":fitcost,
                       "multistart_costs":alternatives},
        "wall_metrics":detail,"occupancy_metrics":occupancy,
        "interpretation":"Map similarity after fitting a transform to this same map; optimistic as an accuracy estimate. Does not measure loop closure correctness or independent ground truth."
    }
    args.out.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n")
    save_plot(args.out.with_suffix(".png"),refknown,refocc,sampled_known,
              sampled_occ,refpts,genpts,fit,detail,mirror=args.diagnostic_reflection,
              reference_resolution_m=res)
    print(json.dumps(report,indent=2))
    print("Overlay:",args.out.with_suffix(".png"),file=sys.stderr)
    review_files(args.out.with_suffix(".review.json"), "fr079_map_compare", [
        ("map_agreement_json", args.out.with_suffix(".json"), True),
        ("map_alignment_overlay_png", args.out.with_suffix(".png"), True),
    ])
    return 0

if __name__=="__main__":
    raise SystemExit(main())
