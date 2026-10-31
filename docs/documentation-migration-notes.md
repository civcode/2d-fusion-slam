# Documentation Migration Notes — v0.2

**Date:** 2026-10-31  
**Branch:** `review/benchmark-first-architecture`  
**Status:** Documentation refactor; no code or benchmark executions are claimed.

## Rationale

The earlier v0.1 documents encoded many valuable correctness requirements, but treated unmeasured optimization choices as accepted first-implementation mandates. v0.2 retains source/time/SE(2), map, graph-evidence, and pose-validity guarantees while centering the first implementation and tests on **synthetic and validated public datasets**. LD19/MPU6050 Raspberry Pi 5 capture is developed **in parallel**, not as a prerequisite for these initial tests.

| Superseded initial requirement | Revised v0.2 direction |
| --- | --- |
| Exactly eight CMake libraries, four SDK exports and three daemons | Incremental real targets; installed SDK scope follows real consumers |
| Four mandatory runtime processes and Web Lab | Bounded owners/channels first; optional measured process isolation |
| Wheel+gyro as assumed baseline | LiDAR+gyro supported without wheels; measured wheel translation optional |
| Full SE(2) deskew without source qualification | Keep SE(2) reference math; use gyro-only until credible XY source |
| Fixed Ceres odometry prior | Compare geometry-only vs regularized tracker, no conditioned graph factor reuse |
| Fixed 90/180 scans, 0.20 m/5° keyframes | Candidate experiments with identical datasets and alternative thresholds |
| Incremental EDT 32×32 tiles | Coherent full-field EDT first; optimize after Pi bottleneck evidence |
| Full keyframe/submap/bias GTSAM graph first | Simple pose-only workstation batch reference with unique source factors |
| Exhaustive global BnB recovery | Safe bounded top-K interface; compare BnB/particle/hybrid |
| Persistent submap-only online graph | Defer Pi graph topology until localizer profiling and long-session trials |
| Indexed replay and semantic checkpoints first | Deterministic sequential replay first; optimize seek on measured need |
| “Accepted” conflated design and proof | Separate design selected / implemented / synthetic / public / real device / Pi evidence |

The five canonical documents and repository README are the current normative baseline. They include no claimed executed tests, current benchmark metrics or mandatory unavailable tooling. The detailed v0.1 proposal is preserved in repository history; old choices are not live implementation requirements.
