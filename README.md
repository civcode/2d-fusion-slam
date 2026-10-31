# 2D Fusion SLAM

**Project stage:** Architecture and benchmark planning. No SLAM source code, fixtures, build configurations, or benchmark results are implemented in this repository yet.

The target sensor combination is **LD19 2D LiDAR + MPU6050 gyro**, with **wheel odometry optional and only used if actually measured**. The portable C++20 core is designed for three modes:

| Profile | Environment | Goal |
| --- | --- | --- |
| **Offline SLAM** | Powerful workstation | Reproducible Ceres/GTSAM batch mapping and reliable map export |
| **Known-map localization** | Raspberry Pi 5 | Read-only map, bounded tracking and explicit global pose validity, no initial GTSAM |
| **Online SLAM** | Raspberry Pi 5, later | Reduced, resource-limited incremental mapping after benchmarks |

**Initial implementation and benchmark priority:** deterministic synthetic apartment/warehouse fixtures and opt-in validated public datasets. **In parallel**, develop LD19/MPU6050 Pi acquisition and native raw recording; its actual hardware measurements do not block the initial algorithmic milestones. Real-device calibration and Pi profiling are required before claiming hardware-specific performance.

## Current specifications

- [Architecture and runtime modes](docs/architecture-specification.md)
- [Technical and sensor-capability requirements](docs/technical-specification.md)
- [Implementation and dependency contract](docs/implementation-contract.md)
- [Synthetic/public datasets and acceptance plan](docs/benchmarks-and-datasets.md)
- [Decisions OD-001–OD-025 and evidence status](docs/open-design-choices.md)
- [Migration rationale from v0.1](docs/documentation-migration-notes.md)

**Working rule:** Preserve rigorous time/frame/source/map/pose contracts immediately; make sophisticated solvers, caches, graph summarization, relocalization and process topology earn their complexity through reproducible benchmarks.

No GitHub Actions are required by the design. Any future workflow should be manual-only by default, and tests should run locally or in Docker when code exists.
