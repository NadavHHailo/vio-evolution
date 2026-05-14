# vio-evolution

Comparative evaluation of four visual-inertial estimation systems on EuRoC MAV: **OpenVINS**, **Basalt**, **ORB-SLAM3**, **SchurVINS**. Measures trajectory accuracy (ATE/RPE), per-frame runtime, and resource footprint (CPU%, peak RSS) on the same three sequences (V1_01_easy, MH_03_medium, V2_02_medium), against the same ground truth.

Companion to [`NadavHHailo/open_vins`](https://github.com/NadavHHailo/open_vins) and the existing `catkin_ws_ov` ROS 2 workspace. Where `catkin_ws_ov` benchmarks OpenVINS in isolation ([cross-platform.md](https://github.com/NadavHHailo/openvins-ros2-workspace/blob/master-candidate/docs/cross-platform/cross-platform.md)), this repo extends that work to a four-way comparison.

## The plan in one paragraph

Treat OpenVINS as the baseline — it already produces per-frame timing CSVs, TUM trajectories, and ATE/RPE via `ov_eval error_singlerun` on x86 and RPi5. Build a sibling harness here that runs each of the other three systems through the same EuRoC sequences against the same ground truth file, normalizes their outputs into the same four files (`<seq>_trajectory.txt`, `<seq>_timing.csv`, `<seq>_proc.csv`, `<seq>_stdout.log`), and aggregates everything into a single side-by-side comparison report. Phased rollout by integration friction: ORB-SLAM3 first (standalone CMake, validates the harness), then Basalt (also standalone CMake), then SchurVINS (ROS 1 Melodic in a Docker container — heaviest friction). x86 only for now; embedded targets reuse the harness later.

Full plan with phase-by-phase steps, file-by-file changes, and verification gates: **[docs/plan.md](docs/plan.md)**.

## Dataset: same data, two on-disk formats

All four systems run on the same EuRoC sensor recordings against the same ground truth, but they consume two different on-disk container formats. Not a fairness issue — same images byte-for-byte, same IMU samples byte-for-byte, different container.

| System | Reads from | Path |
|---|---|---|
| OpenVINS | ROS 2 bag (`.db3`) | `~/datasets/euroc/<seq>/<seq>.db3` |
| ORB-SLAM3 | EuRoC ASL folder (PNG + CSV) | `~/datasets/euroc-asl/<seq>/mav0/...` |
| Basalt | EuRoC ASL folder | same as ORB-SLAM3 |
| SchurVINS | ROS 1 bag (`.bag`) | `~/datasets/euroc-ros1/<seq>.bag` |

Each system's reported wall-ms is the **VIO update time** (frontend + backend), *not* the total including bag/PNG decode — the harness keeps I/O time separate from algorithm time so the comparison stays apples-to-apples. Ground truth is the same `ov_eval`-format file for all four (derived from EuRoC ASL's `state_groundtruth_estimate0/data.csv`).

## Layout

```
systems/
  orb_slam3/   submodule  → NadavHHailo/ORB_SLAM3  (fork of UZ-SLAMLab/ORB_SLAM3)
  basalt/      submodule  → NadavHHailo/basalt     (mirror of VladyslavUsenko/basalt)
  schurvins/   submodule  → NadavHHailo/SchurVINS  (fork of bytedance/SchurVINS)
scripts/
  run_system.sh                 entrypoint: <system> <seq> [--reps N]
  run_eval.sh                   ATE/RPE via ov_eval error_singlerun (from catkin_ws_ov)
  compare_report.py             cross-system aggregator → docs/comparison.md
  adapters/                     per-system trajectory → TUM converters
docs/
  plan.md                       implementation plan (this work)
  comparison.md                 final report (populated by compare_report.py)
  build-{orb_slam3,basalt,schurvins}.md  per-system build notes
```

OpenVINS is intentionally **not** a submodule here — it builds inside the existing `catkin_ws_ov/` ROS 2 colcon workspace and its results are picked up by `compare_report.py` from `~/results/openvins/...`.

## Status

| Phase | What | State |
|---|---|---|
| 0a | Repo bootstrap (this skeleton + plan) | done (commit `72707c5`) |
| 0b | Download EuRoC ASL + ROS 1 `.bag` distributions | pending |
| 0c | Fork upstreams (ORB-SLAM3, Basalt, SchurVINS) into `NadavHHailo/` | pending |
| 1  | ORB-SLAM3 integration + harness validation | pending |
| 2  | Basalt integration | pending |
| 3  | SchurVINS integration (Melodic Docker) | pending |
| 4  | Cross-system comparison report → `docs/comparison.md` | pending |
