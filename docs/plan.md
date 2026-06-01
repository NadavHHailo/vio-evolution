# Comparative Evaluation: OpenVINS vs Basalt vs ORB-SLAM3 vs SchurVINS

## Context

The OpenVINS evaluation pipeline in [`catkin_ws_ov/`](/home/hailo/workspace/catkin_ws_ov/) is mature: it produces per-frame timing CSVs, TUM-style trajectory estimates, and ATE/RPE via `ov_eval error_singlerun`, across four host targets (x86-J, x86-H, RPi5-U, RPi5-T) — documented in [`docs/cross-platform/cross-platform.md`](/home/hailo/workspace/catkin_ws_ov/docs/cross-platform/cross-platform.md). To justify (or replace) OpenVINS as the Hailo VIO backbone, we need the same numbers for **Basalt**, **ORB-SLAM3**, and **SchurVINS** on the same EuRoC sequences, measured against the same ground truth.

**Scope of this plan**: x86-only, phased rollout. RPi5/embedded mirroring is a follow-up that will reuse the harness this plan builds.

**Outcome**: A reusable per-system harness (build, run, adapt-output, evaluate) and a cross-system comparison report covering trajectory accuracy (ATE/RPE), runtime/latency, and resource footprint (CPU%, peak RSS) on EuRoC V1_01_easy, MH_03_medium, V2_02_medium.

---

## Design

### Layout (new sibling git repo: `NadavHHailo/vio-evaluation`)

Don't fold three new systems into `catkin_ws_ov/src/` — their dep stacks (Pangolin, DBoW2, TBB, custom Sophus) will fight OpenVINS' ROS2 environment. Create a new sibling git repo at `https://github.com/NadavHHailo/vio-evaluation`, cloned locally to `/home/hailo/workspace/vio-evaluation/`:

```
/home/hailo/workspace/
  catkin_ws_ov/                    # existing — OpenVINS (untouched)
  vio-evaluation/                   # NEW git repo (NadavHHailo/vio-evaluation)
    .gitmodules                    # pins the three submodules below
    systems/
      orb_slam3/                   # submodule → NadavHHailo/ORB_SLAM3 (fork of UZ-SLAMLab/ORB_SLAM3)
      basalt/                      # submodule → NadavHHailo/basalt    (fork of VladyslavUsenko/basalt)
      schurvins/                   # submodule → NadavHHailo/SchurVINS (fork of bytedance/SchurVINS)
    scripts/
      run_system.sh                # entrypoint: <system> <seq> [--reps N]
      adapters/
        orb_slam3_to_tum.py        # KeyFrameTrajectory.txt → TUM
        basalt_to_tum.py           # basalt traj JSON/txt → TUM
        schurvins_to_tum.py        # ROS topic dump → TUM
      run_eval.sh                  # sources ROS, calls ov_eval error_singlerun
      compare_report.py            # ingests all 4 systems' results, emits table
    docs/
      comparison.md                # final report (mirrors cross-platform.md style)
```

**All three submodules are forked into `NadavHHailo/` first**, then submoduled from the fork (not from upstream). Same pattern as `catkin_ws_ov/src/open_vins` → `NadavHHailo/open_vins`. This is non-optional: at least SchurVINS will almost certainly need timing-instrumentation patches, and likely ORB-SLAM3 will need a small patch to dump per-frame timing as CSV — both require a writable fork. The OpenVINS submodule-management workflow in [`catkin_ws_ov/CLAUDE.md`](/home/hailo/workspace/catkin_ws_ov/CLAUDE.md) (push fork before bumping outer pointer; track branch independently from outer repo) applies verbatim to each new submodule.

**OpenVINS is intentionally *not* a submodule of `vio-evaluation/`.** It only builds inside a ROS 2 colcon workspace, which `catkin_ws_ov/` already provides — duplicating that under `vio-evaluation/systems/openvins/` would give you a non-buildable tree. Instead, OpenVINS benchmark numbers come from running the existing [`catkin_ws_ov/scripts/run_full_benchmark.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/run_full_benchmark.sh) and are picked up by `compare_report.py` from `~/results/openvins/<arch>/<env>/<tag>/` alongside the new three. The "four-system comparison" framing lives in the report and aggregator, not the directory layout.

### Output convention (mirrors existing OpenVINS layout)

```
~/results/<system>/<arch>/<env>/<tag>/
  <seq>_trajectory.txt   # TUM-format: timestamp tx ty tz qx qy qz qw
  <seq>_timing.csv       # timestamp,frontend,backend,total  (system-normalized)
  <seq>_proc.csv         # /usr/bin/time -v derived: peak_rss_kb, user_s, sys_s, %CPU
  <seq>_stdout.log       # full system output for debugging
```

`<system>` ∈ `{openvins, orb_slam3, basalt, schurvins}`. `<arch>/<env>` and `<tag>` follow the existing scheme in [`bench_lib.sh:arch_results_base()`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L20-L29), so the existing parse logic stays compatible for the OpenVINS rows.

### Reused components

- **Ground truth**: `catkin_ws_ov/src/open_vins/src/ov_data/euroc/{V1_01_easy,MH_03_medium,V2_02_medium}.txt` — the same GT files OpenVINS already uses. No re-export needed.
- **ATE/RPE engine**: `ros2 run ov_eval error_singlerun posyaw <gt> <est> 8 16 24 32 40` — called via [`run_eval.sh`](/home/hailo/workspace/vio-evaluation/scripts/run_eval.sh). One-shot per estimate file, no need to reimplement.
- **ROS sourcing pattern**: copy [`bench_lib.sh:source_ros()`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L53-L71) verbatim so `ov_eval` works regardless of the active distro.
- **Sequences + GT alignment**: same three EuRoC sequences as the cross-platform doc. Stereo + IMU (matches OpenVINS benchmark config).

### Dataset: same data, two on-disk formats

The four systems consume two distinct on-disk formats. They wrap the **same underlying ETH EuRoC sensor recordings** (same camera frames byte-for-byte, same IMU samples byte-for-byte, same ground truth) — so the comparison is fair.

| System | Reads from | Path |
|---|---|---|
| OpenVINS | ROS 2 bag (existing, untouched) | `~/datasets/euroc/<seq>/<seq>.db3` |
| ORB-SLAM3 | EuRoC ASL folder (PNG + CSV) | `~/datasets/euroc-asl/<seq>/mav0/...` |
| Basalt | EuRoC ASL folder | same as ORB-SLAM3 |
| SchurVINS | **ROS 1 `.bag`** (confirmed from upstream README: `rosbag play <seq>.bag`) | `~/datasets/euroc-ros1/<seq>.bag` |

**Why not unify**: forcing OpenVINS onto ASL would re-baseline every result already committed to `cross-platform.md` (different input code path → different per-frame timing). Forcing ORB-SLAM3/Basalt onto `.db3` requires writing rosbag2 readers into two upstreams. Both options trade harness simplicity for either lost validation or non-trivial patches. The two-format compromise is cheaper.

**Fair-timing discipline**: each system's reported wall-ms is the **VIO update time** (frontend + backend), *not* the total wall including bag/PNG decode. The OpenVINS harness already separates these via per-stage CSVs; the new systems must replicate the same separation. Bag-vs-PNG I/O time is reported separately, never folded into the VIO comparison number.

**Ground truth**: all four systems evaluate against `catkin_ws_ov/src/open_vins/src/ov_data/euroc/<seq>.txt` (already in `ov_eval` format, derived from EuRoC ASL's `state_groundtruth_estimate0/data.csv`). No GT divergence across systems.

### Per-system runner contract

`run_system.sh <system> <seq>` must produce all four output files. Anything system-specific (build env, EuRoC config path, output post-processing) lives inside a per-system thin wrapper called by `run_system.sh`. The wrapper's job: launch the binary, capture timing, run the adapter, write the four canonical files.

### Resource footprint capture

System-agnostic — wrap each launch in `/usr/bin/time -v` and parse the post-run output for `Maximum resident set size`, `User time`, `System time`, `Percent of CPU this job got`. No need to instrument each codebase. Optional: layer `psrecord --plot` for time-series RAM/CPU graphs if static peaks aren't enough.

### Determinism

All three new systems are non-deterministic by default (multi-threaded frontends, RANSAC without fixed seed). Match OpenVINS' subscribe-mode protocol: **5 reps per sequence**, report mean/std on ATE and wall-ms. Don't chase bit-determinism per system — too much per-system surgery.

---

## Phased rollout (x86)

### Phase 0a — Bootstrap the repo ✅ DONE

1. ✅ Empty repo `NadavHHailo/vio-evaluation` created on GitHub (web UI, no `gh` CLI installed).
2. ✅ Local clone at `/home/hailo/workspace/vio-evaluation/` with skeleton (`systems/`, `scripts/adapters/`, `docs/`), `README.md`, `.gitignore`, and `docs/plan.md` (copy of this file).
3. ✅ Initial commit `72707c5` pushed to `origin/main`.

### Phase 0b — Dataset preparation (ASL + ROS 1 .bag alongside .db3)

Currently `~/datasets/euroc/` contains only ROS 2 bags. Need to add two more formats side-by-side: ASL for ORB-SLAM3/Basalt, and ROS 1 `.bag` for SchurVINS.

1. Download the three EuRoC ASL zips from the ETH page (https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) into `~/datasets/euroc-asl/`:
   - `V1_01_easy.zip` (~1.4 GB), `MH_03_medium.zip` (~1.5 GB), `V2_02_medium.zip` (~1.6 GB).
   - Unzip each into `~/datasets/euroc-asl/<seq>/` so the layout is `~/datasets/euroc-asl/<seq>/mav0/{cam0,cam1,imu0,state_groundtruth_estimate0}/...`.
2. Download the three EuRoC ROS 1 bags from the same ETH page into `~/datasets/euroc-ros1/`:
   - `V1_01_easy.bag`, `MH_03_medium.bag`, `V2_02_medium.bag` (similar sizes to the .db3 bags).
3. Verify the ASL `state_groundtruth_estimate0/data.csv` and the existing `catkin_ws_ov/src/open_vins/src/ov_data/euroc/<seq>.txt` refer to the same trajectory (sanity check — they should, modulo OpenVINS' format conversion).

This keeps `~/datasets/euroc/` (existing `.db3` path for OpenVINS) untouched. Total additional disk: ~9 GB across the three sequences in two formats.

### Phase 0c — Fork the three upstreams into NadavHHailo/

1. `NadavHHailo/ORB_SLAM3` — via GitHub web "Fork" button on https://github.com/UZ-SLAMLab/ORB_SLAM3 (no `gh` CLI installed).
2. `NadavHHailo/SchurVINS` — via GitHub web "Fork" button on https://github.com/bytedance/SchurVINS.
3. `NadavHHailo/basalt` — upstream is on GitLab. Mirror manually:
   - Create empty `NadavHHailo/basalt` on GitHub web UI.
   - `git clone --mirror https://gitlab.com/VladyslavUsenko/basalt.git /tmp/basalt-mirror`
   - `cd /tmp/basalt-mirror && git push --mirror git@github.com:NadavHHailo/basalt.git`

### Phase 1 — Harness validation on ORB-SLAM3 (smoothest first)

ORB-SLAM3 first because: ships a ready-made [`Examples/Stereo-Inertial/stereo_inertial_euroc`](https://github.com/UZ-SLAMLab/ORB_SLAM3) with EuRoC configs and `EuRoC_TimeStamps/`, has a well-known TUM-format trajectory output (`CameraTrajectory.txt`), and the largest community → least time spent debugging build issues. Validating the full harness end-to-end here de-risks the other two.

Steps:
1. Add ORB-SLAM3 as a submodule at `vio-evaluation/systems/orb_slam3/` (`git submodule add <url> systems/orb_slam3`). Pin to a known-good tag. Build natively on x86 (Pangolin, OpenCV ≥3, Eigen3, DBoW2/g2o vendored). Document exact build incantation in `vio-evaluation/docs/build-orb-slam3.md`.
2. Write `vio-evaluation/systems/orb_slam3/run.sh` — invokes `stereo_inertial_euroc` against the EuRoC bag dirs already at `~/datasets/euroc/`. Note: ORB-SLAM3 wants the *image folder* layout (`mav0/cam0/data/*.png`), not the rosbag — confirm the dataset on disk has both.
3. Adapter `adapters/orb_slam3_to_tum.py`: convert `CameraTrajectory.txt` (already TUM-ish, but timestamps may be in ns vs the GT's seconds) to the canonical `<seq>_trajectory.txt`. Validate timestamp alignment against GT.
4. Timing: ORB-SLAM3 exposes `Tracking::vdTrackTotal_ms` etc. — enable the dump (or patch a one-shot CSV writer onto the example main). Normalize to `timestamp,frontend,backend,total`.
5. End-to-end smoke test: run all three sequences × 5 reps. Verify `run_eval.sh` outputs sensible ATE on V1_01_easy (expect sub-10 cm RMSE — ORB-SLAM3 paper reports ~0.04 m on V1_01).
6. **Gate**: harness is valid if `compare_report.py` emits a table with both `openvins` and `orb_slam3` rows side-by-side, fed from `~/results/{openvins,orb_slam3}/x86/native_jazzy/...`.

### Phase 2 — Basalt

Second because: still a standalone CMake build (no ROS dependency for offline VIO), same dataset format (EuRoC ASL) as Phase 1, so the harness scaffold from Phase 1 transfers almost verbatim. Heavier deps than ORB-SLAM3 (TBB, Pangolin, custom Sophus fork) but no Docker/distro juggling.

Steps:
1. Add Basalt as a submodule at `vio-evaluation/systems/basalt/` (Basalt vendors `basalt-headers` as its own submodule, so a recursive `git submodule update --init --recursive` after adding it picks both up). Build natively (CMake + TBB + Pangolin). Document gotchas in `vio-evaluation/docs/build-basalt.md`.
2. Reuse the EuRoC calibration JSON shipped in Basalt's `data/euroc_ds_calib.json` if present, or convert from `kalibr_imucam_chain.yaml` if upstream stopped shipping it.
3. Runner: invoke `basalt_vio --dataset-path ~/datasets/euroc-asl/<seq> --cam-calib <calib.json> --config-path data/euroc_config.json --result-path /tmp/basalt_traj.txt`.
4. Adapter `adapters/basalt_to_tum.py`: Basalt's trajectory output is already TUM-formatted text — just normalize timestamp units and rename.
5. Timing: Basalt's `--show-gui` mode logs per-frame stats; offline mode dumps them to stderr — capture and parse.
6. 5-rep run × 3 sequences; cross-check against Basalt RA-L 2020 Table 2 (ATE on EuRoC).

### Phase 3 — SchurVINS (heaviest friction — ROS 1 Melodic in Docker)

Last because the integration friction is qualitatively different: SchurVINS' upstream README mandates **Ubuntu 18.04 + ROS 1 Melodic**, both EOL. Neither runs natively on the host (24.04 Noble). Realistic path is a Melodic Docker container, mirroring the `openvins-humble:latest` pattern that `catkin_ws_ov` already uses for `rpi5-T`.

Steps:
1. Add SchurVINS as a submodule at `vio-evaluation/systems/schurvins/`.
2. Write `vio-evaluation/systems/schurvins/Dockerfile` based on `osrf/ros:melodic-desktop`. Install SVO/SchurVINS deps (Eigen, OpenCV 3.x, Sophus, glog, yaml-cpp) and `catkin_make` the SchurVINS workspace inside the image. Pin the base image digest for reproducibility.
3. System runner mounts `~/datasets/euroc-ros1/` and `~/results/schurvins/` into the container; runs `roslaunch svo_ros euroc_vio_stereo.launch` + a sidecar `rosbag play <seq>.bag --rate 1.0`. Capture the published odometry topic (likely `/svo/pose` or `/svo/odometry` — confirm against the launch file) to a TUM-format file via a thin `rostopic echo` + adapter.
4. Adapter `adapters/schurvins_to_tum.py`: parse the captured topic dump → canonical `<seq>_trajectory.txt`.
5. Timing: instrument SchurVINS' frontend/backend dispatch points if no built-in CSV. Worst case, expose only `total` (wall ms per processed frame from log lines) and accept reduced per-stage breakdown.
6. 5-rep run × 3 sequences; cross-check against numbers in the SchurVINS CVPR 2024 paper Table 2 (ATE on EuRoC).

### Phase 4 — Comparison report

Once all four systems have populated `~/results/<system>/x86/native_jazzy/<tag>/`:

1. `compare_report.py` (new): walk all four `<system>` dirs, for each `(system, seq)` aggregate over reps, call `ov_eval error_singlerun` once per estimate file, and emit:
   - **Headline table**: per-sequence ATE (m, mean±std), wall-ms/frame (mean±std), peak RSS (MB).
   - **Per-stage timing**: where available, side-by-side frontend/backend ms. Note which systems only expose `total` (likely SchurVINS).
   - **Ranking**: vs OpenVINS x86-J as the baseline (matches existing cross-platform.md convention).
2. Render the report to `vio-evaluation/docs/comparison.md` following the same style as [`cross-platform.md` §2.1](/home/hailo/workspace/catkin_ws_ov/docs/cross-platform/cross-platform.md#21-subscribe-summary-across-sequences--the-at-a-glance-table) — citations per table, reproducible invocation block at the top, known caveats called out (e.g., ORB-SLAM3's loop closure makes ATE strictly better than pure VIO — flag it).

---

## Critical files to read/modify

**Reused (no edits)**:
- [`catkin_ws_ov/scripts/bench_lib.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh) — copy `source_ros()` and `detect_ros_distro()` patterns into `vio-evaluation/scripts/run_eval.sh`.
- [`catkin_ws_ov/scripts/parse_results.py`](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py) — reference for the `error_singlerun` invocation form (lines 69-100); reuse same ANSI strip + RPE regex.
- `catkin_ws_ov/src/open_vins/src/ov_data/euroc/*.txt` — ground truth files.

**New (to be created)**:
- `vio-evaluation/` itself — `git init` + initial `.gitignore` (results/build artifacts).
- `vio-evaluation/.gitmodules` — three submodules (orb_slam3, basalt, schurvins), pinned to upstream tags or fork commits.
- `vio-evaluation/scripts/run_system.sh` — entrypoint dispatcher.
- `vio-evaluation/scripts/run_eval.sh` — `ov_eval` wrapper.
- `vio-evaluation/scripts/adapters/{orb_slam3,basalt,schurvins}_to_tum.py` — trajectory converters.
- `vio-evaluation/scripts/compare_report.py` — cross-system aggregator.
- `vio-evaluation/systems/<system>/run.sh` — per-system launcher (3 files).
- `vio-evaluation/docs/{build-orb-slam3,build-basalt,build-schurvins,comparison}.md`.

**Existing untouched**:
- All of `catkin_ws_ov/` — OpenVINS pipeline keeps producing baseline numbers from existing `run_full_benchmark.sh` invocations.

---

## Verification

End-to-end correctness gates (run after each phase):

1. **Harness gate** (after Phase 1): `compare_report.py` produces a 2-system table (OpenVINS + ORB-SLAM3) where:
   - ATE on V1_01_easy is sub-15 cm for both (sanity floor).
   - ORB-SLAM3 ATE is in the ballpark of its paper (~0.04 m on V1_01_easy stereo-inertial).
   - Timing CSVs have ≥2000 rows per sequence (every EuRoC bag has >2000 frames; matches `MIN_ROWS_FOR_COMPLETE_RUN` check in [bench_lib.sh:214](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L214)).
2. **Per-system smoke** (after each system added): `run_system.sh <sys> V1_01_easy` produces all four canonical output files; trajectory file passes a TUM-format syntactic check (`python3 -c "import numpy as np; np.loadtxt('<file>'); assert np.loadtxt('<file>').shape[1] == 8"`).
3. **Numbers cross-check**: each system's V1_01_easy ATE is within ~2× of the value in its respective paper. Discrepancies > 2× indicate config or alignment bugs (likely culprit: timestamp units or stereo extrinsics not picked up from EuRoC's `mav0/cam{0,1}/sensor.yaml`).
4. **Final comparison table**: re-render `vio-evaluation/docs/comparison.md` from a clean re-run of `compare_report.py` against `~/results/*/x86/native_jazzy/<tag>/`. Spot-check one cell per system against its source CSV.

Reproducible end-to-end check (single command, full sweep):
```bash
for sys in openvins orb_slam3 basalt schurvins; do
  for seq in V1_01_easy MH_03_medium V2_02_medium; do
    bash vio-evaluation/scripts/run_system.sh "$sys" "$seq" --reps 5 --tag baseline_x86
  done
done
python3 vio-evaluation/scripts/compare_report.py ~/results --tag baseline_x86 --out vio-evaluation/docs/comparison.md
```

---

## Out of scope (deferred)

- **RPi5 / Hailo embedded targets** — explicitly Phase-5+. Once `vio-evaluation/` works on x86, the same per-system Docker-image pattern that gave OpenVINS its `rpi5-T` (openvins-humble container) can be repeated per system. Don't pre-design the Dockerfiles now; the constraints will be system-specific and easier to address once the x86 harness exists.
- **Loop closure fairness debate** — ORB-SLAM3 will look unfairly good on ATE because of its loop closure. Note it in the report; don't try to disable it (configuration surgery that diverges from "stock" defeats the point of the comparison).
- **Additional sequences** beyond V1_01/MH_03/V2_02 — leave for later. The existing three are what OpenVINS cross-platform.md uses; comparing on the same set is the priority.
- **Multi-rate / threading sweeps** — defer until the four-system baseline is stable. The existing OpenVINS `--rate` and `--threads` sweep dimensions can be added back per-system once each runs cleanly at the default (1.0 rate, stock thread count).
