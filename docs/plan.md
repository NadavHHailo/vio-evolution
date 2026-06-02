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
      db3_to_asl.py                # reconstruct EuRoC ASL layout from a ROS 2 .db3 (ETH host dead)
      run_system.sh                # entrypoint: <system> <seq> [--reps N]
      adapters/
        orb_slam3_to_tum.py        # f_*.txt (per-frame TUM) → canonical TUM
        basalt_to_tum.py           # basalt traj JSON/txt → TUM
        schurvins_to_tum.py        # ROS topic dump → TUM
      compare_report.py            # ingests all 4 systems' results, emits table
                                   #   (imports catkin_ws_ov parse_results.py for ATE/RPE)
    docs/
      comparison.md                # final report (mirrors cross-platform.md style)
```

**All three submodules are forked into `NadavHHailo/` first**, then submoduled from the fork (not from upstream). Same pattern as `catkin_ws_ov/src/open_vins` → `NadavHHailo/open_vins`. This is non-optional: at least SchurVINS will almost certainly need timing-instrumentation patches, and likely ORB-SLAM3 will need a small patch to dump per-frame timing as CSV — both require a writable fork. The OpenVINS submodule-management workflow in [`catkin_ws_ov/CLAUDE.md`](/home/hailo/workspace/catkin_ws_ov/CLAUDE.md) (push fork before bumping outer pointer; track branch independently from outer repo) applies verbatim to each new submodule.

**OpenVINS is intentionally *not* a submodule of `vio-evaluation/`.** It only builds inside a ROS 2 colcon workspace, which `catkin_ws_ov/` already provides — duplicating that under `vio-evaluation/systems/openvins/` would give you a non-buildable tree. Instead, OpenVINS benchmark numbers come from running the existing [`catkin_ws_ov/scripts/run_full_benchmark.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/run_full_benchmark.sh) and are picked up by `compare_report.py` from their existing location `~/results/<arch>/<env>/<tag>/<mode>/<seq>_<N>thr_est.txt` (no `<system>` prefix — see §"Output convention") alongside the new three. The "four-system comparison" framing lives in the report and aggregator, not the directory layout.

### Output convention (two distinct schemes — do not conflate them)

OpenVINS results already exist on disk and are **not** moved or renamed. The three new systems get a fresh, sibling scheme. `compare_report.py` reads each from its own path.

**OpenVINS (existing, untouched)** — produced by `run_full_benchmark.sh` via [`bench_lib.sh:arch_results_base()`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L20-L29):

```
~/results/<arch>/<env>/<tag>/<mode>/
  <seq>_<N>thr_est.txt    # OpenVINS state-dump (NOT TUM): timestamp qx qy qz qw px py pz vx ...
  <seq>_<N>thr_wall.txt   # per-frame wall timing
  <seq>_<N>thr_cpu.txt, _feats.txt, _thread.txt
```
`<mode>` ∈ `{serial, subscribe}`; `<N>` ∈ thread counts. There is **no** `<system>` prefix.

**Three new systems (new scheme):**

```
~/results/<system>/<arch>/<env>/<tag>/
  <seq>_trajectory.txt   # TUM-format: timestamp tx ty tz qx qy qz qw  (timestamps in seconds)
  <seq>_timing.csv       # timestamp,frontend,backend,total  (system-normalized)
  <seq>_proc.csv         # /usr/bin/time -v derived: peak_rss_kb, user_s, sys_s, %CPU
  <seq>_stdout.log       # full system output for debugging
```

`<system>` ∈ `{orb_slam3, basalt, schurvins}`. `<arch>/<env>/<tag>` reuse the same vocabulary as `arch_results_base()` (e.g. `x86/native_jazzy/baseline_x86`) but with the `<system>` prefix added and **no** `<mode>` subdir. These differences are why `compare_report.py` cannot uniformly walk `~/results/<system>/...` for all four — it special-cases the OpenVINS path (see §"Reused components").

### Reused components

- **Ground truth**: `catkin_ws_ov/src/open_vins/ov_data/euroc_mav/{V1_01_easy,MH_03_medium,V2_02_medium}.txt` — the same GT files OpenVINS already uses (header `# timestamp(s) tx ty tz qx qy qz qw`, timestamps in **seconds**, 8 data columns — already TUM-compatible). No re-export needed.
- **ATE/RPE + parsing engine**: reuse [`catkin_ws_ov/scripts/parse_results.py`](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py) directly — import its `run_ate_rpe(gt, traj)` ([L251](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py#L251)), `_run_ros2`, `_RPE_RE`, and timing parsers (`parse_timing_csv`, `parse_timing_csv_components`). The underlying call is `ros2 run ov_eval error_singlerun posyaw <gt> <tum>` (no segment-length args) run with `MPLBACKEND=Agg` and a ~15 s timeout, because `error_singlerun` blocks on `matplotlibcpp::show()`. `_run_ros2` already encapsulates this — don't reimplement it in a shell wrapper.
  - **Caveat**: `run_ate_rpe` internally calls `_est_to_tum(traj)`, which assumes an OpenVINS **state-dump** input and reorders columns. The three new systems emit **TUM directly**, so passing them through `run_ate_rpe` would double-convert (quaternion read as position → wildly inflated ATE). `compare_report.py` must call `error_singlerun` on their TUM files directly (reusing `_run_ros2` + `_RPE_RE`, bypassing `_est_to_tum`). Only OpenVINS rows go through `_est_to_tum`.
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

**Ground truth**: all four systems evaluate against `catkin_ws_ov/src/open_vins/ov_data/euroc_mav/<seq>.txt` (already in `ov_eval`/TUM format — seconds, 8 columns — derived from EuRoC ASL's `state_groundtruth_estimate0/data.csv`). No GT divergence across systems.

### Per-system runner contract

Each per-system runner must produce all four output files. The runner launches the binary, captures timing, runs the adapter, and writes the four canonical files. **Runners live in the outer repo at `scripts/run_<system>.sh`** (e.g. `scripts/run_orb_slam3.sh`) — *not* inside `systems/<system>/`, because that directory is the system's submodule and a file there would belong to the fork, not to `vio-evaluation`.

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

### Phase 0b — Dataset preparation (ASL + ROS 1 .bag alongside .db3) ✅ ASL DONE; ROS 1 bags deferred to Phase 3

`~/datasets/euroc/` had only ROS 2 bags. Added two more formats side-by-side: ASL for
ORB-SLAM3/Basalt, ROS 1 `.bag` for SchurVINS.

**The canonical ETH download host `robotics.ethz.ch` is dead** (port 80 times out; DNS resolves
but no TCP). It is the only published source of the ASL `.zip` archives, and the
`projects.asl.ethz.ch` landing page only links back to it. So the original "download the ASL
zips" step is not executable. Two workarounds, both verified:

1. ✅ **ASL — reconstructed locally from the existing `.db3`** (no download). The ROS 2 bag
   already carries `/cam0/image_raw`, `/cam1/image_raw`, `/imu0` — the same frames/IMU OpenVINS
   benchmarks against. [`scripts/db3_to_asl.py`](/home/hailo/workspace/vio-evaluation/scripts/db3_to_asl.py)
   (ROS 2 `rosbag2_py` + `cv_bridge`) writes the EuRoC ASL layout
   `~/datasets/euroc-asl/<seq>/mav0/{cam0,cam1}/data/<ts_ns>.png` (+ `data.csv`) and
   `mav0/imu0/data.csv`. Frame counts match EuRoC exactly: V1_01_easy 2912, MH_03_medium 2700,
   V2_02_medium 2348 stereo pairs (3.0 GB total). Calibration yamls are not in the bag and not
   needed (ORB-SLAM3 ships `EuRoC.yaml`; Basalt takes `--cam-calib` JSON).
2. **ROS 1 `.bag` — DEFERRED to Phase 3** (SchurVINS only; not on the Phase 1/2 critical path).
   The ETH host is dead here too. The [OpenVINS datasets page](https://docs.openvins.com/gs-datasets.html)
   mirrors each EuRoC ROS 1 bag on Google Drive (IDs: V1_01_easy `1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh`,
   MH_03_medium `1er07gZ8rso8R3Su00hJMm_GZ4z1n9Rpq`, V2_02_medium `1Gj4psmvcAwYwCp4T4CQH-d2ZVJ09d3x2`),
   but `gdown` currently can't fetch them ("cannot retrieve public link" — Drive access-quota
   throttling on these popular files). **Host-independent fallback** (preferred, mirrors the ASL
   approach): convert the local `~/datasets/euroc/<seq>/<seq>.db3` → ROS 1 `.bag` with the
   `rosbags` library (`rosbags-convert`). The db3 already has `/cam0/image_raw`, `/cam1/image_raw`,
   `/imu0` — the exact topics SchurVINS' launch consumes — so no re-download is needed. Resolve
   when Phase 3 begins.
3. ✅ **GT sanity check**: reconstructed cam0 first timestamp `1403715273.262143 s` matches the GT
   start in `ov_data/euroc_mav/V1_01_easy.txt` (`1403715273.26214`) to the microsecond — the bag
   preserves original EuRoC sensor timestamps, so ORB-SLAM3's `EuRoC_TimeStamps/<seq>.txt` and the
   GT will align. Images verified 480×752 mono8.

`~/datasets/euroc/` (the `.db3` path for OpenVINS) is untouched. Added disk: ASL ~3 GB + ROS 1 bags ~8 GB.

### Phase 0c — Fork the three upstreams into NadavHHailo/ ✅ DONE (all public)

1. ✅ `NadavHHailo/ORB_SLAM3` — forked from https://github.com/UZ-SLAMLab/ORB_SLAM3 via GitHub web UI (no `gh` CLI / API token on the box). Carried tags incl. `v1.0-release` (`0df83dd`) for pinning.
2. ✅ `NadavHHailo/SchurVINS` — forked from https://github.com/bytedance/SchurVINS. HEAD `d8ab6df` (upstream has no tags → pin to this commit).
3. ✅ `NadavHHailo/basalt` — upstream is on GitLab, so not a GitHub fork. Empty repo created via web UI, then mirrored locally over SSH:
   - `git clone --mirror https://gitlab.com/VladyslavUsenko/basalt.git /tmp/basalt-mirror`
   - `git -c safe.bareRepository=all --git-dir=/tmp/basalt-mirror push --mirror git@github.com:NadavHHailo/basalt.git`
   - (`--git-dir` + `safe.bareRepository=all` needed because the host git sets `safe.bareRepository=explicit`.) Landed `master` (`0f3b2b5`) + tags 0.1.0–0.1.7. Note the mirror also copied GitLab `refs/merge-requests/*` and `refs/pipelines/*` — harmless, ignore.

### Phase 1 — Harness validation on ORB-SLAM3 ✅ VALIDATED on V1_01_easy

ORB-SLAM3 first because: ships a ready-made `Examples/Stereo-Inertial/stereo_inertial_euroc`, emits per-frame trajectory output, and has the largest community. Validating the full harness end-to-end here de-risks the other two.

**As built (deviations from the original sketch noted inline):**
1. ✅ **Submodule + build.** `systems/orb_slam3` → `NadavHHailo/ORB_SLAM3` @ `v1.0-release`, on branch `vio-eval-build`. Pangolin **v0.6** built from source to `~/opt/pangolin` (v0.6, not v0.8 — uses the present GLEW/X11, no apt/sudo). ORB-SLAM3 patches were minimal: **C++14** (not C++11), drop Sophus `-Werror`; no OpenCV-4 code patches needed. Full recipe in [`docs/build-orb-slam3.md`](/home/hailo/workspace/vio-evaluation/docs/build-orb-slam3.md).
2. ✅ **Runner** lives at [`scripts/run_orb_slam3.sh`](/home/hailo/workspace/vio-evaluation/scripts/run_orb_slam3.sh) — **NOT** `systems/orb_slam3/run.sh`: that path is the submodule, so a runner there would belong to the fork. Per-system runners live in the outer repo's `scripts/`. It generates the timestamps file from our own ASL cam0 frames (the shipped `EuRoC_TimeStamps/*.txt` use float-rounded ns that don't match our bag-derived PNG names), runs the binary headless (`bUseViewer=false` already in the example), wraps it in `/usr/bin/time -v` → `<seq>_proc.csv`, and writes per-rep canonical files under `~/results/orb_slam3/x86/native_jazzy/<tag>/`.
3. ✅ **Adapter** [`scripts/adapters/orb_slam3_to_tum.py`](/home/hailo/workspace/vio-evaluation/scripts/adapters/orb_slam3_to_tum.py): ORB-SLAM3's `f_<seq>.txt` is TUM-ordered but timestamped in **nanoseconds** → divide by 1e9 → `<seq>_trajectory.txt`; asserts 8 cols + GT time-overlap.

**Alignment**: the harness evaluates **all** systems with `se3` (not `posyaw`). ORB-SLAM3's output frame differs from the EuRoC GT by a full 3D rotation, so `posyaw` gave a bogus 1.72 m; `se3` gives **0.020 m** ATE pos RMSE on V1_01_easy (sim3 0.011 m, RPE ~3 cm/8 m) — paper ballpark. `compare_report.py` recomputes OpenVINS with `se3` too (does not touch `cross-platform.md`'s `posyaw` record).
4. **Timing patch** (the fork's reason to exist): the example main accumulates `vdTrackTotal_ms` / `vdLocalMapTrack_ms` but only prints median/mean at shutdown. Patch it to dump per-frame `timestamp,frontend,backend,total` → `<seq>_timing.csv` (frontend = tracking, backend = local mapping), matching OpenVINS' frontend/backend separation discipline. Bag/PNG decode time stays out of these numbers.
5. **Smoke test**: V1_01_easy × 5 reps, then all three × 5 reps once V1_01 looks right. ATE via the reused `error_singlerun` path (TUM input, no `_est_to_tum`). Expect sub-10 cm RMSE (ORB-SLAM3 paper ≈ 0.04 m on V1_01_easy stereo-inertial).
6. **Gate**: harness is valid when `compare_report.py` emits a table with both `openvins` and `orb_slam3` rows side-by-side — OpenVINS from `~/results/x86/native_jazzy/<tag>/<mode>/<seq>_<N>thr_est.txt` (via `_est_to_tum`), ORB-SLAM3 from `~/results/orb_slam3/x86/native_jazzy/<tag>/<seq>_trajectory.txt` (TUM direct). Timing CSV ≥ 2000 rows. (Note the loop-closure caveat — see Out of scope.)

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

Once OpenVINS results exist at `~/results/x86/native_jazzy/<tag>/<mode>/` and the three new systems have populated `~/results/<system>/x86/native_jazzy/<tag>/`:

1. `compare_report.py` (new): collect results per the two-path scheme (§"Output convention") — OpenVINS from its existing `<seq>_<N>thr_est.txt` (state-dump → `_est_to_tum`), the three new systems from `<seq>_trajectory.txt` (TUM, fed to `error_singlerun` directly). For each `(system, seq)` aggregate over reps, run `error_singlerun` once per estimate file (reusing `parse_results._run_ros2` + `_RPE_RE`), and emit:
   - **Headline table**: per-sequence ATE (m, mean±std), wall-ms/frame (mean±std), peak RSS (MB).
   - **Per-stage timing**: where available, side-by-side frontend/backend ms. Note which systems only expose `total` (likely SchurVINS).
   - **Ranking**: vs OpenVINS x86-J as the baseline (matches existing cross-platform.md convention).
2. Render the report to `vio-evaluation/docs/comparison.md` following the same style as [`cross-platform.md` §2.1](/home/hailo/workspace/catkin_ws_ov/docs/cross-platform/cross-platform.md#21-subscribe-summary-across-sequences--the-at-a-glance-table) — citations per table, reproducible invocation block at the top, known caveats called out (e.g., ORB-SLAM3's loop closure makes ATE strictly better than pure VIO — flag it).

---

## Critical files to read/modify

**Reused (no edits)**:
- [`catkin_ws_ov/scripts/parse_results.py`](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py) — **import directly**: `run_ate_rpe` ([L251](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py#L251)) for OpenVINS rows, plus `_run_ros2` (MPLBACKEND=Agg + timeout), `_RPE_RE`, ANSI strip, and timing parsers for all rows. New systems call `error_singlerun` on TUM directly (bypass `_est_to_tum`).
- [`catkin_ws_ov/scripts/bench_lib.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh) — reference for `arch_results_base()` path vocabulary and `detect_ros_distro()`.
- `catkin_ws_ov/src/open_vins/ov_data/euroc_mav/*.txt` — ground truth files (TUM, seconds).

**New (to be created)**:
- `vio-evaluation/` itself — `git init` + initial `.gitignore` (results/build artifacts).
- `vio-evaluation/.gitmodules` — three submodules (orb_slam3, basalt, schurvins), pinned to upstream tags or fork commits.
- `vio-evaluation/scripts/db3_to_asl.py` — ✅ created; reconstructs EuRoC ASL from the ROS 2 `.db3`.
- `vio-evaluation/scripts/run_system.sh` — entrypoint dispatcher.
- `vio-evaluation/scripts/adapters/{orb_slam3,basalt,schurvins}_to_tum.py` — trajectory converters.
- `vio-evaluation/scripts/compare_report.py` — cross-system aggregator; imports `parse_results.py` for ATE/RPE + timing (no separate `run_eval.sh` shell wrapper needed).
- `vio-evaluation/scripts/run_<system>.sh` — per-system launcher (3 files; in the outer repo, not inside the submodule).
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
4. **Final comparison table**: re-render `vio-evaluation/docs/comparison.md` from a clean re-run of `compare_report.py ~/results --tag <tag>` (which resolves both path schemes internally). Spot-check one cell per system against its source CSV.

Reproducible end-to-end check (full sweep):
```bash
# OpenVINS baseline comes from the existing catkin harness (its own path scheme):
bash ~/workspace/catkin_ws_ov/scripts/run_full_benchmark.sh --tag baseline_x86   # writes ~/results/x86/native_jazzy/baseline_x86/<mode>/...

# The three new systems go through this repo's dispatcher:
for sys in orb_slam3 basalt schurvins; do
  for seq in V1_01_easy MH_03_medium V2_02_medium; do
    bash vio-evaluation/scripts/run_system.sh "$sys" "$seq" --reps 5 --tag baseline_x86
  done
done

# Aggregator reads both schemes (see §"Output convention"):
python3 vio-evaluation/scripts/compare_report.py ~/results --tag baseline_x86 --out vio-evaluation/docs/comparison.md
```

---

## Out of scope (deferred)

- **RPi5 / Hailo embedded targets** — explicitly Phase-5+. Once `vio-evaluation/` works on x86, the same per-system Docker-image pattern that gave OpenVINS its `rpi5-T` (openvins-humble container) can be repeated per system. Don't pre-design the Dockerfiles now; the constraints will be system-specific and easier to address once the x86 harness exists.
- **Loop closure fairness** — ORB-SLAM3 can look unfairly good on ATE vs the pure-VIO systems because of loop closure. Rather than leave this as a caveat, the runner offers a **`--vio-only`** mode that disables loop closure / global BA via ORB-SLAM3's own `loopClosing: 0` setting (no "stock"-diverging surgery — it's a built-in config key). This yields an apples-to-apples VIO comparison; full-SLAM runs are kept too (separate `<tag>_vioonly/` dir) so the loop-closure benefit is also measurable. On V1_01_easy the two are within noise (0.019 vs 0.020 m); the gap grows on loop-heavy sequences.
- **Additional sequences** beyond V1_01/MH_03/V2_02 — leave for later. The existing three are what OpenVINS cross-platform.md uses; comparing on the same set is the priority.
- **Multi-rate / threading sweeps** — defer until the four-system baseline is stable. The existing OpenVINS `--rate` and `--threads` sweep dimensions can be added back per-system once each runs cleanly at the default (1.0 rate, stock thread count).
