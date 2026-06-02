# VIO comparison — Evaluation-DR §3.1 metrics (align=se3, tag=baseline_x86)

Aggregated mean ± std over reps. Accuracy via `ov_eval error_singlerun`; latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`.
RPE is the mean of per-segment medians (8/16/24/32/40 m). ORB-SLAM3 runs in **sequential** mode and is reported in two variants: **(SLAM)** = full pipeline with loop closure / global BA, and **(VIO-only)** = `loopClosing: 0` (pure sliding-window VIO, comparable to OpenVINS/Basalt/SchurVINS). **x86 performance figures are illustrative** (DR: perf profiling belongs on embedded HW). ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect the per-frame tracking front-end.

| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.113 ± 0.000 | 1.17 ± 0.00 | 0.160 ± 0.000 | 1.19 ± 0.00 | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | V1_01_easy | 0.020 ± 0.000 | 0.41 ± 0.00 | 0.029 ± 0.000 | 0.42 ± 0.00 | 96.5 | 0.0 | 28.3/37.6 | 35.4 | 324 | 805 | 1 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.019 ± 0.000 | 0.43 ± 0.00 | 0.029 ± 0.000 | 0.43 ± 0.00 | 96.5 | 0.0 | 33.4/45.7 | 30.0 | 334 | 814 | 1 |
| openvins | MH_03_medium | — | — | — | — | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | MH_03_medium | 0.025 ± 0.000 | 1.10 ± 0.00 | 0.086 ± 0.000 | 0.53 ± 0.00 | 86.3 | 0.0 | 29.5/40.1 | 34.3 | 330 | 836 | 1 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.026 ± 0.000 | 1.15 ± 0.00 | 0.093 ± 0.000 | 0.56 ± 0.00 | 86.4 | 0.0 | 34.9/48.7 | 29.0 | 336 | 837 | 1 |
| openvins | V2_02_medium | — | — | — | — | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | V2_02_medium | 0.019 ± 0.000 | 0.85 ± 0.00 | 0.036 ± 0.000 | 1.11 ± 0.00 | 96.9 | 0.0 | 28.9/37.7 | 35.1 | 336 | 833 | 1 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.022 ± 0.000 | 0.87 ± 0.00 | 0.038 ± 0.000 | 1.10 ± 0.00 | 96.9 | 0.0 | 33.9/44.0 | 29.9 | 343 | 829 | 1 |
