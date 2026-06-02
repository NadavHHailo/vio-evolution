# Building ORB-SLAM3 (x86, Ubuntu 24.04 / gcc-13 / OpenCV 4.6)

Pinned to `NadavHHailo/ORB_SLAM3` tag `v1.0-release` (`0df83dd`), built natively.
Patches live on branch `vio-eval-build`. Everything below is reproducible from a
fresh checkout of the submodule.

## Host prerequisites (already present on this machine)
gcc 13.3, cmake 3.28, OpenCV 4.6.0, Eigen 3.4.0, Boost serialization/system 1.83,
libgl1-mesa-dev, libglew-dev, libx11-dev (+ usual X11 dev libs). **No sudo/apt was
needed** — Pangolin builds against the present GLEW/X11.

## 1. Pangolin v0.6 → local prefix `~/opt/pangolin`
ORB-SLAM3 needs Pangolin to link (the viewer is compiled in even when disabled).
v0.6 uses the GLEW+X11 backend, so no `libepoxy`/`wayland-protocols` (which would
need apt). Two patches were required for gcc-13 / FFmpeg-6:

```bash
git clone --branch v0.6 --depth 1 https://github.com/stevenlovegrove/Pangolin.git ~/opt/src/Pangolin
cd ~/opt/src/Pangolin
# (a) gcc-13 no longer transitively includes <cstdint> -> add it to platform.h:
#     include/pangolin/platform.h:  #include <cstdint>   (right after the #pragma once)
cmake -B build -DCMAKE_INSTALL_PREFIX=$HOME/opt/pangolin -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF        # (b) FFmpeg 6.x dropped avpicture_*/av_register_all
cmake --build build -j"$(nproc)" --target install
```
Produces `~/opt/pangolin/lib/libpangolin.so` + `lib/cmake/Pangolin/`.

## 2. ORB-SLAM3 patches (branch `vio-eval-build`, off `v1.0-release`)
v1.0 forces `-std=c++11`, which fails on gcc-13 + Eigen 3.4 + vendored g2o/Sophus.
- **`CMakeLists.txt`** — replace the `c++11/c++0x` flag block with
  `set(CMAKE_CXX_STANDARD 14)` (+ `STANDARD_REQUIRED ON`); keep
  `add_definitions(-DCOMPILEDWITHC11)` (source gates its `std::chrono` timing path on it).
- **`Thirdparty/Sophus/CMakeLists.txt`** — `CMAKE_CXX_STANDARD 11 → 14`; drop `-Werror`
  and the explicit `-std=c++11` from the GNU flags (gcc-13 raises new warnings).
- DBoW2 / g2o / OpenCV: **no patches** — they compile on gcc-13's default standard and
  v1.0 already targets OpenCV 4.

## 3. Build
```bash
cd systems/orb_slam3
( cd Thirdparty/DBoW2 && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j )
( cd Thirdparty/g2o   && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j )
( cd Thirdparty/Sophus&& cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j )
( cd Vocabulary && tar -xf ORBvoc.txt.tar.gz )
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/opt/pangolin
cmake --build build -j8
```
Artifacts: `lib/libORB_SLAM3.so`, `Examples/Stereo-Inertial/stereo_inertial_euroc`.

## 4. Running (headless)
- The example already calls `System(..., bUseViewer=false)` → no `DISPLAY` needed.
- It reads the EuRoC **ASL** folder (`mav0/cam{0,1}/data`, `mav0/imu0/data.csv`) and a
  timestamps file (one ns-timestamp per line). The shipped `EuRoC_TimeStamps/*.txt` use
  float-rounded ns that do NOT match our bag-derived PNG names, so `run.sh` regenerates
  the timestamps from `~/datasets/euroc-asl/<seq>/mav0/cam0/data`.
- Output `f_<seq>.txt` is TUM-ordered but timestamped in **nanoseconds**;
  `scripts/adapters/orb_slam3_to_tum.py` divides by 1e9 → canonical seconds.
- Set `LD_LIBRARY_PATH` to include `~/opt/pangolin/lib` and the ORB-SLAM3/Thirdparty libs.

See [`scripts/run_orb_slam3.sh`](../scripts/run_orb_slam3.sh) for the full invocation.

### Run modes
- **Sequential** (default): frames fed back-to-back (DR clean-accuracy protocol). `--realtime` restores real-time pacing.
- **VIO-only** (`--vio-only`): disables ORB-SLAM3's loop-closure / global-BA stage so it behaves as pure sliding-window VIO — comparable to OpenVINS/Basalt/SchurVINS. No source patch: `System.cc` reads a `loopClosing` YAML key, so the runner appends `loopClosing: 0` to a temp copy of `EuRoC.yaml`. Results land under a separate `<tag>_vioonly/` dir. On V1_01_easy the gap is negligible (0.019 vs 0.020 m — short sequence, little revisit); it grows on loop-heavy sequences.

## Validation (V1_01_easy, stereo-inertial)
ATE pos RMSE (se3 alignment) = **0.020 m** (sim3 = 0.011 m), RPE ≈ 3 cm / 8 m — in line
with the ORB-SLAM3 paper. `posyaw` is NOT used: ORB-SLAM3's output frame differs from the
EuRoC GT by a full 3D rotation, so the harness aligns all systems with `se3`.
