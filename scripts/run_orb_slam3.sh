#!/usr/bin/env bash
# Phase-1 runner for ORB-SLAM3 (stereo-inertial, EuRoC ASL).
# Lives in the outer repo (NOT inside the systems/orb_slam3 submodule).
#
# Produces, per rep, under ~/results/orb_slam3/<arch>/<env>/<tag>/ :
#   <seq>_rep<i>_trajectory.txt  canonical TUM (seconds) via orb_slam3_to_tum.py
#   <seq>_rep<i>_proc.csv        peak_rss_kb,user_s,sys_s,pct_cpu,wall_s  (/usr/bin/time -v)
#   <seq>_rep<i>_stdout.log      full ORB-SLAM3 output
#
# ORB-SLAM3 reads the EuRoC ASL folder (mav0/cam{0,1}/data + imu0/data.csv) and a
# timestamps file (one ns-timestamp per line). We generate that file from our own
# reconstructed cam0 frames so the PNG names always match (the shipped EuRoC_TimeStamps
# files use float-rounded ns that don't match our bag-derived names).
set -euo pipefail

usage() { echo "usage: run_orb_slam3.sh <seq> [--reps N] [--tag TAG]"; exit 1; }
[ $# -ge 1 ] || usage
SEQ="$1"; shift
REPS=1; TAG="baseline_x86"
while [ $# -gt 0 ]; do
  case "$1" in
    --reps) REPS="$2"; shift 2;;
    --tag)  TAG="$2";  shift 2;;
    *) usage;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # scripts/
REPO="$(cd "$HERE/.." && pwd)"                                # vio-evaluation
ORB="$REPO/systems/orb_slam3"                                 # submodule
BIN="$ORB/Examples/Stereo-Inertial/stereo_inertial_euroc"
VOC="$ORB/Vocabulary/ORBvoc.txt"
YAML="$ORB/Examples/Stereo-Inertial/EuRoC.yaml"
ASL="$HOME/datasets/euroc-asl/$SEQ"
GT="$HOME/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav/$SEQ.txt"
ADAPTER="$HERE/adapters/orb_slam3_to_tum.py"

for p in "$BIN" "$VOC" "$YAML" "$ASL/mav0/cam0/data" "$GT" "$ADAPTER"; do
  [ -e "$p" ] || { echo "ERROR: missing $p" >&2; exit 1; }
done

OUT="$HOME/results/orb_slam3/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
export LD_LIBRARY_PATH="$HOME/opt/pangolin/lib:$ORB/lib:$ORB/Thirdparty/DBoW2/lib:$ORB/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

TIMES="$OUT/${SEQ}_times.txt"
ls "$ASL/mav0/cam0/data/" | sed 's/\.png$//' | sort -n > "$TIMES"
echo "[$SEQ] $(wc -l < "$TIMES") frames; reps=$REPS; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  WORK="$(mktemp -d)"
  echo "[$SEQ] rep $i ..."
  ( cd "$WORK" && /usr/bin/time -v -o time.log \
      "$BIN" "$VOC" "$YAML" "$ASL" "$TIMES" "$SEQ" >stdout.log 2>&1 )
  python3 "$ADAPTER" "$WORK/f_$SEQ.txt" "$OUT/${SEQ}_rep${i}_trajectory.txt" "$GT"
  cp "$WORK/${SEQ}_timing.csv" "$OUT/${SEQ}_rep${i}_timing.csv"
  awk '
    /Maximum resident set size/ {rss=$NF}
    /User time/                 {usr=$NF}
    /System time/               {sys=$NF}
    /Percent of CPU/            {cpu=$NF; gsub(/%/,"",cpu)}
    /Elapsed .wall clock/       {split($NF,a,":"); wall=(length(a)==3)?a[1]*3600+a[2]*60+a[3]:a[1]*60+a[2]}
    END {printf "peak_rss_kb,user_s,sys_s,pct_cpu,wall_s\n%s,%s,%s,%s,%s\n",rss,usr,sys,cpu,wall}
  ' "$WORK/time.log" > "$OUT/${SEQ}_rep${i}_proc.csv"
  cp "$WORK/stdout.log" "$OUT/${SEQ}_rep${i}_stdout.log"
  rm -rf "$WORK"
done
echo "[$SEQ] done."
