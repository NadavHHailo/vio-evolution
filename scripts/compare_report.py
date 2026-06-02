#!/usr/bin/env python3
"""Cross-system comparison report aligned to the Evaluation-DR (§3.1 metrics + §2.6 RPE-over-segments).

Reports, per (system, sequence), aggregated over reps with one alignment mode for
all systems (se3 default):

  §3.1 summary : ATE-trans (m), ATE-rot (deg), RPE-trans/rot (mean over segments),
                 completeness (%), track-loss, latency p50/p99 (ms), FPS, CPU (%), peak RSS (MB)
  §2.6 detail  : RPE-translation (m) and RPE-rotation (deg) per segment length
                 (8/16/24/32/40 m), the standard VIO drift-rate-over-distance view.

Reuses catkin_ws_ov/scripts/parse_results.py (_run_ros2, _est_to_tum, _ANSI_RE, _RPE_RE)
for the ov_eval plumbing. ORB-SLAM3 is reported in two variants: (SLAM) = full pipeline,
(VIO-only) = loopClosing:0. x86 performance figures are illustrative (DR: perf profiling
belongs on embedded HW).

Usage:
  compare_report.py [--root ~/results] [--tag baseline_x86] [--align se3]
                    [--seqs V1_01_easy,...] [--openvins-est PATH] [--out report.md]
"""
import argparse
import glob
import os
import statistics
import sys
from collections import defaultdict

CATKIN_SCRIPTS = os.path.expanduser("~/workspace/catkin_ws_ov/scripts")
sys.path.insert(0, CATKIN_SCRIPTS)
import parse_results as pr  # noqa: E402

GT_DIR = os.path.expanduser("~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav")
ORB_DIR = "orb_slam3/x86/native_jazzy"


# ─── per-rep metric extractors ───
def run_eval(gt, tum, align):
    """ov_eval error_singlerun → {ate_ori, ate_pos, rpe:{seg:{ori,pos}}} (rpe per segment length)."""
    res = pr._run_ros2(["ros2", "run", "ov_eval", "error_singlerun", align, gt, tum], timeout=40)
    ate_ori = ate_pos = None
    rpe = {}
    for line in res.stdout.splitlines():
        clean = pr._ANSI_RE.sub("", line).strip()
        if clean.startswith("rmse_ori"):
            try:
                p = clean.split("|")
                ate_ori, ate_pos = float(p[0].split("=")[1]), float(p[1].split("=")[1])
            except (ValueError, IndexError):
                pass
            continue
        m = pr._RPE_RE.match(clean)
        if m:
            rpe[int(m.group(1))] = {"ori": float(m.group(2)), "pos": float(m.group(3))}
    return {"ate_ori": ate_ori, "ate_pos": ate_pos, "rpe": rpe}


def timing_stats(timing_csv):
    vals = []
    try:
        with open(timing_csv) as f:
            next(f)
            for line in f:
                c = line.split(",")
                if len(c) >= 2 and c[1] not in ("nan", ""):
                    try:
                        vals.append(float(c[1]))
                    except ValueError:
                        pass
    except OSError:
        return None
    if not vals:
        return None
    return pr.percentile(vals, 50), pr.percentile(vals, 99), 1000.0 / statistics.mean(vals)


_FRAME_CACHE = {}


def euroc_frames(seq):
    """Canonical input-frame timestamps (s) for a sequence — the cam0 frames of the
    shared EuRoC recording (same for every system). Cached per sequence."""
    if seq not in _FRAME_CACHE:
        d = os.path.expanduser(f"~/datasets/euroc-asl/{seq}/mav0/cam0/data")
        ts = sorted(int(os.path.splitext(f)[0]) / 1e9 for f in os.listdir(d)
                    if f.endswith(".png")) if os.path.isdir(d) else []
        _FRAME_CACHE[seq] = ts
    return _FRAME_CACHE[seq]


def robustness_stats(traj, frames):
    """Return {compl, compl_post, init} given a trajectory file (first column = timestamp,
    works for TUM and OpenVINS state-dump) and the canonical input-frame timestamps:
      compl       : poses / all input frames (%)            — DR completeness (#5)
      compl_post  : poses / frames at-or-after the first pose (%) — tracking continuity once
                    initialized (excludes the VI-init warm-up)
      init        : seconds from first input frame to first output pose — DR init-time (#11)
    """
    try:
        poses = [float(ln.split()[0]) for ln in open(traj)
                 if ln.strip() and not ln.startswith("#")]
    except OSError:
        return None
    if not poses or not frames:
        return None
    first_pose, first_frame = poses[0], frames[0]
    n_after = sum(1 for f in frames if f >= first_pose - 1e-6)
    return {"compl": 100.0 * len(poses) / len(frames),
            "compl_post": (100.0 * len(poses) / n_after) if n_after else None,
            "init": max(0.0, first_pose - first_frame)}


def track_loss(stdout_log):
    try:
        n = sum(1 for ln in open(stdout_log) if "Creation of new map with id:" in ln)
        return max(0, n - 1)
    except OSError:
        return None


def proc_stats(proc_csv):
    try:
        lines = open(proc_csv).read().splitlines()
        h, r = lines[0].split(","), lines[1].split(",")
        return float(r[h.index("peak_rss_kb")]) / 1024.0, float(r[h.index("pct_cpu")])
    except (OSError, IndexError, ValueError):
        return None


def openvins_timing(wall_csv):
    """(p50_ms, p99_ms, fps) from OpenVINS _wall.txt 'total' column (seconds)."""
    secs = pr.parse_timing_csv(wall_csv)  # total column, seconds
    if not secs:
        return None
    ms_ = [s * 1000.0 for s in secs]
    return pr.percentile(ms_, 50), pr.percentile(ms_, 99), 1000.0 / statistics.mean(ms_)


def openvins_cpu_pct(wall_csv, cpu_csv):
    """Mean CPU as % of one core = total CPU-seconds / total wall-seconds * 100."""
    wall, cpu = pr.parse_timing_csv(wall_csv), pr.parse_timing_csv(cpu_csv)
    if not wall or not cpu:
        return None
    return 100.0 * sum(cpu) / sum(wall)


# ─── aggregation helpers ───
def ms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return statistics.mean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0)


def cell(agg, prec=3):
    return f"{agg[0]:.{prec}f} ± {agg[1]:.{prec}f}" if agg else "—"


def num(v, prec=1):
    return f"{v:.{prec}f}" if v is not None else "—"


def new_metrics():
    return {k: [] for k in ("ate_ori", "ate_pos", "compl", "compl_post", "init", "loss",
                            "p50", "p99", "fps", "cpu", "rss")} | \
           {"rpe_pos_seg": defaultdict(list), "rpe_ori_seg": defaultdict(list)}


def add_eval(M, e):
    M["ate_ori"].append(e["ate_ori"]); M["ate_pos"].append(e["ate_pos"])
    for seg, v in e["rpe"].items():
        M["rpe_pos_seg"][seg].append(v["pos"])
        M["rpe_ori_seg"][seg].append(v["ori"])


# ─── per-system evaluation ───
def eval_orb(root, tag, seq, gt, align):
    base = f"{root}/{ORB_DIR}/{tag}"
    trajs = sorted(glob.glob(f"{base}/{seq}_rep*_trajectory.txt"))
    frames = euroc_frames(seq)
    M = new_metrics()
    for t in trajs:
        add_eval(M, run_eval(gt, t, align))
        rb = robustness_stats(t, frames)
        if rb:
            M["compl"].append(rb["compl"]); M["compl_post"].append(rb["compl_post"])
            M["init"].append(rb["init"])
        M["loss"].append(track_loss(t.replace("_trajectory.txt", "_stdout.log")))
        ts = timing_stats(t.replace("_trajectory.txt", "_timing.csv"))
        if ts:
            M["p50"].append(ts[0]); M["p99"].append(ts[1]); M["fps"].append(ts[2])
        ps = proc_stats(t.replace("_trajectory.txt", "_proc.csv"))
        if ps:
            M["rss"].append(ps[0]); M["cpu"].append(ps[1])
    return M, len(trajs)


def eval_openvins(root, tag, seq, gt, align, thr):
    """Read OpenVINS serial-mode outputs under <tag>/serial/<seq>_<thr>thr_*.
    Accuracy from _est.txt; latency/FPS from _wall.txt; CPU from _cpu.txt;
    completeness/init from _est.txt vs the canonical EuRoC frames. (RSS is not
    captured by the OpenVINS harness → left blank.)"""
    sdir = f"{root}/x86/native_jazzy/{tag}/serial"
    est = f"{sdir}/{seq}_{thr}thr_est.txt"
    if not os.path.exists(est):  # fall back to whatever thread count exists
        cands = sorted(glob.glob(f"{sdir}/{seq}_*thr_est.txt"))
        if not cands:
            return new_metrics(), 0
        est = cands[0]
    M = new_metrics()
    tum = pr._est_to_tum(est)
    try:
        add_eval(M, run_eval(gt, tum, align))
    finally:
        if tum and os.path.exists(tum):
            os.remove(tum)
    rb = robustness_stats(est, euroc_frames(seq))
    if rb:
        M["compl"].append(rb["compl"]); M["compl_post"].append(rb["compl_post"])
        M["init"].append(rb["init"])
    wall, cpu = est.replace("_est.txt", "_wall.txt"), est.replace("_est.txt", "_cpu.txt")
    ts = openvins_timing(wall)
    if ts:
        M["p50"].append(ts[0]); M["p99"].append(ts[1]); M["fps"].append(ts[2])
    c = openvins_cpu_pct(wall, cpu)
    if c is not None:
        M["cpu"].append(c)
    return M, 1


# ─── rendering ───
def flat_mean(seg_dict):
    allv = [v for vals in seg_dict.values() for v in vals if v is not None]
    return (statistics.mean(allv), statistics.stdev(allv) if len(allv) > 1 else 0.0) if allv else None


def summary_row(label, seq, M, n):
    p50, p99 = ms(M["p50"]), ms(M["p99"])
    lat = f"{p50[0]:.1f}/{p99[0]:.1f}" if p50 and p99 else "—"
    fps, cpu, rss = ms(M["fps"]), ms(M["cpu"]), ms(M["rss"])
    compl, compl_post = ms(M["compl"]), ms(M["compl_post"])
    init, loss = ms(M["init"]), ms(M["loss"])
    return "| " + " | ".join([
        label, seq,
        cell(ms(M["ate_pos"]), 3), cell(ms(M["ate_ori"]), 2),
        cell(flat_mean(M["rpe_pos_seg"]), 3), cell(flat_mean(M["rpe_ori_seg"]), 2),
        num(compl[0] if compl else None, 1), num(compl_post[0] if compl_post else None, 1),
        num(init[0] if init else None, 2), num(loss[0] if loss else None, 1),
        lat, num(fps[0] if fps else None, 1),
        num(cpu[0] if cpu else None, 0), num(rss[0] if rss else None, 0), str(n),
    ]) + " |"


def seg_row(label, seq, seg_dict, segments, prec):
    cells = [label, seq]
    for s in segments:
        cells.append(num(ms(seg_dict.get(s, []))[0] if ms(seg_dict.get(s, [])) else None, prec)
                     if seg_dict.get(s) else "—")
    return "| " + " | ".join(cells) + " |"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.expanduser("~/results"))
    ap.add_argument("--tag", default="baseline_x86")
    ap.add_argument("--align", default="se3")
    ap.add_argument("--seqs", default="V1_01_easy,MH_03_medium,V2_02_medium")
    ap.add_argument("--openvins-thr", default="1",
                    help="OpenVINS serial thread count to report (default 1 = single-thread)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    # collect every (label, seq, M, n)
    rows = []
    for seq in args.seqs.split(","):
        gt = f"{GT_DIR}/{seq}.txt"
        ovM, ovn = eval_openvins(args.root, args.tag, seq, gt, args.align, args.openvins_thr)
        rows.append(("openvins", seq, ovM, ovn))
        orbM, orbn = eval_orb(args.root, args.tag, seq, gt, args.align)
        rows.append(("orb_slam3 (SLAM)", seq, orbM, orbn))
        vioM, vion = eval_orb(args.root, f"{args.tag}_vioonly", seq, gt, args.align)
        if vion > 0:
            rows.append(("orb_slam3 (VIO-only)", seq, vioM, vion))

    # union of segment lengths actually reported by ov_eval
    segments = sorted({s for _, _, M, _ in rows for s in M["rpe_pos_seg"]})
    seg_hdr = " | ".join(f"{s} m" for s in segments)

    L = [
        f"# VIO comparison — Evaluation-DR metrics (align={args.align}, tag={args.tag})",
        "",
        "Aggregated mean ± std over reps. Accuracy via `ov_eval error_singlerun`; "
        "latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`. ORB-SLAM3 runs "
        "in **sequential** mode, reported as **(SLAM)** (loop closure on) and **(VIO-only)** "
        "(`loopClosing:0`). **x86 performance figures are illustrative** (DR: perf belongs on "
        "embedded HW); ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect the "
        "per-frame tracking front-end. **OpenVINS runs in serial mode, single-thread (1thr)**; "
        "its latency/FPS use the per-frame `total` update time, and RSS is left blank (not "
        "captured by the OpenVINS benchmark harness).",
        "",
        "## §3.1 Summary (RPE columns = mean over segment lengths)",
        "",
        "*Compl %* = poses ÷ all input frames; *Compl(p-i) %* = poses ÷ frames after the first "
        "pose (tracking continuity, excludes the VI-init warm-up); *Init (s)* = time to first pose.",
        "",
        "| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | Compl(p-i) % | "
        "Init (s) | Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    L += [summary_row(lbl, seq, M, n) for lbl, seq, M, n in rows]

    # §2.6 RPE over segment lengths — translation
    L += [
        "",
        "## §2.6 RPE over segment lengths — translation (m)",
        "",
        "Local drift accumulated over fixed sub-trajectory lengths (the standard VIO "
        "drift-rate-over-distance view). Each cell is the mean over reps of `ov_eval`'s "
        "per-segment median translation error.",
        "",
        f"| System | Seq | {seg_hdr} |",
        "|---|---|" + "---|" * len(segments),
    ]
    L += [seg_row(lbl, seq, M["rpe_pos_seg"], segments, 3) for lbl, seq, M, n in rows]

    # §2.6 RPE over segment lengths — rotation
    L += [
        "",
        "## §2.6 RPE over segment lengths — rotation (°)",
        "",
        f"| System | Seq | {seg_hdr} |",
        "|---|---|" + "---|" * len(segments),
    ]
    L += [seg_row(lbl, seq, M["rpe_ori_seg"], segments, 2) for lbl, seq, M, n in rows]

    report = "\n".join(L)
    print(report)
    if args.out:
        with open(args.out, "w") as f:
            f.write(report + "\n")
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
