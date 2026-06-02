#!/usr/bin/env python3
"""Cross-system comparison report aligned to the Evaluation-DR §3.1 metric table.

Reports, per (system, sequence), aggregated over reps with one alignment mode for
all systems (se3 default):

  Accuracy   : ATE-trans (m), ATE-rot (deg), RPE-trans (m/seg), RPE-rot (deg/seg)
  Robustness : trajectory completeness (%), track-loss / re-init count
  Performance: latency p50/p99 (ms/frame), throughput (FPS), CPU (%), peak RSS (MB)

Reuses catkin_ws_ov/scripts/parse_results.py (_run_ros2, _est_to_tum, _ANSI_RE,
_RPE_RE) for the ov_eval plumbing. Accuracy uses ov_eval error_singlerun; timing
comes from the per-rep <seq>_timing.csv (ORB-SLAM3) and resource from <seq>_proc.csv.

NOTE: x86 performance numbers are illustrative (the DR calls perf profiling on
desktop CPUs "un-useful" — embedded HW is the real target). Accuracy is the headline.

Usage:
  compare_report.py [--root ~/results] [--tag baseline_x86] [--align se3]
                    [--seqs V1_01_easy,...] [--openvins-est PATH] [--out report.md]
"""
import argparse
import glob
import os
import statistics
import sys

CATKIN_SCRIPTS = os.path.expanduser("~/workspace/catkin_ws_ov/scripts")
sys.path.insert(0, CATKIN_SCRIPTS)
import parse_results as pr  # noqa: E402

GT_DIR = os.path.expanduser("~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav")
ORB_DIR = "orb_slam3/x86/native_jazzy"


# ─── per-rep metric extractors ───
def run_eval(gt, tum, align):
    """ov_eval error_singlerun → {ate_ori, ate_pos, rpe_ori, rpe_pos} (RPE = mean over segments)."""
    res = pr._run_ros2(["ros2", "run", "ov_eval", "error_singlerun", align, gt, tum], timeout=40)
    ate_ori = ate_pos = None
    rpe_ori, rpe_pos = [], []
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
            rpe_ori.append(float(m.group(2)))
            rpe_pos.append(float(m.group(3)))
    return {"ate_ori": ate_ori, "ate_pos": ate_pos,
            "rpe_ori": statistics.mean(rpe_ori) if rpe_ori else None,
            "rpe_pos": statistics.mean(rpe_pos) if rpe_pos else None}


def timing_stats(timing_csv):
    """(p50_ms, p99_ms, fps) from the frontend column; fps = 1000/mean."""
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
    return (pr.percentile(vals, 50), pr.percentile(vals, 99),
            1000.0 / statistics.mean(vals))


def completeness_pct(traj, times_file):
    try:
        npose = sum(1 for ln in open(traj) if ln.strip() and not ln.startswith("#"))
        nframe = sum(1 for ln in open(times_file) if ln.strip())
        return 100.0 * npose / nframe if nframe else None
    except OSError:
        return None


def track_loss(stdout_log):
    """ORB-SLAM3 creates a new map (id>0) on track loss; the first map (id 0) is normal init."""
    try:
        n = sum(1 for ln in open(stdout_log) if "Creation of new map with id:" in ln)
        return max(0, n - 1)
    except OSError:
        return None


def proc_stats(proc_csv):
    """(peak_rss_mb, cpu_pct) from /usr/bin/time -v derived CSV."""
    try:
        h, r = (open(proc_csv).read().splitlines())[0].split(","), \
               (open(proc_csv).read().splitlines())[1].split(",")
        rss = float(r[h.index("peak_rss_kb")]) / 1024.0
        cpu = float(r[h.index("pct_cpu")])
        return rss, cpu
    except (OSError, IndexError, ValueError):
        return None


# ─── aggregation helpers ───
def ms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return (statistics.mean(vals), statistics.stdev(vals) if len(vals) > 1 else 0.0)


def cell(agg, prec=3):
    return f"{agg[0]:.{prec}f} ± {agg[1]:.{prec}f}" if agg else "—"


def num(v, prec=1):
    return f"{v:.{prec}f}" if v is not None else "—"


# ─── per-system evaluation ───
def eval_orb(root, tag, seq, gt, align):
    base = f"{root}/{ORB_DIR}/{tag}"
    trajs = sorted(glob.glob(f"{base}/{seq}_rep*_trajectory.txt"))
    times_file = f"{base}/{seq}_times.txt"
    M = {k: [] for k in ("ate_ori", "ate_pos", "rpe_ori", "rpe_pos",
                          "compl", "loss", "p50", "p99", "fps", "cpu", "rss")}
    for t in trajs:
        e = run_eval(gt, t, align)
        for k in ("ate_ori", "ate_pos", "rpe_ori", "rpe_pos"):
            M[k].append(e[k])
        M["compl"].append(completeness_pct(t, times_file))
        M["loss"].append(track_loss(t.replace("_trajectory.txt", "_stdout.log")))
        ts = timing_stats(t.replace("_trajectory.txt", "_timing.csv"))
        if ts:
            M["p50"].append(ts[0]); M["p99"].append(ts[1]); M["fps"].append(ts[2])
        ps = proc_stats(t.replace("_trajectory.txt", "_proc.csv"))
        if ps:
            M["rss"].append(ps[0]); M["cpu"].append(ps[1])
    return M, len(trajs)


def eval_openvins(root, seq, gt, align, explicit):
    ests = [explicit] if explicit else sorted(
        glob.glob(f"{root}/**/subscribe/{seq}_*_est.txt", recursive=True))
    M = {k: [] for k in ("ate_ori", "ate_pos", "rpe_ori", "rpe_pos")}
    for e in ests:
        tum = pr._est_to_tum(e)
        try:
            ev = run_eval(gt, tum, align)
            for k in M:
                M[k].append(ev[k])
        finally:
            if tum and os.path.exists(tum):
                os.remove(tum)
    return M, len(ests)


def row(sys_, seq, M, n):
    compl = ms(M.get("compl", []))
    loss = ms(M.get("loss", []))
    p50 = ms(M.get("p50", [])); p99 = ms(M.get("p99", []))
    fps = ms(M.get("fps", [])); cpu = ms(M.get("cpu", [])); rss = ms(M.get("rss", []))
    lat = (f"{p50[0]:.1f}/{p99[0]:.1f}" if p50 and p99 else "—")
    return "| " + " | ".join([
        sys_, seq,
        cell(ms(M["ate_pos"]), 3), cell(ms(M["ate_ori"]), 2),
        cell(ms(M["rpe_pos"]), 3), cell(ms(M["rpe_ori"]), 2),
        num(compl[0] if compl else None, 1),
        num(loss[0] if loss else None, 1),
        lat,
        num(fps[0] if fps else None, 1),
        num(cpu[0] if cpu else None, 0),
        num(rss[0] if rss else None, 0),
        str(n),
    ]) + " |"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.expanduser("~/results"))
    ap.add_argument("--tag", default="baseline_x86")
    ap.add_argument("--align", default="se3")
    ap.add_argument("--seqs", default="V1_01_easy,MH_03_medium,V2_02_medium")
    ap.add_argument("--openvins-est", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    lines = [
        f"# VIO comparison — Evaluation-DR §3.1 metrics (align={args.align}, tag={args.tag})",
        "",
        "Aggregated mean ± std over reps. Accuracy via `ov_eval error_singlerun`; "
        "latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`.",
        "RPE is the mean of per-segment medians (8/16/24/32/40 m). ORB-SLAM3 runs in "
        "**sequential** mode and is reported in two variants: **(SLAM)** = full pipeline "
        "with loop closure / global BA, and **(VIO-only)** = `loopClosing: 0` (pure "
        "sliding-window VIO, comparable to OpenVINS/Basalt/SchurVINS). "
        "**x86 performance figures are illustrative** (DR: perf profiling belongs on "
        "embedded HW). ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect "
        "the per-frame tracking front-end.",
        "",
        "| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | "
        "Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for seq in args.seqs.split(","):
        gt = f"{GT_DIR}/{seq}.txt"
        ovM, ovn = eval_openvins(args.root, seq, gt, args.align, args.openvins_est)
        orbM, orbn = eval_orb(args.root, args.tag, seq, gt, args.align)
        vioM, vion = eval_orb(args.root, f"{args.tag}_vioonly", seq, gt, args.align)
        lines.append(row("openvins", seq, ovM, ovn))
        lines.append(row("orb_slam3 (SLAM)", seq, orbM, orbn))
        if vion > 0:
            lines.append(row("orb_slam3 (VIO-only)", seq, vioM, vion))

    report = "\n".join(lines)
    print(report)
    if args.out:
        with open(args.out, "w") as f:
            f.write(report + "\n")
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
