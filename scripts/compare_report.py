#!/usr/bin/env python3
"""Cross-system comparison report (Phase 1: OpenVINS vs ORB-SLAM3).

Aggregates ATE across systems on the same EuRoC GT, using ONE alignment mode for
all systems (se3 by default) so the table is apples-to-apples. Reuses the existing
OpenVINS eval machinery in catkin_ws_ov/scripts/parse_results.py (`_run_ros2`,
`_est_to_tum`, `_ANSI_RE`) rather than reimplementing the ov_eval plumbing.

Two on-disk result schemes (see docs/plan.md §"Output convention"):
  - OpenVINS : <root>/<arch>/<env>/<tag>/<mode>/<seq>_<N>thr[_runK]_est.txt  (state-dump)
               -> converted to TUM via _est_to_tum before eval.
  - new sys  : <root>/<system>/<arch>/<env>/<tag>/<seq>_rep<i>_trajectory.txt (TUM)
               -> fed to error_singlerun directly.

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


def ate(gt, traj_tum, align):
    """Return (rmse_ori_deg, rmse_pos_m) from ov_eval error_singlerun, or (None, None)."""
    res = pr._run_ros2(["ros2", "run", "ov_eval", "error_singlerun", align, gt, traj_tum],
                       timeout=40)
    for line in res.stdout.splitlines():
        clean = pr._ANSI_RE.sub("", line).strip()
        if clean.startswith("rmse_ori"):
            try:
                parts = clean.split("|")
                return float(parts[0].split("=")[1]), float(parts[1].split("=")[1])
            except (ValueError, IndexError):
                return None, None
    return None, None


def agg(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    m = statistics.mean(vals)
    s = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return m, s, len(vals)


def peak_rss_mb(proc_csv):
    try:
        with open(proc_csv) as f:
            lines = f.read().splitlines()
        hdr, row = lines[0].split(","), lines[1].split(",")
        return float(row[hdr.index("peak_rss_kb")]) / 1024.0
    except (OSError, IndexError, ValueError):
        return None


def median_frontend_ms(timing_csv):
    vals = []
    try:
        with open(timing_csv) as f:
            next(f)  # header
            for line in f:
                p = line.split(",")
                if len(p) >= 2 and p[1] != "nan":
                    try:
                        vals.append(float(p[1]))
                    except ValueError:
                        pass
    except OSError:
        return None
    return statistics.median(vals) if vals else None


def eval_orb(root, tag, seq, gt, align):
    trajs = sorted(glob.glob(f"{root}/orb_slam3/x86/native_jazzy/{tag}/{seq}_rep*_trajectory.txt"))
    pos, rss, fe = [], [], []
    for t in trajs:
        _, p = ate(gt, t, align)
        pos.append(p)
        r = peak_rss_mb(t.replace("_trajectory.txt", "_proc.csv"))
        if r:
            rss.append(r)
        m = median_frontend_ms(t.replace("_trajectory.txt", "_timing.csv"))
        if m:
            fe.append(m)
    return (agg(pos),
            (statistics.mean(rss) if rss else None),
            (statistics.mean(fe) if fe else None),
            len(trajs))


def eval_openvins(root, tag, seq, gt, align, explicit):
    ests = [explicit] if explicit else sorted(
        glob.glob(f"{root}/**/subscribe/{seq}_*_est.txt", recursive=True))
    pos = []
    for e in ests:
        tum = pr._est_to_tum(e)
        try:
            _, p = ate(gt, tum, align)
            pos.append(p)
        finally:
            if tum and os.path.exists(tum):
                os.remove(tum)
    return agg(pos), len(ests)


def fmt(a):
    return f"{a[0]:.3f} ± {a[1]:.3f}" if a else "—"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.expanduser("~/results"))
    ap.add_argument("--tag", default="baseline_x86")
    ap.add_argument("--align", default="se3")
    ap.add_argument("--seqs", default="V1_01_easy")
    ap.add_argument("--openvins-est", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rows = []
    for seq in args.seqs.split(","):
        gt = f"{GT_DIR}/{seq}.txt"
        ov, ov_n = eval_openvins(args.root, args.tag, seq, gt, args.align, args.openvins_est)
        orb, orb_rss, orb_fe, orb_n = eval_orb(args.root, args.tag, seq, gt, args.align)
        rows.append(("openvins", seq, ov, None, None, ov_n))
        rows.append(("orb_slam3", seq, orb, orb_rss, orb_fe, orb_n))

    lines = [
        f"# VIO comparison (Phase 1) — align={args.align}, tag={args.tag}",
        "",
        "Frontend ms/frame = median per-frame tracking latency. ORB-SLAM3's heavy backend "
        "(local BA) runs on a separate async thread, off the per-frame critical path, so it "
        "is not in this number.",
        "",
        "| System | Sequence | ATE pos RMSE (m) | Frontend ms/frame | Peak RSS (MB) | reps |",
        "|---|---|---|---|---|---|",
    ]
    for sys_, seq, a, rss, fe, n in rows:
        rss_cell = f"{rss:.0f}" if rss else "—"
        fe_cell = f"{fe:.1f}" if fe else "—"
        lines.append(f"| {sys_} | {seq} | {fmt(a)} | {fe_cell} | {rss_cell} | {n} |")
    report = "\n".join(lines)
    print(report)
    if args.out:
        with open(args.out, "w") as f:
            f.write(report + "\n")
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
