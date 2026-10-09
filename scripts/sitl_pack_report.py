"""Rank the candidates of a scripts/sitl_pack.py run against a reference candidate (default "current").

  python scripts/sitl_pack_report.py results/sitl_pack/X [--ref current]

Each metric is averaged over a candidate's flights and divided by the reference's average (1.0 = same, below 1 =
better); the score is the weighted mean of those ratios, with a penalty for any flight that did not pass or did not
put the feet down before lowering the nose. Lower is better.
"""
import argparse, json, math
from collections import defaultdict
from pathlib import Path

# metric: (weight, label). All "lower is better".
METRICS = {
    "jolt_max": (5.0, "worst jolt deg/s"),
    "drift_total": (4.0, "drift from lift-off m"),
    "climb_q_rms": (4.0, "climb pitch-rate osc rms deg/s"),
    "handover_q_rms": (2.0, "handover pitch-rate osc rms deg/s"),
    "hover_alt_std": (2.0, "hover height std m"),
    "hover_drift": (2.0, "hover drift m"),
    "hover_pitch_osc_deg": (1.5, "hover pitch osc deg"),
    "hover_roll_osc_deg": (1.0, "hover roll osc deg"),
    "hover_rates_rms": (1.0, "hover rates rms deg/s"),
    "hover_cmd_jitter": (0.5, "motor cmd jitter"),
    "vib_gyro_hover": (1.0, "gyro vibration hover"),
    "vib_acc_hover": (0.5, "accel vibration hover"),
    "lift_rate_dev": (1.0, "lift rate dev deg/s"),
    "lift_jerk_rms": (1.0, "lift jerk deg/s2"),
    "td_vz": (2.0, "touchdown vz m/s"),
    "td_vh": (1.0, "touchdown vh m/s"),
    "lower_after_td_s": (0.5, "wait before lowering s"),
    "nose_drop_before_lower": (3.0, "nose fall before lowering deg"),
    "lower_rate_dev": (1.5, "lowering rate dev deg/s"),
    "lower_jerk_rms": (1.5, "lowering jerk deg/s2"),
    "front_touch_rate": (1.5, "front leg touch rate deg/s"),
}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--ref", default="current")
    ap.add_argument("--md", help="also write a Markdown table here")
    a = ap.parse_args()
    s = json.load(open(Path(a.dir) / "summary.json"))
    by = defaultdict(list)
    for r in s["results"]:
        by[r["cand"]].append(r)

    def avg(rs, k):
        v = [r[k] for r in rs if isinstance(r.get(k), (int, float)) and math.isfinite(r[k])]
        return sum(v) / len(v) if v else None

    ref = {k: avg(by[a.ref], k) for k in METRICS}
    rows = []
    for cand, rs in by.items():
        ok = sum(r["cls"] == "PASS" for r in rs); legs = sum(bool(r.get("legs_first")) for r in rs)
        ratios, wsum = 0.0, 0.0
        vals = {}
        for k, (w, _) in METRICS.items():
            v = avg(rs, k); vals[k] = v
            if v is None or not ref.get(k):
                continue
            ratios += w * (v / ref[k]); wsum += w
        score = ratios / wsum if wsum else float("inf")
        score += 1.0 * (len(rs) - ok) + 0.5 * (len(rs) - legs)
        rows.append((score, cand, ok, legs, len(rs), vals))
    rows.sort()
    keys = list(METRICS)
    head = ["score", "candidate", "pass", "legs first"] + [METRICS[k][1] for k in keys]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for score, cand, ok, legs, n, vals in rows:
        cells = [f"{score:.3f}", cand, f"{ok}/{n}", f"{legs}/{n}"] + ["-" if vals[k] is None else f"{vals[k]:.3g}" for k in keys]
        lines.append("| " + " | ".join(cells) + " |")
    print("\n".join(lines))
    print(f"\n{s['flights']} flights, {s['wall_s']} s wall on {len(s['instances'])} instances "
          f"({60 * s['flights'] / max(s['wall_s'], 1):.1f} flights/min)")
    if a.md:
        Path(a.md).write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
