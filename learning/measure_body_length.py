# -*- coding: utf-8 -*-
"""Estimate the body length of the recorded individuals from the raw landmarks.

For every frame in data/raw/data231015.csv where all five landmarks (head1, head2,
body3, tail4, tail5) are valid, the chain length is the sum of the four consecutive
segments. The sum is insensitive to body curvature; the straight head1-to-tail5
distance is reported for reference only and underestimates the length on curved
frames. Units are the same as HEIGHT (cm).

Result: over the 41 frames with all five landmarks valid, the median chain length is
14.81 cm (mean 14.89 +/- 0.19, range [14.65, 15.38]). Reported in the manuscript as
approximately 14.8 cm.

Usage (run from the repository root):
    PYTHONIOENCODING=utf-8 python learning/measure_body_length.py
"""

import numpy as np
import pandas as pd

NAMES = ["head1", "head2", "body3", "tail4", "tail5"]


def main():
    df = pd.read_csv("data/raw/data231015.csv", skiprows=2, header=None)
    print(f"total frames: {len(df)}")
    pts = df.iloc[:, 2:17].astype(float).to_numpy().reshape(-1, 5, 3)

    # per-landmark validity (-1 marks an invalid landmark)
    for i, n in enumerate(NAMES):
        ok = ~np.any(pts[:, i, :] == -1, axis=1)
        print(f"{n:6s} valid frames: {ok.sum()}")

    all5 = ~np.any(pts.reshape(len(df), -1) == -1, axis=1)
    p = pts[all5]
    print(f"\nframes with all five landmarks valid: {len(p)}")

    def seg(a, b):
        return np.linalg.norm(p[:, a, :] - p[:, b, :], axis=1)

    segs = {
        "head1-head2 (head)": seg(0, 1),
        "head2-body3 (neck to trunk)": seg(1, 2),
        "body3-tail4 (trunk to tail)": seg(2, 3),
        "tail4-tail5 (tail)": seg(3, 4),
    }
    chain = sum(segs.values())
    straight = seg(0, 4)
    print("\nsegment length (cm): median [min, max]")
    for k, v in segs.items():
        print(f"  {k:30s} {np.median(v):6.2f}  [{v.min():5.2f}, {v.max():5.2f}]")
    print(f"  {'chain (sum of the four)':30s} {np.median(chain):6.2f}  [{chain.min():5.2f}, {chain.max():5.2f}]"
          f"  mean={chain.mean():.2f} std={chain.std():.2f}")
    print(f"  {'straight head1 to tail5':30s} {np.median(straight):6.2f}  [{straight.min():5.2f}, {straight.max():5.2f}]")
    print(f"\nheight range (head1.y, units cross-check): [{p[:, 0, 1].min():.2f}, {p[:, 0, 1].max():.2f}]")


if __name__ == "__main__":
    main()
