# -*- coding: utf-8 -*-
"""Cleaning rules for the raw annotated detachment points.

Both data_preprocess.__main__ (which writes the training data
data/processed/dettach*_clean.csv) and plot_action_space_data.load_dettach (which draws
the annotated points) go through here, so the points the network is trained on and the
points shown in the figures are always the same set. The raw exports
data/dettach-*.xlsx are never modified; the uncorrected data/processed/dettach*.csv are
kept as produced, and the excluded versions are written to dettach*_clean.csv.

Rules (input must already have DISC folded from [0,180] into [-90,90]; the DISC < 10
threshold below refers to that folded quantity):
1) Single-frame HEIGHT outliers (all trials): |delta H| to both neighbours exceeds 5 cm
   while DISC and NECK change by less than 3 degrees per frame. Across the detachment
   data this is met once, at index 25 of trial 3 (H 8.21 -> 0.35 -> 9.11 with
   delta DISC and delta NECK around 1 degree): a corrupted frame rather than motion.
2) Wall-hugging prefix of trial 1: all points with DISC < 10 degrees are dropped -- the
   contiguous prefix of 22 points at indices 0-21. The comparable prefixes in trials 2
   and 3 (8 and 17 points) are kept.
"""

import numpy as np

HEIGHT_SPIKE_CM = 5.0   # rule 1: single-frame HEIGHT outlier threshold (both neighbours)
ANGLE_SPIKE_DEG = 3.0   # rule 1: frame-to-frame DISC/NECK change must be below this
D1_MIN_DISC_DEG = 10.0  # rule 2: DISC cut-off for the trial-1 wall-hugging prefix

# Guard: expected number of hits per trial, as (height outliers, trial-1 DISC<10 points).
# If the raw data changes, fail loudly rather than silently altering what is excluded.
EXPECTED_HITS = {1: (0, 22), 2: (0, 0), 3: (1, 0)}


def _height_spike_keep(height, disc, neck):
    """Rule 1: keep mask with single-frame HEIGHT outliers set False (ends excluded)."""
    n = len(height)
    keep = np.ones(n, bool)
    if n < 3:
        return keep
    dh = np.abs(np.diff(height))
    dd = np.abs(np.diff(disc))
    dn = np.abs(np.diff(neck))
    for i in range(1, n - 1):
        if (dh[i - 1] > HEIGHT_SPIKE_CM and dh[i] > HEIGHT_SPIKE_CM
                and dd[i - 1] < ANGLE_SPIKE_DEG and dd[i] < ANGLE_SPIKE_DEG
                and dn[i - 1] < ANGLE_SPIKE_DEG and dn[i] < ANGLE_SPIKE_DEG):
            keep[i] = False
    return keep


def apply_dettach_exclusion(trial_no, height, disc, neck):
    """Apply the exclusion rules to one trial, returning the kept (height, disc, neck).

    Raises if the number of hits differs from EXPECTED_HITS.
    """
    height = np.asarray(height, dtype=float)
    disc = np.asarray(disc, dtype=float)
    neck = np.asarray(neck, dtype=float)

    spike_keep = _height_spike_keep(height, disc, neck)
    n_spike = int((~spike_keep).sum())

    disc_keep = np.ones(len(height), bool)
    if trial_no == 1:
        disc_keep = disc >= D1_MIN_DISC_DEG
    n_disc = int(len(height) - disc_keep.sum()) if trial_no == 1 else 0

    exp_spike, exp_disc = EXPECTED_HITS.get(trial_no, (0, 0))
    if n_spike != exp_spike or n_disc != exp_disc:
        raise AssertionError(
            f"dettach{trial_no}: {n_spike} height outliers and {n_disc} DISC<"
            f"{D1_MIN_DISC_DEG:g} points, expected ({exp_spike}, {exp_disc}) -- the raw "
            f"data may have changed; check the rules in this module")

    keep = spike_keep & disc_keep
    print(f"dettach{trial_no}_clean: {len(height)} -> {int(keep.sum())} points "
          f"({n_spike} height outliers and {n_disc} DISC<{D1_MIN_DISC_DEG:g} excluded)")
    return height[keep], disc[keep], neck[keep]
