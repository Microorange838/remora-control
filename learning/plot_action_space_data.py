# -*- coding: utf-8 -*-
"""Decision space drawn from the data alone: the (DISC, HEIGHT) -> NECK samples are
plotted without any model, as a reference against the network's own decision-space
figures. Where the model output extends past the data it is extrapolating, which the
side-by-side comparison makes visible.

Each behaviour produces two figures:
  1) figures/action_space_data_<group>.png
     two panels: the annotated measurement points with their trial trajectories, and
     the binned mean over the covered region
  2) figures/action_space_compare_<group>.png
     the network's decision-space figure next to the binned data mean (skipped if the
     network figure is absent)

Data sources (same angle definitions as data_preprocess.py):
  attachment: the raw motion-capture export data/raw/data231015.csv (acquired at 250 Hz,
     digitized every 10th frame -> 41 frames):
         NECK = arccos(unit(head1-head2) . unit(head2-body3))
         DISC = arcsin(unit(head1-head2)_y), HEIGHT = head1.y
     and the resampled sequence data/processed/attach23_01.csv (410 frames).
  detachment: the raw exports data/raw/dettach-1/2/3.xlsx (column A = HEIGHT,
     B = DISC folded from [0,180] into [-90,90], C = NECK), after the exclusions applied
     by data_cleaning, and the resampled data/processed/dettach1/2/3_clean.csv.

Usage (run from the repository root):
  MPLBACKEND=Agg python learning/plot_action_space_data.py
"""
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from data_cleaning import apply_dettach_exclusion

# same font and sizes as mlp26/config.py
plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["font.size"] = 20
plt.rcParams["axes.titlesize"] = 20
plt.rcParams["axes.labelsize"] = 20
plt.rcParams["xtick.labelsize"] = 20
plt.rcParams["ytick.labelsize"] = 20
plt.rcParams["legend.fontsize"] = 20

PICS = Path("figures")

# Decision-space panels are always square: HEIGHT (cm) and DISC (deg) are different
# units, and the square panel is the convention used in the manuscript.


def load_processed_csvs(paths):
    """Read the resampled 250 Hz CSVs (3 header rows) into concatenated arrays."""
    frames = [pd.read_csv(p, skiprows=2) for p in paths]
    df = pd.concat(frames, ignore_index=True)
    return dict(
        disc=df["DISC"].to_numpy(float),
        neck=df["NECK"].to_numpy(float),
        height=df["HEIGHT"].to_numpy(float),
    )


def load_attach23():
    """Raw motion capture -> the 41 annotated points (same formulas as data_preprocess)."""
    file = pd.read_csv("data/raw/data231015.csv").iloc[1:, :]
    file = file[~file["head1"].isin(["-1"])]
    h1 = file.iloc[:, 2:5].astype("float32").to_numpy()
    h2 = file.iloc[:, 5:8].astype("float32").to_numpy()
    b3 = file.iloc[:, 8:11].astype("float32").to_numpy()

    hv = h1 - h2
    hv = hv / np.linalg.norm(hv, axis=1, keepdims=True)
    bv = h2 - b3
    bv = bv / np.linalg.norm(bv, axis=1, keepdims=True)

    trial = dict(
        disc=np.degrees(np.arcsin(np.clip(hv[:, 1], -1, 1))),
        neck=np.degrees(np.arccos(np.clip(np.sum(hv * bv, axis=1), -1, 1))),
        height=h1[:, 1].astype(float),
    )
    interp = load_processed_csvs(["data/processed/attach23_01.csv"])
    return [trial], interp


def load_dettach():
    """Raw xlsx -> the annotated points of the three trials (same column mapping and
    filtering as the detachment branch of data_preprocess).

    The points pass through data_cleaning.apply_dettach_exclusion, the same exclusion
    rules that produce the training data dettach*_clean.csv (the wall-hugging prefix of
    trial 1 and a single-frame HEIGHT outlier in trial 3); the resampled samples are read
    from those same _clean.csv files, while the raw dettach*.csv and the xlsx are kept
    as produced.
    """
    trials = []
    for i in (1, 2, 3):
        f = pd.read_excel(f"data/raw/dettach-{i}.xlsx", header=None).iloc[1:, :]
        f = f[~f.iloc[:, 1].isin([-1])]
        f = f.dropna(subset=[f.columns[2]])
        disc = f.iloc[:, 1].astype("float32").to_numpy()
        disc = np.where(disc > 90, disc - 180, disc)
        neck = f.iloc[:, 2].astype("float32").to_numpy(float)
        height = f.iloc[:, 0].astype("float32").to_numpy(float)
        height, disc, neck = apply_dettach_exclusion(i, height, disc, neck)
        trials.append(dict(disc=disc, neck=neck, height=height))
    interp = load_processed_csvs([f"data/processed/dettach{i}_clean.csv" for i in (1, 2, 3)])
    return trials, interp


def draw_binned_mean(ax, disc, height, neck, extent, bins, count_levels,
                     vmin, vmax, title):
    """Binned mean of NECK as a filled colour map plus sample-count contours (empty cells
    are left blank). Axes are x=HEIGHT and y=DISC; extent=(h0,h1,d0,d1) and
    bins=(along HEIGHT, along DISC)."""
    rng = [[extent[0], extent[1]], [extent[2], extent[3]]]
    Hs, xe, ye = np.histogram2d(height, disc, bins=bins, range=rng, weights=neck)
    Hn, _, _ = np.histogram2d(height, disc, bins=bins, range=rng)
    mean = np.full_like(Hs, np.nan)
    np.divide(Hs, Hn, out=mean, where=Hn > 0)

    im = ax.pcolormesh(xe, ye, mean.T, cmap="RdYlBu_r", shading="auto",
                       vmin=vmin, vmax=vmax)
    ax.figure.colorbar(im, ax=ax, label="Measured NECK mean (deg)")

    xc = 0.5 * (xe[:-1] + xe[1:])
    yc = 0.5 * (ye[:-1] + ye[1:])
    cs = ax.contour(xc, yc, Hn.T, levels=count_levels, colors="black",
                    linewidths=1.2, linestyles="--")
    ax.clabel(cs, fmt="%d", fontsize=14)

    ax.set_xlabel("HEIGHT (cm)")
    ax.set_ylabel("DISC (deg)")
    ax.set_title(title)
    ax.set_box_aspect(1)  # square decision-space panel


def make_group_figure(group, trials, interp, extent, bins, count_levels, out_path,
                      figsize=(14, 6.2), legend_loc="upper right"):
    """Two panels: annotated points with their trajectories, and the binned mean with
    coverage contours.

    Both the scatter and the binned mean use the same colour map as the network's heat
    map (viz.plot_heatmap_neck), so the two sides of the comparison figure can be read
    against each other directly. legend_loc places the legend in an empty corner per
    group, so that it never covers a trajectory.
    """
    all_neck = np.concatenate([np.concatenate([t["neck"] for t in trials]),
                               interp["neck"]])
    vmin, vmax = float(all_neck.min()), float(all_neck.max())

    fig, axes = plt.subplots(1, 2, figsize=figsize, layout="compressed")

    # left: the annotated measurement points (one marker per trial, joined by trajectories)
    markers = ["o", "s", "^", "D"]
    for i, tr in enumerate(trials):
        axes[0].plot(tr["height"], tr["disc"], color="gray", lw=0.6, alpha=0.6)
        sc = axes[0].scatter(tr["height"], tr["disc"], c=tr["neck"],
                             s=55, cmap="RdYlBu_r", vmin=vmin, vmax=vmax,
                             marker=markers[i % len(markers)],
                             edgecolors="black", linewidths=0.4,
                             label=f"trial {i + 1} (n={len(tr['neck'])})")
    fig.colorbar(sc, ax=axes[0], label="NECK (deg)")
    axes[0].legend(loc=legend_loc, fontsize=20)
    axes[0].set_title("Annotated points")
    axes[0].set_xlabel("HEIGHT (cm)")
    axes[0].set_ylabel("DISC (deg)")
    axes[0].set_box_aspect(1)

    # right: the binned mean, as the data-side counterpart of the model heat map
    draw_binned_mean(axes[1], interp["disc"], interp["height"], interp["neck"],
                     extent, bins, count_levels, vmin, vmax,
                     "Binned mean")

    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"written: {out_path}")


def _trim_white(img, tol=0.97):
    """Trim the near-white border of a PNG so that the embedded network figure fills its
    panel to the same visual size as the natively drawn panel next to it."""
    mask = (img[..., :3] < tol).any(axis=2)
    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    if len(rows) == 0 or len(cols) == 0:
        return img
    return img[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1]


def make_compare_figure(group, interp, extent, bins, count_levels, vmin_max, out_path,
                        model_png, figsize=(15, 7.0), width_ratios=1.5):
    if not Path(model_png).exists():
        print(f"comparison figure skipped (missing {model_png})")
        return
    # the left width ratio follows the aspect ratio of the embedded PNG; aspect='equal'
    # scales it to fill the axes without stretching, so the side margins stay invisible
    fig, axes = plt.subplots(1, 2, figsize=figsize, layout="compressed",
                             gridspec_kw=dict(width_ratios=[width_ratios, 1]))
    axes[0].imshow(_trim_white(plt.imread(model_png)), aspect="equal")
    axes[0].axis("off")
    axes[0].set_title("Model: two-input network")
    draw_binned_mean(axes[1], interp["disc"], interp["height"], interp["neck"],
                     extent, bins, count_levels, vmin_max[0], vmin_max[1],
                     "Data: annotated-point mean")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"written: {out_path}")


def run_group(group, trials, interp, extent, bins, count_levels, model_png,
              group_figsize, compare_figsize, legend_loc, compare_width_ratios):
    all_neck = np.concatenate([np.concatenate([t["neck"] for t in trials]),
                               interp["neck"]])
    vmin, vmax = float(all_neck.min()), float(all_neck.max())
    make_group_figure(group, trials, interp, extent, bins, count_levels,
                      PICS / f"action_space_data_{group}.png", figsize=group_figsize,
                      legend_loc=legend_loc)
    make_compare_figure(group, interp, extent, bins, count_levels, (vmin, vmax),
                        PICS / f"action_space_compare_{group}.png", model_png,
                        figsize=compare_figsize, width_ratios=compare_width_ratios)


if __name__ == "__main__":
    PICS.mkdir(exist_ok=True)

    trials, interp = load_attach23()
    print(f"attach23: {sum(len(t['neck']) for t in trials)} annotated frames, "
          f"{len(interp['neck'])} resampled samples")
    # square panels, so all three groups share the same figsize; the legend goes to the
    # empty corner outside the arc (DISC<0 and H>6 for this group)
    run_group("attach23", trials, interp,
              extent=(0, 9.5, -25, 45), bins=(19, 28), count_levels=(5, 25),
              model_png=str(PICS / "action_space_simple_attach23.png"),
              group_figsize=(15, 7.2), compare_figsize=(15, 7.0),
              legend_loc="lower right", compare_width_ratios=0.95)

    trials, interp = load_dettach()
    print(f"dettach: {sum(len(t['neck']) for t in trials)} annotated frames, "
          f"{len(interp['neck'])} resampled samples")
    # the legend goes top left (no data at low height and high DISC)
    run_group("dettach", trials, interp,
              extent=(-0.5, 18.5, -5, 70), bins=(38, 30), count_levels=(10, 50),
              model_png=str(PICS / "action_space_simple_dettach.png"),
              group_figsize=(15, 7.2), compare_figsize=(15, 7.0),
              legend_loc="upper left", compare_width_ratios=0.95)

    # pooled group: the attachment and detachment annotated points together, matching the
    # pooled training data (config.MIXED_DATA_FILES) of the --mix option
    a_trials, _ = load_attach23()
    d_trials, d_interp = load_dettach()
    a_interp = load_processed_csvs(["data/processed/attach23_01.csv"])
    mixed_interp = dict(
        disc=np.concatenate([a_interp["disc"], d_interp["disc"]]),
        neck=np.concatenate([a_interp["neck"], d_interp["neck"]]),
        height=np.concatenate([a_interp["height"], d_interp["height"]]),
    )
    print(f"mixed: {sum(len(t['neck']) for t in a_trials + d_trials)} annotated frames, "
          f"{len(mixed_interp['neck'])} resampled samples")
    run_group("mixed", a_trials + d_trials, mixed_interp,
              extent=(-0.5, 18.5, -25, 70), bins=(38, 38), count_levels=(10, 50),
              model_png=str(PICS / "action_space_simple_mixed.png"),
              group_figsize=(15, 7.2), compare_figsize=(15, 7.0),
              legend_loc="upper left", compare_width_ratios=0.95)
