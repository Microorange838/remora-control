# -*- coding: utf-8 -*-
"""Decision-space figure of the two-input network, with the underlying grid as a CSV.

One two-input network is trained per behaviour (seed 0, as for the evaluation), on the
resampled sequences with a randomly held-out 30% supervising early stopping. Its
predictions are rendered over a DISC x HEIGHT grid and written out, both as a figure and
as a long-format table.

The figure places DISC on the horizontal axis and HEIGHT on the vertical one, matching
the layout used in the manuscript; the network still takes [DISC, HEIGHT] in that order.

Outputs (--out-prefix to change, defaults below):
- figures/x1_decision_space_attach.png / .csv (100 x 100 grid)
- figures/x1_decision_space_dettach.png / .csv
CSV columns: height_cm, disc_deg, predicted_neck_deg.

Usage (run from the repository root):
    PYTHONIOENCODING=utf-8 MPLBACKEND=Agg python learning/plot_x1_decision_space.py
    # single behaviour: add --groups attach
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
import pandas as pd

import mlp26.config  # noqa: F401  # sets the rcParams; the figure then switches to a Latin font
from eval_raw_points import SEEDS, train_once

GROUPS = {
    "attach": dict(files=["data/processed/attach23_01.csv"], title="Attachment"),
    "dettach": dict(files=[f"data/processed/dettach{i}_clean.csv" for i in (1, 2, 3)],
                    title="Detachment"),
}

# Same grid as the decision-space figure produced by mlp26.main; the grid data does not
# depend on which axis is drawn horizontally.
DISC_RANGE = (0, 60)
HEIGHT_RANGE = (0, 10)
RESOLUTION = 100

FONT_SIZE = 28
CLABEL_SIZE = 18
BOX_ASPECT = 1.5  # panel width/height, kept the same as the measured-vs-predicted figure


def plot_decision_space_v1(model, scaler_x, scaler_y, title, save_path, csv_path,
                           show_colorbar=True):
    """Render the decision space with DISC on x and HEIGHT on y, and write the grid CSV.

    Predictions come from the two-input network over the DISC x HEIGHT grid, with
    contour lines and a filled colour map. The panel is locked to the same width/height
    as the measured-vs-predicted figure; the colour bar is optional.
    """
    disc_values = np.linspace(*DISC_RANGE, RESOLUTION)
    height_values = np.linspace(*HEIGHT_RANGE, RESOLUTION)
    DISC_grid, HEIGHT_grid = np.meshgrid(disc_values, height_values)  # x=DISC, y=HEIGHT

    # the network takes [DISC, HEIGHT]; standardization uses the first two columns of
    # the six-column scaler
    grid = np.column_stack([DISC_grid.flatten(), HEIGHT_grid.flatten()])
    grid_scaled = (grid - scaler_x.mean_[:2]) / scaler_x.scale_[:2]
    preds = scaler_y.inverse_transform(model.predict(grid_scaled, verbose=0)).flatten()
    Z = preds.reshape(DISC_grid.shape)

    fig, ax = plt.subplots(figsize=(10, 7))
    contour = ax.contour(DISC_grid, HEIGHT_grid, Z, levels=15,
                         colors="black", linewidths=0.5)
    ax.clabel(contour, inline=True, fontsize=CLABEL_SIZE)
    im = ax.contourf(DISC_grid, HEIGHT_grid, Z, levels=50, cmap="RdYlBu_r", alpha=0.8)
    # lock the panel before adding the colour bar, otherwise the bar is sized against the
    # pre-lock box. matplotlib's set_box_aspect takes height/width, hence 1/BOX_ASPECT.
    ax.set_box_aspect(1 / BOX_ASPECT)
    if show_colorbar:
        cb = fig.colorbar(im, ax=ax, label="Neck angle (°)")
        cb.locator = MaxNLocator(nbins=5)
        cb.update_ticks()
    ax.set_xlabel("Disc pitch (°)")
    ax.set_ylabel("Height (cm)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)

    pd.DataFrame({"height_cm": HEIGHT_grid.flatten(),
                  "disc_deg": DISC_grid.flatten(),
                  "predicted_neck_deg": preds}
                 ).to_csv(csv_path, index=False, float_format="%.6f")

    print(f"figure: {save_path} | grid data: {csv_path}")
    print(f"prediction range: [{preds.min():.4f}, {preds.max():.4f}] | mean {preds.mean():.4f}"
          f" | std {preds.std():.4f}")


def main():
    ap = argparse.ArgumentParser(description="Decision-space figure of the two-input network")
    ap.add_argument("--out-prefix", default="figures/x1_decision_space",
                    help="output path prefix (default figures/x1_decision_space)")
    ap.add_argument("--seed", type=int, default=SEEDS[0],
                    help="training seed (default 0, the seed used for the evaluation)")
    ap.add_argument("--groups", default="attach,dettach",
                    help="behaviours to draw, comma separated (default attach,dettach)")
    ap.add_argument("--colorbar", action="store_true",
                    help="add a colour bar (off by default: the contour labels carry the values)")
    args = ap.parse_args()

    # the shared rcParams already select a Latin font; pin it for this figure too, so
    # that the output does not depend on the fonts installed on the machine
    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["DejaVu Sans"],
                         "font.size": FONT_SIZE,
                         "axes.titlesize": FONT_SIZE,
                         "axes.labelsize": FONT_SIZE,
                         "xtick.labelsize": FONT_SIZE,
                         "ytick.labelsize": FONT_SIZE,
                         "legend.fontsize": FONT_SIZE})

    selected = [n for n in (s.strip() for s in args.groups.split(",")) if n in GROUPS]
    for name in selected:
        g = GROUPS[name]
        print("=" * 72)
        print(f"group = {name} | training files = {g['files']} | seed = {args.seed}")
        print("=" * 72)
        model, d, epochs_run = train_once(g["files"], args.seed, "simple")
        print(f"training finished: {epochs_run} epochs")
        plot_decision_space_v1(model, d.scaler_x, d.scaler_y, g["title"],
                               f"{args.out_prefix}_{name}.png",
                               f"{args.out_prefix}_{name}.csv",
                               show_colorbar=args.colorbar)


if __name__ == "__main__":
    main()
