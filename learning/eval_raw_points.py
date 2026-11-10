# -*- coding: utf-8 -*-
"""Evaluation of the neck controllers on the original annotated measurement points.

Training uses the resampled sequences, as during deployment, but the error is measured
only at the frames that were digitized by hand: no interpolated sample enters the
evaluation. The evaluation points are the same ones drawn in the decision-space
figures (plot_action_space_data.load_attach23 / load_dettach, the latter already
passed through data_cleaning.apply_dettach_exclusion).

Protocol:
- attachment: trained on data/processed/attach23_01.csv (410 resampled frames),
  evaluated at the 41 annotated frames;
- detachment: trained on data/processed/dettach{1,2,3}_clean.csv (750 resampled
  frames), evaluated at the 12+25+38 = 75 annotated frames;
- --model simple (default): the two-input network, inputs (DISC, HEIGHT). The mapping
  is static, so the raw points are fed directly (standardization uses the first two
  columns of the six-column scaler, as in viz.plot_heatmap_neck);
- --model param: the six-input controller, inputs [DISC, HEIGHT, DISC_V, DISC_A, H_V,
  H_A]. The derivatives are recomputed per trial from the 25 Hz annotated sequences
  with the same formula used in data_preprocess (v[1:] = diff * rate, then a moving
  average; a[2:] = diff(v[1:]) * rate, then a moving average). Two smoothing variants
  are reported: the window used in training for the behaviour, and the other window as
  a control. Derivatives are never computed across trials.
- the randomly held-out 30% (random_state=42, as in build_datasets) only supervises
  early stopping;
- a single seed (0) is used, matching the reported numbers.

Usage (run from the repository root):
    PYTHONIOENCODING=utf-8 MPLBACKEND=Agg python learning/eval_raw_points.py
    PYTHONIOENCODING=utf-8 MPLBACKEND=Agg python learning/eval_raw_points.py --model param
    # measured vs predicted figure + the same data as a CSV:
    PYTHONIOENCODING=utf-8 MPLBACKEND=Agg python learning/eval_raw_points.py \
        --model param --save-figs figures/x2_pred_vs_actual
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tensorflow as tf

from data_preprocess import moving_average
from mlp26.dataset import build_datasets
from mlp26.models import build_simple_mlp, build_param_mlp
from plot_action_space_data import load_attach23, load_dettach

SEEDS = (0,)
RATE_HZ = 25  # sampling rate of the annotated sequences (acquired at 250 Hz, digitized every 10th frame)

GROUPS = {
    "attach": dict(files=["data/processed/attach23_01.csv"], loader=load_attach23,
                   # trained with window 5; evaluated at the same window and at window 1
                   variants=[("w5", 5), ("w1", 1)]),
    "dettach": dict(files=[f"data/processed/dettach{i}_clean.csv" for i in (1, 2, 3)],
                    loader=load_dettach,
                    # trained with window 1; window 5 reported as a control
                    variants=[("w1", 1), ("w5", 5)]),
}


def derivative_feats(x, rate, window):
    """Same differencing formula as data_preprocess: v[1:] = diff * rate, then smoothing;
    a[2:] = diff(v[1:]) * rate, then smoothing."""
    v = np.zeros(len(x))
    v[1:] = np.diff(x) * rate
    v = moving_average(v, window)
    a = np.zeros(len(x))
    a[2:] = np.diff(v[1:]) * rate
    a = moving_average(a, window)
    return v, a


def build_raw_xy(trials, window):
    """Annotated points to controller inputs. The two-input network uses the first two
    columns. Derivatives are computed per trial, never across trial boundaries."""
    xs, ys = [], []
    for t in trials:
        dv, da = derivative_feats(np.asarray(t["disc"], float), RATE_HZ, window)
        hv, ha = derivative_feats(np.asarray(t["height"], float), RATE_HZ, window)
        xs.append(np.column_stack([t["disc"], t["height"], dv, da, hv, ha]))
        ys.append(np.asarray(t["neck"], float))
    return np.vstack(xs), np.concatenate(ys)


def raw_point_metrics(model, d, x_raw):
    """Predictions at the annotated points. x_raw has 2 or 6 columns depending on the model."""
    xs = (x_raw - d.scaler_x.mean_[:x_raw.shape[1]]) / d.scaler_x.scale_[:x_raw.shape[1]]
    pred = d.scaler_y.inverse_transform(model.predict(xs, verbose=0)).flatten()
    return pred


def train_once(files, seed, model_name):
    """One training run without checkpointing: returns (model, d, epochs_run)."""
    np.random.seed(seed)
    tf.keras.utils.set_random_seed(seed)
    d = build_datasets(files, split="random")  # the held-out 30% only supervises early stopping
    model = build_simple_mlp() if model_name == "simple" else build_param_mlp()
    xtr, xva = (d.x_train[:, :2], d.x_val[:, :2]) if model_name == "simple" \
        else (d.x_train, d.x_val)
    history = model.fit(
        xtr, d.y_train,
        epochs=200, batch_size=32,
        validation_data=(xva, d.y_val),
        callbacks=[
            tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=15,
                                             restore_best_weights=True, verbose=0),
            tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", factor=0.5,
                                                 patience=10, min_lr=1e-6, verbose=0),
        ],
        verbose=0,
    )
    return model, d, len(history.history["loss"])


def metrics(true, pred):
    err = pred - true
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((true - true.mean()) ** 2))
    return dict(rmse=float(np.sqrt(np.mean(err ** 2))),
                mae=float(np.mean(np.abs(err))),
                max_abs=float(np.max(np.abs(err))),
                r2=1.0 - ss_res / ss_tot)


def plot_pred_vs_actual(true, pred, m, title, save_path):
    """Measured against predicted neck angle at the annotated points, with the error
    printed in the panel. The font is pinned explicitly so that the superscript in R²
    renders independently of the fonts installed on the machine."""
    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["DejaVu Sans"],
                         "font.size": 28, "axes.labelsize": 28, "axes.titlesize": 28,
                         "xtick.labelsize": 28, "ytick.labelsize": 28})
    lo = min(true.min(), pred.min())
    hi = max(true.max(), pred.max())
    pad = 0.05 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    fig, ax = plt.subplots(figsize=(9.25, 6.47), layout="compressed")
    ax.plot([lo, hi], [lo, hi], "--", color="0.4", lw=2, zorder=1)
    ax.scatter(true, pred, s=60, alpha=0.7, zorder=2)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_box_aspect(1 / 1.5)  # panel width/height; matplotlib takes height/width
    ax.set_xlabel("Measured neck angle (°)")
    ax.set_ylabel("Predicted neck angle (°)")
    ax.set_title(title)
    ax.text(0.04, 0.96, f"RMSE = {m['rmse']:.2f}°\nMAE = {m['mae']:.2f}°\nR² = {m['r2']:.3f}",
            transform=ax.transAxes, va="top", fontsize=24,
            bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"))
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def export_pred_vs_actual(trials, true, pred, save_path):
    """The data behind the figure, as a CSV (same arrays as the PNG)."""
    trial_no = np.concatenate([np.full(len(t["neck"]), i) for i, t in enumerate(trials)])
    pd.DataFrame({"trial": trial_no,
                  "measured_neck_deg": true,
                  "predicted_neck_deg": pred}
                 ).to_csv(save_path, index=False, float_format="%.6f")


def main():
    ap = argparse.ArgumentParser(
        description="Evaluation at the annotated measurement points (two-input / six-input network)")
    ap.add_argument("--model", choices=["simple", "param"], default="simple")
    ap.add_argument("--save-figs", default=None, metavar="PREFIX",
                    help="also write {PREFIX}_{attach,dettach}.png and the matching .csv")
    args = ap.parse_args()
    n_in = 2 if args.model == "simple" else 6
    print(f"model = {args.model} ({n_in} inputs)")

    for name, g in GROUPS.items():
        trials, _ = g["loader"]()
        true_all = np.concatenate([np.asarray(t["neck"], float) for t in trials])
        print("\n" + "=" * 72)
        print(f"group = {name} | training files = {g['files']}")
        print(f"annotated evaluation points = {len(true_all)} (per trial "
              f"{[len(t['neck']) for t in trials]})")
        print("=" * 72)

        # Input features at the annotated points for each smoothing variant. The
        # two-input network ignores the variants.
        feat = {tag: build_raw_xy(trials, w)[0] for tag, w in g["variants"]}

        rows = []
        for seed in SEEDS:
            model, d, epochs_run = train_once(g["files"], seed, args.model)
            xtr_val = d.x_val[:, :2] if args.model == "simple" else d.x_val

            # reference: RMSE over the held-out resampled frames
            val_pred = d.scaler_y.inverse_transform(
                model.predict(xtr_val, verbose=0)).flatten()
            val_true = d.scaler_y.inverse_transform(d.y_val.reshape(-1, 1)).flatten()
            val_rmse = float(np.sqrt(np.mean((val_pred - val_true) ** 2)))

            parts = [f"seed={seed} epochs={epochs_run:3d} | held-out frames RMSE={val_rmse:.2f}°"]
            per_variant = {tag: metrics(true_all, raw_point_metrics(model, d, xy[:, :n_in]))
                           for tag, xy in feat.items()}
            for tag, m in per_variant.items():
                parts.append(f"{tag}: RMSE={m['rmse']:.2f}° MAE={m['mae']:.2f}° "
                             f"max|err|={m['max_abs']:.2f}° R²={m['r2']:.3f}")
            print(" | ".join(parts))
            rows.append(per_variant)

            if args.save_figs and seed == SEEDS[0]:
                tag0, _ = g["variants"][0]
                pred0 = raw_point_metrics(model, d, feat[tag0][:, :n_in])
                out_png = f"{args.save_figs}_{name}.png"
                plot_pred_vs_actual(true_all, pred0, per_variant[tag0],
                                    "Attachment" if name == "attach" else "Detachment",
                                    out_png)
                out_csv = f"{args.save_figs}_{name}.csv"
                export_pred_vs_actual(trials, true_all, pred0, out_csv)
                print(f"saved: {out_png} and {out_csv} (seed={seed}, "
                      f"variant={tag0}, n={len(true_all)})")

        for tag, _ in g["variants"]:
            v = np.array([r[tag]["rmse"] for r in rows])
            ma = np.array([r[tag]["mae"] for r in rows])
            print(f"-> {name}[{tag}] ({len(SEEDS)} seed(s)): annotated-point RMSE = "
                  f"{v.mean():.2f}° ± {v.std():.2f}°, MAE = {ma.mean():.2f}° "
                  f"± {ma.std():.2f}° (n={len(true_all)})")


if __name__ == "__main__":
    main()
