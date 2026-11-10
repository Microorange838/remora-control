"""Training entry point for the neck controllers.

Runs, in order: the two-input network used for the decision-space figures, the six-input
controller, and the sliding-window variant; then the closed-loop simulation. Data and
output paths are relative to the repository root, so run it from there:

    python -m learning.mlp26                   # all of the above
    python -m learning.mlp26 --skip-simulation # training only
    python -m learning.mlp26 --export          # train and export the deployed controllers
                                               # -> models/{attachment,detachment}.tflite

Cannot be run as `python learning/mlp26/main.py` (the package's relative imports fail).
"""

import tensorflow as tf
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import load_model
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import pygame
import pymunk.pygame_util
import time
from scipy.interpolate import CubicSpline

import argparse
from pathlib import Path

from .config import DEFAULT_DATA_FILES, MIXED_DATA_FILES
from .dataset import build_datasets
from .export import export_all
from .models import build_simple_mlp, build_param_mlp, build_windowed_mlp, make_callbacks, evaluate_model
from .viz import plot_training_history, plot_heatmap_neck, plot_residuals
from .simulation import plot_simu_bio_simulation_pymunk_v2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Train the neck controllers (run from the repository root)")
    parser.add_argument('--data-file', action='append', default=None,
                        help='training CSV; may be repeated (default data/processed/attach23_01.csv)')
    parser.add_argument('--window-size', type=int, default=8,
                        help='sliding-window length (default 8)')
    parser.add_argument('--epochs', type=int, default=200,
                        help='number of epochs (default 200)')
    parser.add_argument('--batch-size', type=int, default=32,
                        help='batch size (default 32)')
    parser.add_argument('--ckpt-prefix', default='',
                        help='prefix for checkpoint filenames, e.g. verify_ (default empty)')
    parser.add_argument('--skip-simulation', action='store_true',
                        help='skip the pygame closed-loop simulation')
    parser.add_argument('--heatmap-path', default='figures/action_space_simple.png',
                        help='where to write the decision-space figure of the two-input '
                             'network (default figures/action_space_simple.png)')
    parser.add_argument('--mix', action='store_true',
                        help='train on both behaviours pooled (config.MIXED_DATA_FILES) instead '
                             'of a single one; mutually exclusive with --data-file')
    parser.add_argument('--fig-prefix', default='',
                        help='prefix for the training-history figures; empty means do not write '
                             'them (e.g. figures/rnd_attach23_ -> training_history_*.png)')
    parser.add_argument('--split', choices=['random', 'file'], default='random',
                        help='how to split training from validation: random 70/30 (default), or '
                             'file = hold out the whole sequence given by --val-file')
    parser.add_argument('--val-file', type=int, default=None, metavar='IDX',
                        help='with --split file: index of the sequence to hold out (0-based, in '
                             'the order the files were given)')
    parser.add_argument('--export', action='store_true',
                        help='train and export the deployed controllers instead: one six-input '
                             'network per behaviour -> models/{attachment,detachment}.tflite plus '
                             'the matching _scaler.json. Nothing else is trained, plotted or run')
    parser.add_argument('--export-dir', default='models',
                        help='output directory for --export (default models)')
    args = parser.parse_args(argv)
    if args.mix and args.data_file:
        parser.error('--mix and --data-file are mutually exclusive')
    if args.split == 'file':
        if args.val_file is None:
            parser.error('--split file requires --val-file (index of the held-out sequence)')
        data_files = MIXED_DATA_FILES if args.mix else (args.data_file or DEFAULT_DATA_FILES)
        if len(data_files) < 2:
            parser.error('--split file needs at least 2 sequences')
        if not 0 <= args.val_file < len(data_files):
            parser.error(f'--val-file out of range: {len(data_files)} sequences given')
    return args


def run(argv=None):
    args = parse_args(argv)

    # Deployment path: train and export one controller per behaviour, then stop.
    if args.export:
        print("\n" + "="*60)
        print("Exporting the deployed controllers (six inputs -> neck angle)")
        print("="*60)
        export_all(out_dir=args.export_dir)
        print("\nExport complete.")
        return

    if args.mix:
        data_files = MIXED_DATA_FILES
    else:
        data_files = args.data_file if args.data_file else DEFAULT_DATA_FILES

    if args.fig_prefix:
        Path(args.fig_prefix).parent.mkdir(parents=True, exist_ok=True)
    fig_simple = args.fig_prefix + 'training_history_simple.png' if args.fig_prefix else None
    fig_param = args.fig_prefix + 'training_history_param.png' if args.fig_prefix else None
    fig_windowed = args.fig_prefix + 'training_history_windowed.png' if args.fig_prefix else None

    d = build_datasets(data_files, window_size=args.window_size,
                       split=args.split, val_file_idx=args.val_file)

    # ============================================================
    # Two-input network: [DISC, HEIGHT] -> NECK
    # Shares the controller's dataset and scaler; takes the first two columns of x.
    # ============================================================
    print("\n" + "="*60)
    print("Training the two-input network ([DISC, HEIGHT] -> neck angle)")
    print("="*60)

    model_simple = build_simple_mlp()
    callbacks_simple = make_callbacks(args.ckpt_prefix + 'best_model_simple.h5')

    history_simple = model_simple.fit(
        d.x_train[:, :2], d.y_train,
        epochs=args.epochs,
        batch_size=args.batch_size,
        validation_data=(d.x_val[:, :2], d.y_val),
        callbacks=callbacks_simple,
        verbose=1
    )

    train_predictions_simple = model_simple.predict(d.x_train[:, :2])
    val_predictions_simple = model_simple.predict(d.x_val[:, :2])

    train_predictions_simple = d.scaler_y.inverse_transform(train_predictions_simple)
    val_predictions_simple = d.scaler_y.inverse_transform(val_predictions_simple)
    y_train_original_simple = d.scaler_y.inverse_transform(d.y_train.reshape(-1, 1)).flatten()
    y_val_original_simple = d.scaler_y.inverse_transform(d.y_val.reshape(-1, 1)).flatten()

    train_metrics_simple = evaluate_model(y_train_original_simple, train_predictions_simple.flatten(), "two-input network, training set")
    val_metrics_simple = evaluate_model(y_val_original_simple, val_predictions_simple.flatten(), "two-input network, validation set")

    plot_training_history(history_simple, y_train_original_simple, train_predictions_simple,
                          y_val_original_simple, val_predictions_simple,
                          save_path=fig_simple)

    # Decision-space figure: the two-input network maps directly onto a DISC x HEIGHT grid.
    plot_heatmap_neck(model_simple, d.scaler_x, d.scaler_y,
                      disc_range=(0, 60), height_range=(0, 10), resolution=100,
                      save_path=args.heatmap_path)

    # ============================================================
    # Six-input controller
    # ============================================================
    print("\n" + "="*60)
    print("Training the six-input controller")
    print("="*60)

    model = build_param_mlp()
    callbacks = make_callbacks(args.ckpt_prefix + 'best_model.h5')

    history = model.fit(
        d.x_train, d.y_train,
        epochs=args.epochs,
        batch_size=args.batch_size,
        validation_data=(d.x_val, d.y_val),
        callbacks=callbacks,
        verbose=1
    )

    train_predictions = model.predict(d.x_train)
    val_predictions = model.predict(d.x_val)

    train_predictions = d.scaler_y.inverse_transform(train_predictions)
    val_predictions = d.scaler_y.inverse_transform(val_predictions)
    y_train_original = d.scaler_y.inverse_transform(d.y_train.reshape(-1, 1)).flatten()
    y_val_original = d.scaler_y.inverse_transform(d.y_val.reshape(-1, 1)).flatten()

    train_metrics = evaluate_model(y_train_original, train_predictions.flatten(), "six-input controller, training set")
    val_metrics = evaluate_model(y_val_original, val_predictions.flatten(), "six-input controller, validation set")

    plot_training_history(history, y_train_original, train_predictions,
                          y_val_original, val_predictions,
                          save_path=fig_param)

    # ============================================================
    # Sliding-window variant (16 inputs = DISC/HEIGHT over an 8-step window).
    # Explored during development; not part of the reported results.
    # ============================================================
    print("\n" + "="*60)
    print("Training the sliding-window variant (16 inputs)")
    print("="*60)

    model_windowed = build_windowed_mlp()

    print("\nsliding-window model:")
    model_windowed.summary()

    callbacks_windowed = make_callbacks(args.ckpt_prefix + 'best_model_windowed.h5')

    print("\ntraining the sliding-window model...")
    history_windowed = model_windowed.fit(
        d.x_windowed_train, d.y_windowed_train,
        epochs=args.epochs,
        batch_size=args.batch_size,
        validation_data=(d.x_windowed_val, d.y_windowed_val),
        callbacks=callbacks_windowed,
        verbose=1
    )

    # evaluate the sliding-window model
    train_predictions_windowed = model_windowed.predict(d.x_windowed_train)
    val_predictions_windowed = model_windowed.predict(d.x_windowed_val)

    train_predictions_windowed = d.scaler_y.inverse_transform(train_predictions_windowed)
    val_predictions_windowed = d.scaler_y.inverse_transform(val_predictions_windowed)
    y_windowed_train_original = d.scaler_y.inverse_transform(d.y_windowed_train.reshape(-1, 1)).flatten()
    y_windowed_val_original = d.scaler_y.inverse_transform(d.y_windowed_val.reshape(-1, 1)).flatten()

    print("\n" + "="*60)
    print("sliding-window variant")
    print("="*60)
    train_metrics_windowed = evaluate_model(y_windowed_train_original, train_predictions_windowed.flatten(), "sliding-window, training set")
    val_metrics_windowed = evaluate_model(y_windowed_val_original, val_predictions_windowed.flatten(), "sliding-window, validation set")

    print("\n" + "="*60)
    print("validation-set comparison")
    print("="*60)
    print(f"\n{'metric':<10} {'two-input':<22} {'six-input':<22} {'windowed':<22}")
    print("-" * 80)
    print(f"{'RMSE':<10} {val_metrics_simple[1]:<22.4f} {val_metrics[1]:<22.4f} {val_metrics_windowed[1]:<22.4f}")
    print(f"{'MAE':<10} {val_metrics_simple[2]:<22.4f} {val_metrics[2]:<22.4f} {val_metrics_windowed[2]:<22.4f}")
    print(f"{'R2':<10} {val_metrics_simple[3]:<22.4f} {val_metrics[3]:<22.4f} {val_metrics_windowed[3]:<22.4f}")

    plot_training_history(history_windowed, y_windowed_train_original, train_predictions_windowed,
                          y_windowed_val_original, val_predictions_windowed,
                          save_path=fig_windowed)

    # Closed-loop simulation. Uses the six-input controller, whose state vector matches
    # the simulation loop; the two-input network cannot be substituted directly.
    if not args.skip_simulation:
        plot_simu_bio_simulation_pymunk_v2(model, d.scaler_x, d.scaler_y, use_drag=True, drag_mode='simple')

    # plot_residuals(y_val_original, val_predictions, "validation set")

    print("\n" + "=" * 50)
    print("summary")
    print("=" * 50)
    print(f"six-input controller, epochs run: {len(history.history['loss'])}")
    print(f"final learning rate: {model.optimizer.learning_rate.numpy():.2e}")
    print(f"training samples: {len(d.x_train)}")
    print(f"validation samples: {len(d.x_val)}")


if __name__ == "__main__":
    run()
