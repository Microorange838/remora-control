"""Data pipeline: CSV loading, standardization, sliding windows, train/validation split.

This module does not import tensorflow.
"""

from dataclasses import dataclass
from typing import Any

import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler


def create_sliding_window_data(x, y, columns=[0, 1], window_size=4, file_labels=None):
    """Sample the data with a sliding window.

    Args:
        x: inputs, shape (n_samples, n_features)
        y: targets, shape (n_samples,)
        columns: columns to use (default [0, 1])
        window_size: window length (default 4)
        file_labels: optional array marking the source sequence of each sample

    Returns:
        x_windowed: (n_valid_windows, len(columns) * window_size)
        y_windowed: (n_valid_windows,)
        window_sources: source of each window -- the file label if file_labels was
            given, otherwise the starting index

    For a window of 4, each row is laid out as:
        [x0_{n-3}, x0_{n-2}, x0_{n-1}, x0_n, x1_{n-3}, x1_{n-2}, x1_{n-1}, x1_n]
    """
    # select the requested columns
    x_selected = x[:, columns]  # shape: (n_samples, len(columns))

    n_samples = x_selected.shape[0]
    n_cols = len(columns)

    if n_samples < window_size:
        raise ValueError(f"window size {window_size} is larger than the {n_samples} samples available")

    # list of valid windows
    valid_windows = []
    valid_y = []
    window_sources = []  # source of each window
    skipped_count = 0

    for i in range(n_samples - window_size + 1):
        # windows must not straddle two sequences
        if file_labels is not None:
            window_file_labels = file_labels[i:i + window_size]
            if not np.all(window_file_labels == window_file_labels[0]):
                skipped_count += 1
                continue

        window_data = x_selected[i:i + window_size, :]  # (window_size, n_cols)

        # flattened column-major: all time steps of column 0, then of column 1
        window_features = np.zeros(n_cols * window_size)
        for col_idx in range(n_cols):
            start_idx = col_idx * window_size
            end_idx = start_idx + window_size
            window_features[start_idx:end_idx] = window_data[:, col_idx]

        valid_windows.append(window_features)
        # the target is the last time step of the window
        valid_y.append(y[i + window_size - 1])

        if file_labels is not None:
            window_sources.append(file_labels[i])
        else:
            window_sources.append(i)

    x_windowed = np.array(valid_windows)
    y_windowed = np.array(valid_y)
    window_sources = np.array(window_sources)

    print(f"\n=== sliding-window data ===")
    print(f"input shape: {x.shape}")
    print(f"columns: {columns}")
    print(f"window size: {window_size}")
    if file_labels is not None:
        n_files = len(np.unique(file_labels))
        print(f"sequences: {n_files}")
        print(f"\nwindows per sequence:")
        unique_sources, counts = np.unique(window_sources, return_counts=True)
        for source_id, count in zip(unique_sources, counts):
            percentage = count / len(window_sources) * 100
            print(f"  sequence {int(source_id)}: {count} windows ({percentage:.1f}%)")
    print(f"windows skipped at sequence boundaries: {skipped_count}")
    print(f"windowed input shape: {x_windowed.shape}")
    print(f"windowed target shape: {y_windowed.shape}")
    print(f"feature order (window size = {window_size}):")
    feature_names = []
    for col_idx in columns:
        for t in range(window_size):
            feature_names.append(f"x{col_idx}_{{n-{window_size-1-t}}}")
    print(f"  {', '.join(feature_names)}")

    return x_windowed, y_windowed, window_sources


@dataclass
class Datasets:
    """Everything build_datasets produces, carried through one run.

    scaler_x / scaler_y are fitted objects and must be passed on as they are to
    evaluation, the decision-space figures and the simulation (never re-fitted here).
    export.py writes their statistics out for the deployment side.
    """
    data_files: Any
    x: Any
    y: Any
    file_labels: Any
    scaler_x: Any
    scaler_y: Any
    x_scaled: Any
    y_scaled: Any
    x_train: Any
    x_val: Any
    y_train: Any
    y_val: Any
    x_windowed: Any
    y_windowed: Any
    window_sources: Any
    x_windowed_train: Any
    x_windowed_val: Any
    y_windowed_train: Any
    y_windowed_val: Any


def build_datasets(data_files, window_size=8, split='random', val_file_idx=None):
    """Load the given processed CSVs, standardize, split and window them.

    The first column of each CSV is the target (neck angle); the remaining six are the
    controller inputs. window_size applies to the sliding-window call only.

    split='random' (default) holds out a random 30% for early stopping.
    split='file' holds out one whole sequence instead: the file at index val_file_idx
    becomes the validation set and the rest is used for training. The scaler is fitted
    on all data either way.
    """
    if split not in ('random', 'file'):
        raise ValueError(f"split must be 'random' or 'file', got {split!r}")
    if split == 'file':
        if val_file_idx is None:
            raise ValueError("split='file' requires val_file_idx (index of the validation sequence)")
        if not 0 <= val_file_idx < len(data_files):
            raise ValueError(f"val_file_idx={val_file_idx} out of range 0~{len(data_files) - 1}")
    all_x = []
    all_y = []
    file_labels = []

    for file_idx, file_path in enumerate(data_files):
        data_file = pd.read_csv(file_path, header=None, skiprows=3)
        data_file = data_file.dropna(subset=[0])  # drop rows with an empty target

        y_file = np.array(data_file.iloc[:, 0])
        x_file = np.array(data_file.iloc[:, 1:])

        all_x.append(x_file)
        all_y.append(y_file)
        file_labels.extend([file_idx] * len(y_file))

    x = np.vstack(all_x)
    y = np.concatenate(all_y)
    file_labels = np.array(file_labels)

    # standardization statistics are fitted over all data at once
    scaler_x = StandardScaler()
    scaler_y = StandardScaler()
    x_scaled = scaler_x.fit_transform(x)
    y_scaled = scaler_y.fit_transform(y.reshape(-1, 1)).flatten()

    df_scaled = pd.DataFrame(x_scaled)
    df_scaled['y'] = y_scaled
    df_scaled['file_label'] = file_labels

    if split == 'random':
        # Note: for temporally ordered data a random split places near-identical
        # neighbouring frames on both sides, so the held-out set measures fit rather
        # than generalization. It is used only to supervise early stopping.
        x_train, x_val, y_train, y_val = train_test_split(
            x_scaled, y_scaled, test_size=0.3, random_state=42
        )
    else:
        # the sequence at val_file_idx becomes the validation set
        val_mask = file_labels == val_file_idx
        train_mask = ~val_mask

        x_train = x_scaled[train_mask]
        x_val = x_scaled[val_mask]
        y_train = y_scaled[train_mask]
        y_val = y_scaled[val_mask]

        print(f"\n=== split by sequence ===")
        print(f"validation sequence: {data_files[val_file_idx]}")
        print(f"training samples: {len(x_train)}")
        print(f"validation samples: {len(x_val)}")
        print(f"validation fraction: {len(x_val)/(len(x_train)+len(x_val))*100:.1f}%")

    x_windowed, y_windowed, window_sources = create_sliding_window_data(x_scaled, y_scaled, columns=[0, 1], window_size=window_size, file_labels=file_labels)

    print(f"\nfirst 3 window samples:")
    for i in range(min(3, len(x_windowed))):
        print(f"sample {i} (sequence {int(window_sources[i])}): {x_windowed[i][:8]}... -> y={y_windowed[i]:.4f}")

    # the held-out sequence keeps its role for the windowed data (windows straddling a
    # sequence boundary were already dropped by create_sliding_window_data)
    if split == 'random':
        x_windowed_train, x_windowed_val, y_windowed_train, y_windowed_val = train_test_split(
            x_windowed, y_windowed, test_size=0.3, random_state=42
        )
    else:
        val_mask_windowed = window_sources == val_file_idx
        train_mask_windowed = ~val_mask_windowed

        x_windowed_train = x_windowed[train_mask_windowed]
        x_windowed_val = x_windowed[val_mask_windowed]
        y_windowed_train = y_windowed[train_mask_windowed]
        y_windowed_val = y_windowed[val_mask_windowed]

        print(f"\n=== windowed data, split by sequence ===")
        print(f"validation sequence: {data_files[val_file_idx]}")
        print(f"windowed training samples: {len(x_windowed_train)}")
        print(f"windowed validation samples: {len(x_windowed_val)}")
        print(f"windowed validation fraction: {len(x_windowed_val)/(len(x_windowed_train)+len(x_windowed_val))*100:.1f}%")

    print(f"\ndataset:")
    print(f"total samples: {len(x)}")
    print(f"from {len(data_files)} sequences:")
    for idx, file_path in enumerate(data_files):
        count = np.sum(file_labels == idx)
        print(f"  sequence {idx} ({file_path}): {count} rows")

    return Datasets(
        data_files=data_files,
        x=x,
        y=y,
        file_labels=file_labels,
        scaler_x=scaler_x,
        scaler_y=scaler_y,
        x_scaled=x_scaled,
        y_scaled=y_scaled,
        x_train=x_train,
        x_val=x_val,
        y_train=y_train,
        y_val=y_val,
        x_windowed=x_windowed,
        y_windowed=y_windowed,
        window_sources=window_sources,
        x_windowed_train=x_windowed_train,
        x_windowed_val=x_windowed_val,
        y_windowed_train=y_windowed_train,
        y_windowed_val=y_windowed_val,
    )
