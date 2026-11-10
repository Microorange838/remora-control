"""Training and TensorFlow Lite export of the neck controllers.

The controller is the six-input network described in the manuscript: the disc-surface
angle, the head height, and their first and second time derivatives. One controller is
trained per behaviour from the corresponding demonstrations.

The exported model operates on standardized inputs and outputs, so the statistics
fitted during training are written alongside it. Without them the network cannot be
fed correctly.
"""

import json
from pathlib import Path

import numpy as np
import tensorflow as tf

from .config import (DEFAULT_DATA_FILES, DETACHMENT_DATA_FILES,
                     FEATURE_COLUMNS, TARGET_COLUMN)
from .dataset import build_datasets
from .models import build_param_mlp

# Training protocol, matching the manuscript: Adam with a learning rate of 1e-3,
# batch size 32, and a randomly held-out 30% used for early stopping.
SEED = 0
EPOCHS = 200
BATCH_SIZE = 32

# One entry per behaviour: the controller and the demonstrations it is cloned from.
BEHAVIOURS = {
    "attachment": DEFAULT_DATA_FILES,
    "detachment": DETACHMENT_DATA_FILES,
}


def train_controller(data_files, seed=SEED):
    """Train one six-input controller on the given demonstrations.

    Returns (model, datasets). The seed is fixed so that a given run is reproducible.
    """
    np.random.seed(seed)
    tf.keras.utils.set_random_seed(seed)
    d = build_datasets(data_files, split="random")
    model = build_param_mlp()
    model.fit(
        d.x_train, d.y_train,
        epochs=EPOCHS, batch_size=BATCH_SIZE,
        validation_data=(d.x_val, d.y_val),
        callbacks=[
            tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=15,
                                             restore_best_weights=True, verbose=0),
            tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", factor=0.5,
                                                 patience=10, min_lr=1e-6, verbose=0),
        ],
        verbose=0,
    )
    return model, d


def scaler_to_dict(scaler_x, scaler_y):
    """Normalisation statistics as plain lists, for the deployment side."""
    return {
        "target": TARGET_COLUMN,
        "features": list(FEATURE_COLUMNS[:scaler_x.mean_.shape[0]]),
        "input_mean": scaler_x.mean_.tolist(),
        "input_scale": scaler_x.scale_.tolist(),
        "output_mean": scaler_y.mean_.tolist(),
        "output_scale": scaler_y.scale_.tolist(),
    }


def export_controller(data_files, tflite_path, scaler_path, seed=SEED, verbose=True):
    """Train one controller, write its .tflite and its normalisation statistics."""
    model, d = train_controller(data_files, seed=seed)

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()

    Path(tflite_path).parent.mkdir(parents=True, exist_ok=True)
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    with open(scaler_path, "w") as f:
        json.dump(scaler_to_dict(d.scaler_x, d.scaler_y), f, indent=2)

    if verbose:
        print(f"  {tflite_path}: {len(tflite_model)} bytes")
        print(f"  {scaler_path}: {d.scaler_x.mean_.shape[0]} inputs, "
              f"{d.scaler_y.mean_.shape[0]} outputs")
    return model, d


def export_all(out_dir="models", seed=SEED, verbose=True):
    """Export one controller per behaviour: attachment and detachment."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for behaviour, data_files in BEHAVIOURS.items():
        if verbose:
            print(f"\n{behaviour}: {len(data_files)} sequence(s)")
        export_controller(data_files,
                          out_dir / f"{behaviour}.tflite",
                          out_dir / f"{behaviour}_scaler.json",
                          seed=seed, verbose=verbose)
