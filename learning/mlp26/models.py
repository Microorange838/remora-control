"""Model construction, callbacks and evaluation metrics.

The training loops themselves live in main.py and export.py.
"""

import numpy as np
import tensorflow as tf
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

from .config import SIMPLE_MLP_N_INPUT_DIM, PARAM_MLP_N_INPUT_DIM


# Two-input network ([DISC, HEIGHT] -> NECK), used to render the decision-space
# figures. Structurally identical to build_param_mlp; only the input width differs.
# It shares the controller's dataset and scaler, taking the first two columns of the
# standardized inputs (the scaler transforms each column independently, so slicing is
# equivalent to standardizing the two columns on their own).
def build_simple_mlp():
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(256, activation=tf.nn.relu, input_dim=SIMPLE_MLP_N_INPUT_DIM),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(128, activation=tf.nn.relu),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(1)
    ])

    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
                  loss='mse',
                  metrics=['mae', 'mse'])
    return model


# Six-input controller: the disc-surface angle and head height together with their
# first and second time derivatives. Two hidden layers of 256 and 128 units, dropout
# 0.2, Adam at 1e-3; this is the network exported for the robot.
def build_param_mlp():
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(256, activation=tf.nn.relu, input_dim=PARAM_MLP_N_INPUT_DIM),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(128, activation=tf.nn.relu),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(1)
    ])

    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
                  loss='mse',
                  metrics=['mae', 'mse'])
    return model


# Sliding-window variant (16 inputs). Explored during development; not used in the
# reported results.
def build_windowed_mlp():
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(256, activation=tf.nn.relu, input_dim=16),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(128, activation=tf.nn.relu),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(64, activation=tf.nn.relu),
        tf.keras.layers.Dense(1)
    ])

    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
                  loss='mse',
                  metrics=['mae', 'mse'])
    return model


# Early stopping, learning-rate reduction and checkpointing.
def make_callbacks(ckpt_path):
    return [
        tf.keras.callbacks.EarlyStopping(
            monitor='val_loss',
            patience=15,
            restore_best_weights=True,
            verbose=1
        ),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.5,
            patience=10,
            min_lr=1e-6,
            verbose=1
        ),
        tf.keras.callbacks.ModelCheckpoint(
            ckpt_path,
            monitor='val_loss',
            save_best_only=True,
            verbose=1
        )
    ]


def evaluate_model(y_true, y_pred, dataset_name):
    mse = mean_squared_error(y_true, y_pred)
    rmse = np.sqrt(mse)
    mae = mean_absolute_error(y_true, y_pred)
    r2 = r2_score(y_true, y_pred)

    print(f"\n{dataset_name}:")
    print(f"MSE: {mse:.4f}")
    print(f"RMSE: {rmse:.4f}")
    print(f"MAE: {mae:.4f}")
    print(f"R2: {r2:.4f}")

    return mse, rmse, mae, r2
