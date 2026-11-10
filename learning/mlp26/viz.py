"""Figures: training history, decision-space map, residuals.

This module imports neither pygame/pymunk nor tensorflow. Any entry point that produces
a figure must import mlp26.config first, since the rcParams are set there.
"""

import matplotlib.pyplot as plt
import numpy as np

from .config import HEATMAP_TITLE


def plot_training_history(history, y_train_original, train_predictions,
                          y_val_original, val_predictions, save_path=None):
    """Loss and MAE curves, plus measured against predicted on both splits.

    Args:
        history: Keras history object
        y_train_original / val: targets, after inverse standardization
        train_predictions / val: predictions, after inverse standardization
        save_path: if given, the figure is written there (dpi=150) before showing
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), layout="compressed")

    axes[0, 0].plot(history.history['loss'], label='train')
    axes[0, 0].plot(history.history['val_loss'], label='validation')
    axes[0, 0].set_title('Loss')
    axes[0, 0].set_xlabel('Epoch')
    axes[0, 0].set_ylabel('Loss')
    axes[0, 0].legend()

    axes[0, 1].plot(history.history['mae'], label='train')
    axes[0, 1].plot(history.history['val_mae'], label='validation')
    axes[0, 1].set_title('Mean absolute error')
    axes[0, 1].set_xlabel('Epoch')
    axes[0, 1].set_ylabel('MAE')
    axes[0, 1].legend()

    axes[1, 0].scatter(y_train_original, train_predictions, alpha=0.5)
    axes[1, 0].plot([y_train_original.min(), y_train_original.max()],
                    [y_train_original.min(), y_train_original.max()], 'r--', lw=2)
    axes[1, 0].set_title('Training set: predicted vs measured')
    axes[1, 0].set_xlabel('Measured')
    axes[1, 0].set_ylabel('Predicted')

    axes[1, 1].scatter(y_val_original, val_predictions, alpha=0.5)
    axes[1, 1].plot([y_val_original.min(), y_val_original.max()],
                    [y_val_original.min(), y_val_original.max()], 'r--', lw=2)
    axes[1, 1].set_title('Validation set: predicted vs measured')
    axes[1, 1].set_xlabel('Measured')
    axes[1, 1].set_ylabel('Predicted')

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"\ntraining history figure saved: {save_path}")
    plt.show()


# Decision-space map of the two-input network: DISC x HEIGHT -> NECK.
# The input width matches the network, so standardization uses the first two columns of
# the six-column scaler (StandardScaler transforms each column independently, so taking
# the first two entries of mean_/scale_ is the same as standardizing DISC and HEIGHT
# alone, which is what the network was trained on).
def plot_heatmap_neck(model, scaler_x, scaler_y, disc_range=(-20, 60), height_range=(0, 10),
                      resolution=50, save_path=None):
    """Render the decision space of the two-input network over a DISC x HEIGHT grid.

    Args:
        model: Keras model with two inputs
        scaler_x: the six-column StandardScaler fitted during training (first two
            columns are used)
        scaler_y: StandardScaler for the neck angle
        disc_range / height_range: extent of the grid
        resolution: number of points along each edge
        save_path: if given, the figure is written there
    """
    disc_values = np.linspace(disc_range[0], disc_range[1], resolution)
    height_values = np.linspace(height_range[0], height_range[1], resolution)
    HEIGHT_grid, DISC_grid = np.meshgrid(height_values, disc_values)

    grid_size = resolution * resolution
    input_data = np.zeros((grid_size, 2))
    input_data[:, 0] = DISC_grid.flatten()
    input_data[:, 1] = HEIGHT_grid.flatten()

    input_scaled = (input_data - scaler_x.mean_[:2]) / scaler_x.scale_[:2]

    predictions_scaled = model.predict(input_scaled, verbose=0)
    predictions = scaler_y.inverse_transform(predictions_scaled).flatten()

    predictions_grid = predictions.reshape(resolution, resolution)

    # The grid is drawn with DISC on the vertical axis and HEIGHT on the horizontal one;
    # the network still takes [DISC, HEIGHT] in that order.
    fig, ax = plt.subplots(figsize=(9.5, 8.0), layout="compressed")

    contour = ax.contour(HEIGHT_grid, DISC_grid, predictions_grid, levels=15,
                         colors='black', linewidths=0.5)
    ax.clabel(contour, inline=True, fontsize=14)
    im = ax.contourf(HEIGHT_grid, DISC_grid, predictions_grid, levels=50,
                     cmap='RdYlBu_r', alpha=0.8)
    fig.colorbar(im, ax=ax, label='Predicted neck angle (deg)')
    ax.set_xlabel('HEIGHT (cm)')
    ax.set_ylabel('DISC (deg)')
    ax.set_title(HEATMAP_TITLE)
    ax.set_box_aspect(1)

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"\ndecision-space figure saved: {save_path}")
    plt.show()

    print(f"\ndecision space over the grid:")
    print(f"DISC range: {disc_range}")
    print(f"HEIGHT range: {height_range}")
    print(f"prediction range: [{predictions.min():.4f}, {predictions.max():.4f}]")
    print(f"prediction mean: {predictions.mean():.4f}")
    print(f"prediction std: {predictions.std():.4f}")

    return DISC_grid, HEIGHT_grid, predictions_grid


def plot_residuals(y_true, y_pred, title):
    residuals = y_true - y_pred.flatten()

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="compressed")

    axes[0].scatter(y_pred, residuals, alpha=0.5)
    axes[0].axhline(y=0, color='r', linestyle='--')
    axes[0].set_title(f'{title} - residuals')
    axes[0].set_xlabel('Predicted')
    axes[0].set_ylabel('Residual')

    axes[1].hist(residuals, bins=30, alpha=0.7, edgecolor='black')
    axes[1].set_title(f'{title} - residual distribution')
    axes[1].set_xlabel('Residual')
    axes[1].set_ylabel('Count')

    plt.show()
