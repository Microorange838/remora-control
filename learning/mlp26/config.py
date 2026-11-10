"""Shared configuration for the mlp26 package.

The matplotlib rcParams are set at import time, so any entry point that produces a
figure must import this module first.
"""

import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 20
plt.rcParams['axes.titlesize'] = 20
plt.rcParams['axes.labelsize'] = 20
plt.rcParams['xtick.labelsize'] = 20
plt.rcParams['ytick.labelsize'] = 20
plt.rcParams['legend.fontsize'] = 20

# Input dimensionality of the two networks: the two-input visualisation network and
# the six-input controller that is exported for the robot.
SIMPLE_MLP_N_INPUT_DIM = 2   # [DISC, HEIGHT] -> NECK
PARAM_MLP_N_INPUT_DIM = 6    # [DISC, HEIGHT, DISC_VELOCITY, DISC_ACCELERATION, HEIGHT_VELOCITY, HEIGHT_ACCELERATION] -> NECK

# Column order of the processed CSV files written by learning/data_preprocess.py.
# Column 0 is the target; the remaining six are the controller inputs.
TARGET_COLUMN = "NECK"
FEATURE_COLUMNS = ["DISC", "HEIGHT",
                   "DISC_VELOCITY", "DISC_ACCELERATION",
                   "HEIGHT_VELOCITY", "HEIGHT_ACCELERATION"]

# Training sequences, one behaviour per run.
DEFAULT_DATA_FILES = [
    "data/processed/attach23_01.csv",
]

DETACHMENT_DATA_FILES = [
    "data/processed/dettach1_clean.csv",
    "data/processed/dettach2_clean.csv",
    "data/processed/dettach3_clean.csv",
]

# Both behaviours pooled, for the joint-training comparison (--mix).
MIXED_DATA_FILES = DEFAULT_DATA_FILES + DETACHMENT_DATA_FILES

# Title of the decision-space figure produced from the two-input network.
HEATMAP_TITLE = "Neck angle decision space"
