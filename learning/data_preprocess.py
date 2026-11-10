import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

from data_cleaning import apply_dettach_exclusion


def moving_average(data, window_size):
    """Apply moving average smoothing with 'same' output length.
    
    Args:
        data: Input array
        window_size: Window width for moving average
    
    Returns:
        Smoothed array with same length as input
    """
    if window_size <= 1:
        return data
    
    from scipy.ndimage import uniform_filter1d
    # mode='nearest' repeats the boundary value at the edges, which avoids dilution
    return uniform_filter1d(data, size=window_size, mode='nearest')


def neck_data_preprocess(head1_point, head2_point, body3_point, tail4_point, tail5_point,
                  current_sample_rate, target_sample_rate, output_filename, smooth_window=5,
                  save_path=None):
    # Resample data if sample rates differ
    if current_sample_rate != target_sample_rate:

        # Calculate number of samples after resampling
        duration = head1_point.shape[0] / current_sample_rate
        new_num_samples = int(duration * target_sample_rate)

        # Warn if downsampling
        if target_sample_rate < current_sample_rate:
            print(f"WARNING: Downsampling from {current_sample_rate} Hz to {target_sample_rate} Hz. Data may be lost.")

        # Create time arrays
        old_time = np.linspace(0, duration, head1_point.shape[0])
        new_time = np.linspace(0, duration, new_num_samples)

        # Resample all point arrays using cubic interpolation
        head1_point_new = np.zeros((new_num_samples, 3))
        head2_point_new = np.zeros((new_num_samples, 3))
        body3_point_new = np.zeros((new_num_samples, 3))
        tail4_point_new = np.zeros((new_num_samples, 3))
        tail5_point_new = np.zeros((new_num_samples, 3))

        for i in range(3):
            f = interpolate.interp1d(old_time, head1_point[:, i], kind='cubic')
            head1_point_new[:, i] = f(new_time)

            f = interpolate.interp1d(old_time, head2_point[:, i], kind='cubic')
            head2_point_new[:, i] = f(new_time)

            f = interpolate.interp1d(old_time, body3_point[:, i], kind='cubic')
            body3_point_new[:, i] = f(new_time)

            f = interpolate.interp1d(old_time, tail4_point[:, i], kind='cubic')
            tail4_point_new[:, i] = f(new_time)

            f = interpolate.interp1d(old_time, tail5_point[:, i], kind='cubic')
            tail5_point_new[:, i] = f(new_time)

        # Update point arrays with resampled data
        head1_point = head1_point_new
        head2_point = head2_point_new
        body3_point = body3_point_new
        tail4_point = tail4_point_new
        tail5_point = tail5_point_new

    # Recalculate vectors after resampling
    head_vector = head1_point - head2_point
    body_vector = head2_point - body3_point

    for index in range(head_vector.shape[0]):
        norm = np.linalg.norm(head_vector[index, :])
        head_vector[index, :] = head_vector[index, :] / norm

    for index in range(body_vector.shape[0]):
        norm = np.linalg.norm(body_vector[index, :])
        body_vector[index, :] = body_vector[index, :] / norm

    temp_mul = head_vector * body_vector
    head_body_cos = np.sum(temp_mul, axis=1)
    head_body_angle = np.degrees(np.arccos(head_body_cos))
    # plt.plot(range(len(head_body_angle)), head_body_angle)
    # plt.show()

    temp_mul = head_vector
    for index in range(head_vector.shape[0]):
        temp_mul[index, :] = head_vector[index, :] * np.array([0, 1, 0])
    head_ground_sin = np.sum(temp_mul, axis=1)
    head_ground_angle = np.degrees(np.arcsin(head_ground_sin))
    # plt.plot(range(len(head_ground_angle)), head_ground_angle)
    # plt.show()

    # Create a figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(20, 16), layout="compressed")

    # Plot head_body_angle vs height
    axes[0, 0].scatter(head1_point[:, 1], head_body_angle, alpha=0.3, s=0.5)
    axes[0, 0].set_xlabel('Height', fontsize=12)
    axes[0, 0].set_ylabel('Head-Body Angle (degrees)', fontsize=12)
    axes[0, 0].set_title('Head-Body Angle vs Height', fontsize=14)
    axes[0, 0].grid(True, alpha=0.3)

    # Plot head_ground_angle vs height
    axes[0, 1].scatter(head1_point[:, 1], head_ground_angle, alpha=0.3, s=0.5)
    axes[0, 1].set_xlabel('Height', fontsize=12)
    axes[0, 1].set_ylabel('Head-Ground Angle (degrees)', fontsize=12)
    axes[0, 1].set_title('Head-Ground Angle vs Height', fontsize=14)
    axes[0, 1].grid(True, alpha=0.3)

    # Plot height vs time
    time_array = np.arange(len(head1_point)) / target_sample_rate
    axes[1, 0].plot(time_array, head1_point[:, 1], linewidth=0.5)
    axes[1, 0].set_xlabel('Time (s)', fontsize=12)
    axes[1, 0].set_ylabel('Height', fontsize=12)
    axes[1, 0].set_title('Height vs Time', fontsize=14)
    axes[1, 0].grid(True, alpha=0.3)

    # Plot head_body_angle vs head_ground_angle
    axes[1, 1].scatter(head_ground_angle, head_body_angle, alpha=0.3, s=0.5)
    axes[1, 1].set_xlabel('Head-Ground Angle (degrees)', fontsize=12)
    axes[1, 1].set_ylabel('Head-Body Angle (degrees)', fontsize=12)
    axes[1, 1].set_title('Head-Body Angle vs Head-Ground Angle', fontsize=14)
    axes[1, 1].grid(True, alpha=0.3)

    # when save_path is given, write the file first and show afterwards (plt.show() is a
    # no-op in a headless batch run)
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"diagnostic figure written: {save_path}")
    plt.show()


    # Print statistical information
    print(f"\n{'='*60}")
    print(f"Processing file: {output_filename}")
    print(f"Input sample rate:  {current_sample_rate} Hz")
    print(f"Output sample rate: {target_sample_rate} Hz")
    print(f"Total samples:      {len(head_body_angle)}")
    print(f"\nHead-Body Angle Statistics:")
    print(f"  Range: [{np.min(head_body_angle):.2f}°, {np.max(head_body_angle):.2f}°]")
    print(f"\nHead-Ground Angle Statistics:")
    print(f"  Range: [{np.min(head_ground_angle):.2f}°, {np.max(head_ground_angle):.2f}°]")
    print(f"\nHead Height Statistics:")
    print(f"  Range: [{np.min(head1_point[:, 1]):.2f}, {np.max(head1_point[:, 1]):.2f}]")
    print(f"{'='*60}\n")

    

            # Calculate velocity and acceleration for DISC (head_ground_angle)
    disc_velocity = np.zeros(len(head_ground_angle))
    disc_acceleration = np.zeros(len(head_ground_angle))
    disc_velocity[1:] = np.diff(head_ground_angle) * target_sample_rate
    # Apply moving average smoothing to velocity
    disc_velocity = moving_average(disc_velocity, smooth_window)
    disc_acceleration[2:] = np.diff(disc_velocity[1:]) * target_sample_rate
    # Apply moving average smoothing to acceleration
    disc_acceleration = moving_average(disc_acceleration, smooth_window)

    # Calculate velocity and acceleration for HEIGHT (head1_point[:, 1])
    height_velocity = np.zeros(len(head1_point))
    height_acceleration = np.zeros(len(head1_point))
    height_velocity[1:] = np.diff(head1_point[:, 1]) * target_sample_rate
    # Apply moving average smoothing to velocity
    height_velocity = moving_average(height_velocity, smooth_window)
    height_acceleration[2:] = np.diff(height_velocity[1:]) * target_sample_rate
    # Apply moving average smoothing to acceleration
    height_acceleration = moving_average(height_acceleration, smooth_window)

    # Write header rows
    rows = []
    rows.append(f"Samplerate,{target_sample_rate}")
    rows.append(f"Points,{len(head_body_angle)}")
    rows.append("NECK,DISC,HEIGHT,DISC_VELOCITY,DISC_ACCELERATION,HEIGHT_VELOCITY,HEIGHT_ACCELERATION")
    # Write data rows
    for index in range(len(head_body_angle)):
        rows.append(f"{head_body_angle[index]},{head_ground_angle[index]},{head1_point[index, 1]},{disc_velocity[index]},{disc_acceleration[index]},{height_velocity[index]},{height_acceleration[index]}")

    result = "\n".join(rows)

    # Write to CSV file
    with open(output_filename, 'w') as f:
        f.write(result)

    return result


def neck_data_preprocess2(head_body_angle, head_ground_angle, head_height,
                  target_sample_rate, output_filename, smooth_window=1, save_path=None):
    plt.plot(range(len(head_body_angle)), head_body_angle)
    # plt.show()

    plt.plot(range(len(head_ground_angle)), head_ground_angle)
    # plt.show()


    # Create a figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(20, 16), layout="compressed")

    # Plot head_body_angle vs height
    axes[0, 0].scatter(head_height, head_body_angle, alpha=0.3, s=0.5)
    axes[0, 0].set_xlabel('Height', fontsize=12)
    axes[0, 0].set_ylabel('Head-Body Angle (degrees)', fontsize=12)
    axes[0, 0].set_title('Head-Body Angle vs Height', fontsize=14)
    axes[0, 0].grid(True, alpha=0.3)

    # Plot head_ground_angle vs height
    axes[0, 1].scatter(head_height, head_ground_angle, alpha=0.3, s=0.5)
    axes[0, 1].set_xlabel('Height', fontsize=12)
    axes[0, 1].set_ylabel('Head-Ground Angle (degrees)', fontsize=12)
    axes[0, 1].set_title('Head-Ground Angle vs Height', fontsize=14)
    axes[0, 1].grid(True, alpha=0.3)

    # Plot height vs time
    time_array = np.arange(len(head_height)) / target_sample_rate
    axes[1, 0].plot(time_array, head_height, linewidth=0.5)
    axes[1, 0].set_xlabel('Time (s)', fontsize=12)
    axes[1, 0].set_ylabel('Height', fontsize=12)
    axes[1, 0].set_title('Height vs Time', fontsize=14)
    axes[1, 0].grid(True, alpha=0.3)

    # Plot head_body_angle vs head_ground_angle
    axes[1, 1].scatter(head_ground_angle, head_body_angle, alpha=0.3, s=0.5)
    axes[1, 1].set_xlabel('Head-Ground Angle (degrees)', fontsize=12)
    axes[1, 1].set_ylabel('Head-Body Angle (degrees)', fontsize=12)
    axes[1, 1].set_title('Head-Body Angle vs Head-Ground Angle', fontsize=14)
    axes[1, 1].grid(True, alpha=0.3)

    # when save_path is given, write the file first and show afterwards (plt.show() is a
    # no-op in a headless batch run)
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"diagnostic figure written: {save_path}")
    plt.show()

    # Print statistical information
    print(f"\n{'='*60}")
    print(f"Processing file: {output_filename}")
    print(f"Output sample rate: {target_sample_rate} Hz")
    print(f"Total samples:      {len(head_body_angle)}")
    print(f"\nHead-Body Angle Statistics:")
    print(f"  Range: [{np.min(head_body_angle):.2f}°, {np.max(head_body_angle):.2f}°]")
    print(f"\nHead-Ground Angle Statistics:")
    print(f"  Range: [{np.min(head_ground_angle):.2f}°, {np.max(head_ground_angle):.2f}°]")
    print(f"\nHead Height Statistics:")
    print(f"  Range: [{np.min(head_height):.2f}, {np.max(head_height):.2f}]")
    print(f"{'='*60}\n")

    
    # Calculate velocity and acceleration for DISC (head_ground_angle)
    disc_velocity = np.zeros(len(head_ground_angle))
    disc_acceleration = np.zeros(len(head_ground_angle))
    disc_velocity[1:] = np.diff(head_ground_angle) * target_sample_rate
    # Apply moving average smoothing to velocity
    disc_velocity = moving_average(disc_velocity, smooth_window)
    disc_acceleration[2:] = np.diff(disc_velocity[1:]) * target_sample_rate
    # Apply moving average smoothing to acceleration
    disc_acceleration = moving_average(disc_acceleration, smooth_window)

    # Calculate velocity and acceleration for HEIGHT (head_height)
    height_velocity = np.zeros(len(head_height))
    height_acceleration = np.zeros(len(head_height))
    height_velocity[1:] = np.diff(head_height) * target_sample_rate
    # Apply moving average smoothing to velocity
    height_velocity = moving_average(height_velocity, smooth_window)
    height_acceleration[2:] = np.diff(height_velocity[1:]) * target_sample_rate
    # Apply moving average smoothing to acceleration
    height_acceleration = moving_average(height_acceleration, smooth_window)

    # Write header rows
    rows = []
    rows.append(f"Samplerate,{target_sample_rate}")
    rows.append(f"Points,{len(head_body_angle)}")
    rows.append("NECK,DISC,HEIGHT,DISC_VELOCITY,DISC_ACCELERATION,HEIGHT_VELOCITY,HEIGHT_ACCELERATION")
    # Write data rows
    for index in range(len(head_body_angle)):
        rows.append(f"{head_body_angle[index]},{head_ground_angle[index]},{head_height[index]},{disc_velocity[index]},{disc_acceleration[index]},{height_velocity[index]},{height_acceleration[index]}")

    result = "\n".join(rows)

    # Write to CSV file
    with open(output_filename, 'w') as f:
        f.write(result)

    return result


def neck_data_preprocess3(head_body_angle, head_ground_angle, head_height,
                          current_sample_rate, target_sample_rate, output_filename, smooth_window=1,
                          save_path=None):
    # Interpolation
    original_length = len(head_body_angle)
    original_time = np.linspace(0, original_length / current_sample_rate, original_length)
    target_length = int(original_length * target_sample_rate / current_sample_rate)
    target_time = np.linspace(0, original_length / current_sample_rate, target_length)

    head_body_angle = interpolate.interp1d(original_time, head_body_angle, kind='cubic')(target_time)
    head_ground_angle = interpolate.interp1d(original_time, head_ground_angle, kind='cubic')(target_time)
    head_height = interpolate.interp1d(original_time, head_height, kind='cubic')(target_time)

    return neck_data_preprocess2(head_body_angle, head_ground_angle, head_height,
                                 target_sample_rate, output_filename, smooth_window,
                                 save_path=save_path)

def process_tail_data():
    file = pd.read_csv("data/raw/data231015.csv")
    file = file.iloc[1:, :]
    file = file[~file['head1'].isin(['-1'])]

    head1_point = np.array(file.iloc[:, 2:5].astype("float32"))
    head2_point = np.array(file.iloc[:, 5:8].astype("float32"))
    body3_point = np.array(file.iloc[:, 8:11].astype("float32"))
    tail4_point = np.array(file.iloc[:, 11:14].astype("float32"))
    tail5_point = np.array(file.iloc[:, 14:].astype("float32"))

    head_vector = head1_point - head2_point
    body_vector = head2_point - body3_point
    tail1_vector = body3_point - tail4_point
    tail2_vector = tail4_point - tail5_point


    for index in range(body_vector.shape[0]):
        norm = np.linalg.norm(body_vector[index, :])
        body_vector[index, :] = body_vector[index, :] / norm

    for index in range(tail1_vector.shape[0]):
        norm = np.linalg.norm(tail1_vector[index, :])
        tail1_vector[index, :] = tail1_vector[index, :] / norm

    for index in range(tail2_vector.shape[0]):
        norm = np.linalg.norm(tail2_vector[index, :])
        tail2_vector[index, :] = tail2_vector[index, :] / norm

    temp_mul = body_vector * tail1_vector
    tail1_cos = np.sum(temp_mul, axis=1)
    tail1_angle = np.arccos(tail1_cos)
    plt.plot(range(len(tail1_angle)), tail1_angle)
    plt.show()

    temp_mul = tail2_vector * tail1_vector
    tail2_cos = np.sum(temp_mul, axis=1)
    tail2_angle = np.arccos(tail2_cos)
    plt.plot(range(len(tail2_angle)), tail2_angle)
    plt.show()


if __name__  == "__main__":
    # Attachment: link angles are reconstructed from the tracked landmark points.
    file = pd.read_csv("data/raw/data231015.csv")
    file = file.iloc[1:, :]
    file = file[~file.iloc[:, 2].isin(['-1'])]

    head1_point = np.array(file.iloc[:, 2:5].astype("float32"))
    head2_point = np.array(file.iloc[:, 5:8].astype("float32"))
    body3_point = np.array(file.iloc[:, 8:11].astype("float32"))
    tail4_point = np.array(file.iloc[:, 11:14].astype("float32"))
    tail5_point = np.array(file.iloc[:, 14:17].astype("float32"))
    neck_data_preprocess(head1_point, head2_point, body3_point, tail4_point, tail5_point,
                        current_sample_rate=25, target_sample_rate=250, output_filename="data/processed/attach23_01.csv",
                        save_path="figures/raw_attach23_01.png")

    # Detachment: the link angles are already present in the exports.
    for i in (1, 2, 3):
        file4 = pd.read_excel(f"data/raw/dettach-{i}.xlsx", header=None)
        file4 = file4.iloc[1:]
        file4 = file4[~file4.iloc[:, 1].isin([-1])]
        file4 = file4.dropna(subset=[file4.columns[2]])

        # column A = HEIGHT, column B = DISC, column C = NECK
        head_height = np.array(file4.iloc[:, 0].astype("float32"))
        head_ground_angle = np.array(file4.iloc[:, 1].astype("float32"))
        head_body_angle = np.array(file4.iloc[:, 2].astype("float32"))

        # DISC is folded from [0, 180] into [-90, 90]
        head_ground_angle = np.where(head_ground_angle > 90, head_ground_angle - 180, head_ground_angle)

        neck_data_preprocess3(head_body_angle, head_ground_angle, head_height,
                              current_sample_rate=25, target_sample_rate=250,
                              output_filename=f"data/processed/dettach{i}.csv",
                              save_path=f"figures/raw_dettach{i}.png")

        # Outlier and wall-hugging segments removed; the raw dettach*.csv above are
        # kept as produced. Exclusion rules and hit guard live in data_cleaning.py.
        head_height, head_ground_angle, head_body_angle = apply_dettach_exclusion(
            i, head_height, head_ground_angle, head_body_angle)

        neck_data_preprocess3(head_body_angle, head_ground_angle, head_height,
                              current_sample_rate=25, target_sample_rate=250,
                              output_filename=f"data/processed/dettach{i}_clean.csv",
                              save_path=f"figures/raw_dettach{i}_clean.png")

