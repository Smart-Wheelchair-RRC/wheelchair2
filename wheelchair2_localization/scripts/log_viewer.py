import numpy as np
import csv
import matplotlib.pyplot as plt
from sys import argv

def plot_all_trajectories(args: list[str]):
    CSV_FILE_PATH = 'processed_log.csv'
    if len(args) > 1:
        CSV_FILE_PATH = args[1]

    # Lists to hold the final, processed positions
    pose_positions, odom_positions, cmd_positions = [], [], []

    try:
        with open(CSV_FILE_PATH, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Read the pre-calculated position data for each trajectory
                pose_positions.append([float(row['pose_px']), float(row['pose_py'])])
                odom_positions.append([float(row['odom_px']), float(row['odom_py'])])
                cmd_positions.append([float(row['cmd_px']), float(row['cmd_py'])])

    except FileNotFoundError:
        print(f"Error: CSV file not found at '{CSV_FILE_PATH}'.")
        return
    except (Exception, ValueError) as e:
        print(f"Error reading CSV file: {e}")
        return

    # Convert lists to numpy arrays
    pose_positions = np.array(pose_positions)
    odom_positions = np.array(odom_positions)
    cmd_positions = np.array(cmd_positions)
    
    # --- Plotting ---
    plt.figure(figsize=(12, 10))
    
    plt.plot(cmd_positions[:, 0], cmd_positions[:, 1],
             label='Commanded Trajectory', color='green', linewidth=2, linestyle='--')

    plt.plot(pose_positions[:, 0], pose_positions[:, 1],
             label='Localization Pose (AMCL)', color='blue', alpha=0.8)

    plt.plot(odom_positions[:, 0], odom_positions[:, 1],
             label='Wheel Odometry', color='red', alpha=0.8)

    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Trajectory Comparison (Processed Data)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot_all_trajectories(argv)
