import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from sklearn.metrics import mean_squared_error

def evaluate_predictions(predictions_csv_path, plots_dir='./plots'):
    """
    Loads prediction results, calculates metrics, and generates plots.

    Args:
        predictions_csv_path (str): Path to the CSV file containing predictions and actual values.
        plots_dir (str): Directory to save the generated plots.
    """
    try:
        df = pd.read_csv(predictions_csv_path)
    except FileNotFoundError:
        print(f"Error: Predictions file not found at {predictions_csv_path}")
        return
    except Exception as e:
        print(f"Error reading predictions CSV: {e}")
        return

    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir, exist_ok=True)

    run_id = os.path.basename(predictions_csv_path).replace('_predictions.csv', '')

    # 1. Calculate Metrics
    mse_vx = mean_squared_error(df['actual_body_vx'], df['predicted_body_vx'])
    mse_vy = mean_squared_error(df['actual_body_vy'], df['predicted_body_vy'])
    
    final_actual_x = df['actual_pos_x'].iloc[-1]
    final_actual_y = df['actual_pos_y'].iloc[-1]
    final_pred_x = df['predicted_global_x'].iloc[-1]
    final_pred_y = df['predicted_global_y'].iloc[-1]
    
    final_position_error = np.sqrt((final_actual_x - final_pred_x)**2 + (final_actual_y - final_pred_y)**2)

    print(f"--- Evaluation Metrics for {run_id} ---")
    print(f"MSE for Body Velocity vx: {mse_vx:.6f}")
    print(f"MSE for Body Velocity vy: {mse_vy:.6f}")
    print(f"Final Position Error (Euclidean Distance): {final_position_error:.4f} meters (assuming units are meters)")

    # 2. Generate Plots
    time_steps = df.get('time_step', pd.Series(range(len(df)))) # Use time_step if available, else index

    # Plot Body Velocities (vx)
    plt.figure(figsize=(12, 6))
    plt.plot(time_steps, df['actual_body_vx'], label='Actual Body vx', color='blue')
    plt.plot(time_steps, df['predicted_body_vx'], label='Predicted Body vx', color='red', linestyle='--')
    plt.xlabel('Time Step (or Time in s if dt=0.1s)')
    plt.ylabel('Velocity vx (body frame)')
    plt.title(f'Actual vs. Predicted Body Velocity (vx) for {run_id}')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(plots_dir, f'{run_id}_body_vx_comparison.png'))
    plt.close()

    # Plot Body Velocities (vy)
    plt.figure(figsize=(12, 6))
    plt.plot(time_steps, df['actual_body_vy'], label='Actual Body vy', color='blue')
    plt.plot(time_steps, df['predicted_body_vy'], label='Predicted Body vy', color='red', linestyle='--')
    plt.xlabel('Time Step (or Time in s if dt=0.1s)')
    plt.ylabel('Velocity vy (body frame)')
    plt.title(f'Actual vs. Predicted Body Velocity (vy) for {run_id}')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(plots_dir, f'{run_id}_body_vy_comparison.png'))
    plt.close()

    # Plot Trajectory (Global X vs Y)
    plt.figure(figsize=(10, 8))
    plt.plot(df['actual_pos_x'], df['actual_pos_y'], label='Actual Trajectory', color='blue')
    plt.plot(df['predicted_global_x'], df['predicted_global_y'], label='Predicted Trajectory (Dead Reckoning)', color='red', linestyle='--')
    plt.scatter(df['actual_pos_x'].iloc[0], df['actual_pos_y'].iloc[0], marker='o', color='green', label='Start Point', s=100, zorder=5)
    plt.scatter(df['actual_pos_x'].iloc[-1], df['actual_pos_y'].iloc[-1], marker='x', color='black', label='Actual End Point', s=100, zorder=5)
    plt.scatter(df['predicted_global_x'].iloc[-1], df['predicted_global_y'].iloc[-1], marker='x', color='purple', label='Predicted End Point', s=100, zorder=5)
    plt.xlabel('Global X Position')
    plt.ylabel('Global Y Position')
    plt.title(f'Actual vs. Predicted Trajectory for {run_id}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # Ensure aspect ratio is equal for trajectory plots
    plt.savefig(os.path.join(plots_dir, f'{run_id}_trajectory_comparison.png'))
    plt.close()

    print(f"Plots saved to {plots_dir}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluate predictions and generate plots.')
    parser.add_argument('--predictions_csv', type=str, required=True, help='Path to the prediction CSV file (e.g., predictions/run4_predictions.csv).')
    parser.add_argument('--plots_dir', type=str, default='./plots', help='Directory to save the generated plots.')

    args = parser.parse_args()

    evaluate_predictions(
        predictions_csv_path=args.predictions_csv,
        plots_dir=args.plots_dir
    )
