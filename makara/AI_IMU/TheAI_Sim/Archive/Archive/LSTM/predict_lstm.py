import os
import argparse
import pandas as pd
import numpy as np
import tensorflow as tf
import joblib
import matplotlib.pyplot as plt
from collections import deque

# Input features for the LSTM model (used to prepare data for the history deque)
MODEL_INPUT_FEATURES_ORDER = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
# Features to be read directly from the test CSV for each step
IMU_RUDDER_FEATURES_FROM_CSV = ['accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
DT = 0.1 # Constant time step for position integration

def perform_step_by_step_prediction_lstm_velocity(
    model_vx, model_vy, test_df, 
    scaler_X, scaler_y_vel_x, scaler_y_vel_y, 
    sequence_length
):
    actual_positions_df = test_df[['pos_x', 'pos_y']].copy()
    imu_rudder_ground_truth = test_df[IMU_RUDDER_FEATURES_FROM_CSV].values
    
    predicted_positions_list = [] 

    if len(test_df) < sequence_length + 1: # Need enough for initial history and at least one prediction
        raise ValueError(f"Test data too short ({len(test_df)}) for LSTM sequence processing (needs {sequence_length + 1}).")

    # History deque will store scaled MODEL_INPUT_FEATURES_ORDER
    history = deque(maxlen=sequence_length)

    # Initialize history with the first `sequence_length` steps from ground truth
    # Also, store the ground truth positions for this initial period for the plot.
    for k in range(sequence_length):
        # Calculate vel_x_k, vel_y_k using pos_{k+1} and pos_k from test_df
        # This is the velocity *during* the interval ending at pos_{k+1}
        # For the input at step k, we need vel_k (derived from pos_k and pos_{k+1})
        vel_x_k = (test_df.iloc[k+1]['pos_x'] - test_df.iloc[k]['pos_x']) / DT
        vel_y_k = (test_df.iloc[k+1]['pos_y'] - test_df.iloc[k]['pos_y']) / DT
        
        current_imu_rudder_k = imu_rudder_ground_truth[k]
        
        features_for_history_k = np.array([[
            vel_x_k, vel_y_k,
            current_imu_rudder_k[0], current_imu_rudder_k[1], # accel_x, accel_y
            current_imu_rudder_k[2],                       # ang_vel_z
            current_imu_rudder_k[3]                        # rudder_angle
        ]])
        scaled_features_k = scaler_X.transform(features_for_history_k)
        history.append(scaled_features_k.flatten())
        
        # Store actual positions for the initial sequence part of the plot
        predicted_positions_list.append([test_df.iloc[k]['pos_x'], test_df.iloc[k]['pos_y']])

    # Initial state for iterative prediction loop (state at the END of the initial sequence_length period)
    current_pos_x = test_df.iloc[sequence_length-1]['pos_x']
    current_pos_y = test_df.iloc[sequence_length-1]['pos_y']
    
    # current_vel_x/y is the velocity that was *part of the last entry added to history*.
    # This is vel_{sequence_length-1}, calculated from pos_{sequence_length-1} and pos_{sequence_length}
    current_vel_x = (test_df.iloc[sequence_length]['pos_x'] - current_pos_x) / DT 
    current_vel_y = (test_df.iloc[sequence_length]['pos_y'] - current_pos_y) / DT
    
    # The loop starts predicting from step `sequence_length` onwards.
    # The first prediction will be for next_vel at `sequence_length`, using history up to `sequence_length-1`.
    # This predicted next_vel is then used to calculate pos at `sequence_length`.

    # Add the last known ground truth position before prediction starts
    # predicted_positions_list already has positions up to index sequence_length-1
    # The first predicted position will correspond to index sequence_length in test_df

    for i in range(sequence_length, len(test_df)):
        # The input to the LSTM for prediction is the current state of `history`
        model_input_sequence = np.array(list(history)).reshape(1, sequence_length, len(MODEL_INPUT_FEATURES_ORDER))
        
        # Predict next_vel_x and next_vel_y (these are vel_{i})
        scaled_predicted_next_vel_x = model_vx.predict(model_input_sequence, verbose=0)
        predicted_next_vel_x = scaler_y_vel_x.inverse_transform(scaled_predicted_next_vel_x)[0,0]
        
        scaled_predicted_next_vel_y = model_vy.predict(model_input_sequence, verbose=0)
        predicted_next_vel_y = scaler_y_vel_y.inverse_transform(scaled_predicted_next_vel_y)[0,0]
        
        # Integrate position using current_vel_x, current_vel_y (which are effectively vel_{i-1})
        # pos_i = pos_{i-1} + vel_{i-1} * DT
        current_pos_x += current_vel_x * DT
        current_pos_y += current_vel_y * DT
        predicted_positions_list.append([current_pos_x, current_pos_y])

        # Update current velocities to the predicted ones for the *next* iteration's input and history update
        current_vel_x = predicted_next_vel_x
        current_vel_y = predicted_next_vel_y
        
        # Prepare features for the next history update using new current_vel_x/y and IMU from ground truth at step i
        # This represents the state at time i (vel_i, accel_i, etc.)
        current_imu_rudder_i = imu_rudder_ground_truth[i]
        features_for_history_update_np = np.array([[
            current_vel_x, current_vel_y, 
            current_imu_rudder_i[0], current_imu_rudder_i[1],
            current_imu_rudder_i[2], current_imu_rudder_i[3]
        ]])
        scaled_features_for_history_update = scaler_X.transform(features_for_history_update_np)
        history.append(scaled_features_for_history_update.flatten())

    return np.array(predicted_positions_list), actual_positions_df.values

def main():
    parser = argparse.ArgumentParser(description="Predict vessel trajectory using trained Decoupled LSTM models (predicting next velocity).")
    parser.add_argument('--model_vx_path', type=str, default='./Model/BiLSTM_vel_predictor_decoupled/lstm_next_vx_best_model.keras',
                        help='Path to the trained LSTM next_vx model file (.keras).')
    parser.add_argument('--model_vy_path', type=str, default='./Model/BiLSTM_vel_predictor_decoupled/lstm_next_vy_best_model.keras',
                        help='Path to the trained LSTM next_vy model file (.keras).')
    parser.add_argument('--scaler_X_path', type=str, default='./Model/BiLSTM_vel_predictor_decoupled/lstm_scaler_X_vel_input.joblib',
                        help='Path to the common LSTM scaler_X_vel_input.joblib file.')
    parser.add_argument('--scaler_y_vx_path', type=str, default='./Model/BiLSTM_vel_predictor_decoupled/lstm_next_vx_scaler_y.joblib',
                        help='Path to the LSTM next_vx_scaler_y.joblib file.')
    parser.add_argument('--scaler_y_vy_path', type=str, default='./Model/BiLSTM_vel_predictor_decoupled/lstm_next_vy_scaler_y.joblib',
                        help='Path to the LSTM next_vy_scaler_y.joblib file.')
    parser.add_argument('--test_data_path', type=str, default='../../extracted_data_noNoise/run3/synchronized.csv',
                        help='Path to the test synchronized.csv file.')
    parser.add_argument('--sequence_length', type=int, default=10,
                        help='Sequence length used for LSTM model (must match training).')
    parser.add_argument('--output_plot_path', type=str, default='./output/lstm_decoupled_velocity_prediction.png',
                        help='Path to save the Decoupled LSTM (velocity prediction) plot.')

    args = parser.parse_args()
    os.makedirs(os.path.dirname(args.output_plot_path), exist_ok=True)

    print("Loading Decoupled LSTM velocity predictor models and scalers...")
    model_vx = tf.keras.models.load_model(args.model_vx_path)
    model_vy = tf.keras.models.load_model(args.model_vy_path)
    scaler_X = joblib.load(args.scaler_X_path)
    scaler_y_vel_x = joblib.load(args.scaler_y_vx_path)
    scaler_y_vel_y = joblib.load(args.scaler_y_vy_path)

    test_df = pd.read_csv(args.test_data_path)
    if len(test_df) < args.sequence_length + 1: # Ensure enough data for history init + 1 step for vel calc
        raise ValueError(f"Test data too short (has {len(test_df)} rows, needs at least {args.sequence_length + 1}).")

    print("Performing step-by-step Decoupled LSTM velocity prediction and integration...")
    predicted_positions, actual_positions = perform_step_by_step_prediction_lstm_velocity(
        model_vx, model_vy, test_df, 
        scaler_X, scaler_y_vel_x, scaler_y_vel_y, 
        args.sequence_length
    )
    
    plt.figure(figsize=(12, 8))
    plt.plot(actual_positions[:, 0], actual_positions[:, 1], 'b-', label='Actual Trajectory', linewidth=2, alpha=0.7)
    plt.scatter(actual_positions[0, 0], actual_positions[0, 1], c='blue', marker='o', s=100, label='Actual Start')
    plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 'r--', label='Decoupled LSTM Predicted Trajectory (Velocity Prediction)', linewidth=2)
    plt.scatter(predicted_positions[0, 0], predicted_positions[0, 1], c='red', marker='x', s=100, label='Predicted Start (from history)')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Decoupled LSTM Model (Velocity Prediction & Integration): Actual vs. Predicted Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(args.output_plot_path)
    print(f"Decoupled LSTM velocity prediction plot saved to {args.output_plot_path}")
    plt.show()

if __name__ == '__main__':
    main() 