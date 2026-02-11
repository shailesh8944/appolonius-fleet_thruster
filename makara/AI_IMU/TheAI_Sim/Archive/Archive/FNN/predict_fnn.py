import os
import argparse
import pandas as pd
import numpy as np
import tensorflow as tf
import joblib
import matplotlib.pyplot as plt

# Define features - updated for the new model inputs
# Order for model input: pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, ang_vel_z, rudder_angle
IMU_RUDDER_FEATURES = ['accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle'] # Features from CSV for current step
DT = 0.1 # Constant time step

def perform_step_by_step_prediction_fnn_pos_vel(
    model, test_df, scaler_X, scaler_Y
):
    actual_positions_df = test_df[['pos_x', 'pos_y']].copy()
    imu_rudder_data_all_steps = test_df[IMU_RUDDER_FEATURES].values
    
    if len(test_df) < 2:
        raise ValueError("Test data needs at least 2 points to calculate initial velocity.")

    predicted_states_history = [] # Stores [pos_x, pos_y, vel_x, vel_y] for each predicted step
    
    # Initial state from the first row of test_df for position
    current_pos_x = actual_positions_df.iloc[0]['pos_x']
    current_pos_y = actual_positions_df.iloc[0]['pos_y']
    
    # Calculate initial velocity (vel_0) using the first two actual positions and DT
    current_vel_x = (actual_positions_df.iloc[1]['pos_x'] - current_pos_x) / DT
    current_vel_y = (actual_positions_df.iloc[1]['pos_y'] - current_pos_y) / DT

    # Store initial state (position part) for plotting
    predicted_positions_for_plot = [[current_pos_x, current_pos_y]] 

    # Loop to predict N-1 future states
    for i in range(len(test_df) - 1):
        current_imu_rudder = imu_rudder_data_all_steps[i] # IMU/Rudder data at time t_i
        
        # Prepare input for the model using state at t_i
        input_features_np = np.array([[
            current_pos_x, current_pos_y,
            current_vel_x, current_vel_y,
            current_imu_rudder[0], current_imu_rudder[1], # accel_x_i, accel_y_i
            current_imu_rudder[2],                       # ang_vel_z_i
            current_imu_rudder[3]                        # rudder_angle_i
        ]])
        scaled_input_features = scaler_X.transform(input_features_np)
        
        # Predict next state [next_pos_x, next_pos_y, next_vel_x, next_vel_y]
        scaled_predicted_next_state = model.predict(scaled_input_features, verbose=0)
        predicted_next_state = scaler_Y.inverse_transform(scaled_predicted_next_state)[0]
        
        # Update current state with the predicted next state
        current_pos_x = predicted_next_state[0]  # next_pos_x
        current_pos_y = predicted_next_state[1]  # next_pos_y
        current_vel_x = predicted_next_state[2]  # next_vel_x
        current_vel_y = predicted_next_state[3]  # next_vel_y
        
        predicted_positions_for_plot.append([current_pos_x, current_pos_y])
        # Optionally store full state: predicted_states_history.append(predicted_next_state)

    return np.array(predicted_positions_for_plot), actual_positions_df.values

def main():
    parser = argparse.ArgumentParser(description="Predict vessel trajectory using a trained FNN model that predicts next position and velocity.")
    parser.add_argument('--model_path', type=str, default='./FNN/model_pos_vel_predictor/fnn_pos_vel_best_model.keras',
                        help='Path to the trained FNN pos-vel model file (.keras).')
    parser.add_argument('--scaler_X_path', type=str, default='./FNN/model_pos_vel_predictor/fnn_scaler_X_pos_vel.joblib',
                        help='Path to the fnn_scaler_X_pos_vel.joblib file.')
    parser.add_argument('--scaler_Y_path', type=str, default='./FNN/model_pos_vel_predictor/fnn_scaler_Y_pos_vel.joblib',
                        help='Path to the fnn_scaler_Y_pos_vel.joblib file.')
    parser.add_argument('--test_data_path', type=str, default='../../extracted_data_noNoise/run3/synchronized.csv',
                        help='Path to the test synchronized.csv file.')
    parser.add_argument('--output_plot_path', type=str, default='./output/fnn_pos_vel_prediction.png',
                        help='Path to save the FNN (position-velocity prediction) plot.')

    args = parser.parse_args()
    os.makedirs(os.path.dirname(args.output_plot_path), exist_ok=True)

    print("Loading FNN position-velocity predictor model and scalers...")
    model = tf.keras.models.load_model(args.model_path)
    scaler_X = joblib.load(args.scaler_X_path)
    scaler_Y = joblib.load(args.scaler_Y_path)

    test_df = pd.read_csv(args.test_data_path)
    if len(test_df) < 2: # Need at least 2 for initial velocity calculation
        raise ValueError("Test data must contain at least 2 rows.")

    print("Performing step-by-step FNN position-velocity prediction...")
    predicted_positions, actual_positions = perform_step_by_step_prediction_fnn_pos_vel(
        model, test_df, scaler_X, scaler_Y
    )
    
    plt.figure(figsize=(12, 8))
    plt.plot(actual_positions[:, 0], actual_positions[:, 1], 'b-', label='Actual Trajectory', linewidth=2, alpha=0.7)
    plt.scatter(actual_positions[0, 0], actual_positions[0, 1], c='blue', marker='o', s=100, label='Actual Start')
    plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 'r--', label='FNN Predicted Trajectory (Pos-Vel Prediction)', linewidth=2)
    plt.scatter(predicted_positions[0, 0], predicted_positions[0, 1], c='red', marker='x', s=100, label='Predicted Start')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('FNN Model (Position-Velocity Prediction): Actual vs. Predicted Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(args.output_plot_path)
    print(f"FNN position-velocity prediction plot saved to {args.output_plot_path}")
    plt.show()

if __name__ == '__main__':
    main() 