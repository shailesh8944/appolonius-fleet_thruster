import torch
import numpy as np
import pandas as pd
import os
import argparse
import json

from model import StatePredictorFNN
from data_loader import VesselMotionDataset 
from utils import body_to_global_velocity, integrate_position

def predict_trajectory(data_dir, run_id, model_path, scaler_params_path, output_dir='./predictions'):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # Load scaler parameters
    try:
        with open(scaler_params_path, 'r') as f:
            scaler_params = json.load(f)
        print(f"Loaded scaler parameters from {scaler_params_path}")
        
        # Convert scaler params to tensors on the correct device
        input_means_tensor = torch.tensor(scaler_params['input_means'], dtype=torch.float32).to(device)
        input_stds_tensor = torch.tensor(scaler_params['input_stds'], dtype=torch.float32).to(device)
        # These are means/stds of the DELTA targets
        output_delta_means_tensor = torch.tensor(scaler_params['target_means'], dtype=torch.float32).to(device)
        output_delta_stds_tensor = torch.tensor(scaler_params['target_stds'], dtype=torch.float32).to(device)

    except FileNotFoundError:
        print(f"Error: Scaler params file not found at {scaler_params_path}")
        return
    except Exception as e:
        print(f"Error loading or processing scaler_params: {e}")
        return

    try:
        # Initialize dataset with scaler_params for the specific run
        dataset = VesselMotionDataset(data_dir=data_dir, specific_run=run_id, scaler_params=scaler_params)
        if len(dataset.run_indices) == 0 or dataset.run_indices[0]['run_name'] != run_id:
             raise ValueError(f"Run {run_id} could not be loaded by VesselMotionDataset.")
    except Exception as e:
        print(f"Error loading dataset for run {run_id}: {e}")
        return

    # Get tensors for prediction from the dataset
    initial_model_input_t0, full_run_exogenous_inputs_raw, initial_state_t0, actual_original_outputs_and_pos = dataset.get_run_data_tensors_for_prediction(run_id)
    # initial_state_t0 contains: vx_0, vy_0, yaw_0 (original), pos_x_0, pos_y_0
    # actual_original_outputs_and_pos contains: vx, vy, yaw (original), pos_x, pos_y for the whole run
    # full_run_exogenous_inputs_raw contains unscaled ax, ay, wz, rudder_angle

    num_timesteps = full_run_exogenous_inputs_raw.shape[0]
    dt = dataset.dt

    num_input_features = len(dataset.get_input_feature_names()) # Should be 8
    num_output_features = len(dataset.output_features_delta)    # Should be 3
    model = StatePredictorFNN(input_size=num_input_features, output_size=num_output_features).to(device)
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
    except FileNotFoundError:
        print(f"Error: Model file not found at {model_path}")
        return
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    model.eval()

    print(f"Performing delta prediction for {run_id} with {num_timesteps} timesteps.")

    # Initialize current state with values from t=0
    current_vx_body = initial_state_t0[0].item()
    current_vy_body = initial_state_t0[1].item()
    current_yaw = initial_state_t0[2].item() # Original yaw at t=0
    current_x_global = initial_state_t0[3].item()
    current_y_global = initial_state_t0[4].item()

    # Lists to store predicted states (absolute values)
    predicted_body_vx_abs = [current_vx_body]
    predicted_body_vy_abs = [current_vy_body]
    predicted_yaw_abs = [current_yaw]
    predicted_global_x = [current_x_global]
    predicted_global_y = [current_y_global]

    # Lists to store predicted deltas and inputs used
    predicted_delta_vx = []
    predicted_delta_vy = []
    predicted_delta_yaw = []
    input_accel_x = []
    input_accel_y = []
    input_ang_vel_z = []

    # The very first input to the model uses actual values from t=0, already scaled by VesselMotionDataset
    current_model_input_tensor = initial_model_input_t0.unsqueeze(0).to(device)

    # Store inputs corresponding to the first state (t=0)
    # These are used to predict the state at t=1
    initial_exog_inputs = full_run_exogenous_inputs_raw[0] # Raw ax, ay, wz, rudder at t=0
    input_accel_x.append(initial_exog_inputs[0].item())
    input_accel_y.append(initial_exog_inputs[1].item())
    input_ang_vel_z.append(initial_exog_inputs[2].item())

    with torch.no_grad():
        for t in range(num_timesteps - 1):
            # Predict SCALED deltas for t+1 based on SCALED input at t
            predicted_deltas_scaled_tensor = model(current_model_input_tensor) 
            
            # Unscale the predicted deltas
            # Ensure broadcasting is correct if means/stds are 1D and prediction is [1, num_features]
            predicted_deltas_unscaled_tensor = predicted_deltas_scaled_tensor * output_delta_stds_tensor + output_delta_means_tensor
            
            delta_vx_pred = predicted_deltas_unscaled_tensor[0, 0].item()
            delta_vy_pred = predicted_deltas_unscaled_tensor[0, 1].item()
            delta_yaw_pred = predicted_deltas_unscaled_tensor[0, 2].item()

            # Store the predicted deltas
            predicted_delta_vx.append(delta_vx_pred)
            predicted_delta_vy.append(delta_vy_pred)
            predicted_delta_yaw.append(delta_yaw_pred)

            # Update state by adding predicted UN SCALED deltas
            current_vx_body += delta_vx_pred
            current_vy_body += delta_vy_pred
            current_yaw += delta_yaw_pred 
            current_yaw = (current_yaw + np.pi) % (2 * np.pi) - np.pi # Normalize yaw

            predicted_body_vx_abs.append(current_vx_body)
            predicted_body_vy_abs.append(current_vy_body)
            predicted_yaw_abs.append(current_yaw)

            vx_global_pred, vy_global_pred = body_to_global_velocity(
                current_vx_body, current_vy_body, current_yaw # Use updated (absolute) yaw
            )
            current_x_global, current_y_global = integrate_position(
                current_x_global, current_y_global, 
                vx_global_pred, vy_global_pred, dt
            )
            predicted_global_x.append(current_x_global)
            predicted_global_y.append(current_y_global)

            # Prepare input for the next step (predicting for t+2 based on state t+1)
            if t + 1 < num_timesteps:
                ax_next_exog = full_run_exogenous_inputs_raw[t + 1, 0].item() # Raw exogenous
                ay_next_exog = full_run_exogenous_inputs_raw[t + 1, 1].item() # Raw exogenous
                wz_next_exog = full_run_exogenous_inputs_raw[t + 1, 2].item() # Raw exogenous
                rudder_next_exog = full_run_exogenous_inputs_raw[t + 1, 3].item() # Raw exogenous
                
                # Store the inputs that will be used in the *next* iteration (for predicting t+2)
                input_accel_x.append(ax_next_exog)
                input_accel_y.append(ay_next_exog)
                input_ang_vel_z.append(wz_next_exog)

                sin_yaw_next = np.sin(current_yaw) # Use the NEWLY PREDICTED (absolute) yaw
                cos_yaw_next = np.cos(current_yaw)

                # Assemble the RAW feature vector for the next input
                next_model_input_features_raw = torch.tensor([
                    ax_next_exog, ay_next_exog, wz_next_exog,
                    current_vx_body,    # Updated absolute vx (unscaled)
                    current_vy_body,    # Updated absolute vy (unscaled)
                    sin_yaw_next,       # sin of updated absolute yaw (unscaled)
                    cos_yaw_next,       # cos of updated absolute yaw (unscaled)
                    rudder_next_exog
                ], dtype=torch.float32).to(device)
                
                # Scale the raw feature vector
                next_model_input_features_scaled = (next_model_input_features_raw - input_means_tensor) / input_stds_tensor
                current_model_input_tensor = next_model_input_features_scaled.unsqueeze(0)
            else:
                break 

    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    output_file_path = os.path.join(output_dir, f'{run_id}_delta_predictions.csv')
    
    num_predicted_steps = len(predicted_global_x)
    df_data = {
        'time_step': np.arange(num_predicted_steps) * dt,
        # Actual values are original (absolute) values for comparison
        'actual_pos_x': actual_original_outputs_and_pos[:num_predicted_steps, 3].cpu().numpy(),
        'actual_pos_y': actual_original_outputs_and_pos[:num_predicted_steps, 4].cpu().numpy(),
        'actual_yaw': actual_original_outputs_and_pos[:num_predicted_steps, 2].cpu().numpy(),
        'actual_body_vx': actual_original_outputs_and_pos[:num_predicted_steps, 0].cpu().numpy(),
        'actual_body_vy': actual_original_outputs_and_pos[:num_predicted_steps, 1].cpu().numpy(),
        
        'predicted_body_vx': np.array(predicted_body_vx_abs),
        'predicted_body_vy': np.array(predicted_body_vy_abs),
        'predicted_yaw': np.array(predicted_yaw_abs),
        'predicted_global_x': np.array(predicted_global_x),
        'predicted_global_y': np.array(predicted_global_y),

        # Add predicted deltas (note: length is num_steps - 1)
        'predicted_delta_vx': np.array(predicted_delta_vx),
        'predicted_delta_vy': np.array(predicted_delta_vy),
        'predicted_delta_yaw': np.array(predicted_delta_yaw),

        # Add input accelerations/angular velocity (note: length should match num_predicted_steps)
        'input_accel_x': np.array(input_accel_x),
        'input_accel_y': np.array(input_accel_y),
        'input_ang_vel_z': np.array(input_ang_vel_z)
    }

    # Adjust lengths: Predicted states have T steps, predicted deltas have T-1 steps.
    # Inputs correspond to the prediction time step (input at t predicts state at t+1)
    min_len = min(len(predicted_global_x), len(input_accel_x)) # Should be T
    min_len_delta = len(predicted_delta_vx) # Should be T-1

    df_data_aligned = {}
    for key, value in df_data.items():
        if key in ['predicted_delta_vx', 'predicted_delta_vy', 'predicted_delta_yaw']:
            # Pad deltas at the beginning (no delta for t=0)
            df_data_aligned[key] = np.pad(np.array(value), (1, 0), constant_values=np.nan)[:min_len]
        elif len(value) == min_len:
             df_data_aligned[key] = np.array(value)
        else:
             # Handle cases where actual data might be longer/shorter if prediction stopped early
             df_data_aligned[key] = np.array(value)[:min_len]

    # Ensure all arrays have the same final length 'min_len'
    final_min_len = min(len(v) for v in df_data_aligned.values())
    for key in df_data_aligned:
        df_data_aligned[key] = df_data_aligned[key][:final_min_len]


    predictions_df = pd.DataFrame(df_data_aligned)
    predictions_df.to_csv(output_file_path, index=False)
    print(f"Delta predictions saved to {output_file_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Predict trajectory (deltas for vx,vy,yaw) using a trained FNN model.')
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise', help='Directory containing the runXX folders.')
    parser.add_argument('--run_id', type=str, required=True, help='ID of the run to predict (e.g., run4).')
    parser.add_argument('--model_path', type=str, default='./trained_models/state_delta_predictor_fnn.pth', help='Path to the trained delta predictor model (.pth file).')
    parser.add_argument('--scaler_params_path', type=str, required=True, help='Path to the JSON file containing scaler parameters.')
    parser.add_argument('--output_dir', type=str, default='./predictions', help='Directory to save prediction results.')

    args = parser.parse_args()

    predict_trajectory(
        data_dir=args.data_dir,
        run_id=args.run_id,
        model_path=args.model_path,
        scaler_params_path=args.scaler_params_path,
        output_dir=args.output_dir
    ) 