import torch
import numpy as np
import pandas as pd
import os
import argparse

from model import VelocityPredictorFNN # Assuming model.py is in the same directory
from data_loader import VesselMotionDataset # Assuming data_loader.py is in the same directory
from utils import body_to_global_velocity, integrate_position # Assuming utils.py is in the same directory

def predict_trajectory(data_dir, run_id, model_path, output_dir='./predictions'):
    """
    Performs auto-regressive prediction for a specific run using a trained model.

    Args:
        data_dir (str): Directory containing the runXX folders.
        run_id (str): The ID of the run to predict (e.g., 'run4').
        model_path (str): Path to the trained model (.pth file).
        output_dir (str): Directory to save the prediction results.
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 1. Load Dataset to get run-specific data
    # We initialize it to get access to helper methods, not for iteration here.
    try:
        dataset = VesselMotionDataset(data_dir=data_dir, specific_run=run_id)
        if len(dataset.run_indices) == 0 or dataset.run_indices[0]['run_name'] != run_id:
             raise ValueError(f"Run {run_id} could not be loaded by VesselMotionDataset.")
    except Exception as e:
        print(f"Error loading dataset for run {run_id}: {e}")
        return

    initial_input_tensor, full_run_other_inputs, actual_vx_vy_body, actual_pos_xy_yaw = dataset.get_run_data_tensors_for_prediction(run_id)
    num_timesteps = full_run_other_inputs.shape[0]
    dt = dataset.dt # Should be 0.1s

    # 2. Load Trained Model
    num_input_features = len(dataset.get_input_feature_names())
    model = VelocityPredictorFNN(input_size=num_input_features).to(device)
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
    except FileNotFoundError:
        print(f"Error: Model file not found at {model_path}")
        return
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    model.eval() # Set model to evaluation mode

    print(f"Performing prediction for {run_id} with {num_timesteps} timesteps.")

    # Lists to store predictions
    predicted_body_vx = [initial_input_tensor[3].item()] # vx_0 (from actual data)
    predicted_body_vy = [initial_input_tensor[4].item()] # vy_0 (from actual data)
    
    # Initial position from actual data (pos_x_0, pos_y_0)
    # actual_pos_xy_yaw contains [pos_x, pos_y, yaw]
    current_x_global_pred = actual_pos_xy_yaw[0, 0].item()
    current_y_global_pred = actual_pos_xy_yaw[0, 1].item()
    
    predicted_global_x = [current_x_global_pred]
    predicted_global_y = [current_y_global_pred]

    # Current model input starts with the first true full state
    current_model_input = initial_input_tensor.unsqueeze(0).to(device) 
    # initial_input_tensor has [ax_0, ay_0, wz_0, vx_0, vy_0, yaw_0, rudder_0]

    # 3. Auto-regressive Loop
    with torch.no_grad():
        for t in range(num_timesteps - 1): # Predict from t=0 up to t=N-2 to get velocities for t=1 to t=N-1
            # Predict vx_body_t+1, vy_body_t+1
            pred_vx_vy_body_next = model(current_model_input) # Output is [vx_pred_t+1, vy_pred_t+1]
            
            vx_body_pred_next = pred_vx_vy_body_next[0, 0].item()
            vy_body_pred_next = pred_vx_vy_body_next[0, 1].item()

            predicted_body_vx.append(vx_body_pred_next)
            predicted_body_vy.append(vy_body_pred_next)

            # Get yaw_t+1 from actual data for coordinate transformation
            # full_run_other_inputs has ['accel_x', 'accel_y', 'ang_vel_z', 'yaw', 'rudder_angle']
            # So, yaw is at index 3 of the feature dimension of full_run_other_inputs
            yaw_next_actual = full_run_other_inputs[t + 1, 3].item() 

            # Convert predicted body velocities at t+1 to global frame using yaw_t+1
            vx_global_pred_next, vy_global_pred_next = body_to_global_velocity(
                vx_body_pred_next, vy_body_pred_next, yaw_next_actual
            )

            # Integrate to get global position x_t+1, y_t+1
            current_x_global_pred, current_y_global_pred = integrate_position(
                current_x_global_pred, current_y_global_pred, 
                vx_global_pred_next, vy_global_pred_next, dt
            )
            predicted_global_x.append(current_x_global_pred)
            predicted_global_y.append(current_y_global_pred)

            # Prepare input for the next step: current_model_input for t+1
            # ax_t+1, ay_t+1, wz_t+1, yaw_t+1, rudder_t+1 are from actual data (full_run_other_inputs)
            # vx_t+1, vy_t+1 are the predicted ones
            if t + 1 < num_timesteps: # Ensure we don't go out of bounds for full_run_other_inputs
                ax_next = full_run_other_inputs[t + 1, 0]
                ay_next = full_run_other_inputs[t + 1, 1]
                wz_next = full_run_other_inputs[t + 1, 2]
                # yaw_next_actual is already fetched
                rudder_next = full_run_other_inputs[t + 1, 4]

                next_input_features = torch.tensor([
                    ax_next.item(), ay_next.item(), wz_next.item(),
                    vx_body_pred_next, vy_body_pred_next, # Predicted velocities
                    yaw_next_actual, 
                    rudder_next.item()
                ], dtype=torch.float32).to(device)
                current_model_input = next_input_features.unsqueeze(0)
            else:
                break # Should not happen if loop is range(num_timesteps - 1)

    # 4. Save Predictions
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    output_file_path = os.path.join(output_dir, f'{run_id}_predictions.csv')
    
    # Make sure all lists are of the same length for DataFrame creation
    # The loop runs num_timesteps - 1 times, predicting t+1 values.
    # predicted_body_vx/vy will have N items (initial + N-1 predictions)
    # predicted_global_x/y will have N items (initial + N-1 integrations)
    # actual_pos_xy_yaw has N rows. We only need N items from it.
    
    df_data = {
        'time_step': np.arange(num_timesteps) * dt,
        'actual_pos_x': actual_pos_xy_yaw[:num_timesteps, 0].cpu().numpy(),
        'actual_pos_y': actual_pos_xy_yaw[:num_timesteps, 1].cpu().numpy(),
        'actual_yaw': actual_pos_xy_yaw[:num_timesteps, 2].cpu().numpy(),
        'actual_body_vx': actual_vx_vy_body[:num_timesteps, 0].cpu().numpy(),
        'actual_body_vy': actual_vx_vy_body[:num_timesteps, 1].cpu().numpy(),
        'predicted_body_vx': np.array(predicted_body_vx),
        'predicted_body_vy': np.array(predicted_body_vy),
        'predicted_global_x': np.array(predicted_global_x),
        'predicted_global_y': np.array(predicted_global_y)
    }
    # Check lengths to ensure they match before creating DataFrame
    min_len = min(len(v) for v in df_data.values())
    for key in df_data:
        df_data[key] = df_data[key][:min_len]

    predictions_df = pd.DataFrame(df_data)
    predictions_df.to_csv(output_file_path, index=False)
    print(f"Predictions saved to {output_file_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Predict trajectory using a trained FNN model.')
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise', help='Directory containing the runXX folders.')
    parser.add_argument('--run_id', type=str, required=True, help='ID of the run to predict (e.g., run4).')
    parser.add_argument('--model_path', type=str, default='./trained_models/velocity_predictor_fnn.pth', help='Path to the trained model (.pth file).')
    parser.add_argument('--output_dir', type=str, default='./predictions', help='Directory to save prediction results.')

    args = parser.parse_args()

    predict_trajectory(
        data_dir=args.data_dir,
        run_id=args.run_id,
        model_path=args.model_path,
        output_dir=args.output_dir
    ) 