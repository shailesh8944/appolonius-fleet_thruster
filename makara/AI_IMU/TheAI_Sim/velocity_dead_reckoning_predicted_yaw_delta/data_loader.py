import os
import glob
import pandas as pd
import torch
from torch.utils.data import Dataset, DataLoader
import numpy as np
import json # For saving/loading scaler params

class VesselMotionDataset(Dataset):
    def __init__(self, data_dir, sequence_length=1, specific_run=None, scaler_params=None):
        """
        Args:
            data_dir (str): Directory containing the runXX folders.
            sequence_length (int): The number of time steps for the input sequence.
                                   For a simple FNN predicting t+1 from t, this is 1.
            specific_run (str, optional): If specified, only loads data from this run (e.g., 'run4').
            scaler_params (dict, optional): If provided, use these for standardization.
        """
        self.data_dir = data_dir
        self.sequence_length = sequence_length
        self.dt = 0.1  # s, as specified

        # yaw is replaced by sin(yaw) and cos(yaw)
        self.input_features_original = ['accel_x', 'accel_y', 'ang_vel_z', 'vel_x', 'vel_y', 'yaw', 'rudder_angle']
        self.input_features_processed = ['accel_x', 'accel_y', 'ang_vel_z', 'vel_x', 'vel_y', 'sin_yaw', 'cos_yaw', 'rudder_angle']
        
        # Targets are now deltas
        self.output_features_delta = ['delta_vel_x', 'delta_vel_y', 'delta_yaw'] 
        # Original features from which deltas are calculated
        self.output_features_original = ['vel_x', 'vel_y', 'yaw']
        
        self.ground_truth_eval_features = ['pos_x', 'pos_y', 'yaw', 'vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']

        all_data_x_processed = []
        all_data_y_delta = []
        self.run_indices = [] # To track start and end of each run's data

        if specific_run:
            run_folders = [os.path.join(data_dir, specific_run)]
            if not os.path.isdir(run_folders[0]):
                raise FileNotFoundError(f"Specific run folder {run_folders[0]} not found.")
        else:
            run_folders = sorted(glob.glob(os.path.join(data_dir, 'run*')))

        current_idx_offset = 0
        for run_folder in run_folders:
            csv_file = os.path.join(run_folder, 'synchronized.csv')
            if os.path.exists(csv_file):
                df = pd.read_csv(csv_file)
                df = df.sort_values(by='timestamp').copy() # Use .copy() to avoid SettingWithCopyWarning

                # Process input features: replace yaw with sin(yaw) and cos(yaw)
                df['sin_yaw'] = np.sin(df['yaw'])
                df['cos_yaw'] = np.cos(df['yaw'])
                x_data_run_processed = df[self.input_features_processed].values
                
                # Calculate target deltas: (value_t+1 - value_t)
                y_data_run_delta = df[self.output_features_original].diff().shift(-1).values
                # diff() computes V_t - V_{t-1}. We want V_{t+1} - V_t.
                # So, take diff of original columns, then shift up to align delta_{t+1} with x_t
                # Example: df[vx,vy,yaw].diff() -> [NaN, d1, d2, ..., dN-1]
                # .shift(-1) -> [d1, d2, ..., dN-1, NaN]
                # This y_data_run_delta[i] corresponds to (state[i+1] - state[i])
                                
                num_samples_in_run = len(df) - self.sequence_length 
                # Due to diff() and shift(-1), the last row of y_data_run_delta will be NaN.
                # And the first row of diff() is NaN. This is handled by slicing.
                # We need self.sequence_length more rows than the resulting samples for x.
                # x_data: 0 to N-2 (N-1 samples if seq_len=1)
                # y_data: (state_1-state_0) .... (state_N-1 - state_N-2) (N-1 samples)

                if num_samples_in_run <= 0:
                    print(f"Skipping {csv_file} due to insufficient data for sequence_length {self.sequence_length} after processing.")
                    continue
                
                # x goes from index 0 to N-1-seq_len
                # y goes from index 0 to N-1-seq_len (corresponding to delta between t+1 and t for input t)
                all_data_x_processed.append(x_data_run_processed[:-self.sequence_length])
                all_data_y_delta.append(y_data_run_delta[:-self.sequence_length])
                
                self.run_indices.append({
                    'run_name': os.path.basename(run_folder),
                    'start_idx': current_idx_offset,
                    'end_idx': current_idx_offset + num_samples_in_run,
                    'original_data': df # Store df *after* adding sin_yaw, cos_yaw
                })
                current_idx_offset += num_samples_in_run
            else:
                print(f"Warning: {csv_file} not found in {run_folder}")

        if not all_data_x_processed:
            raise ValueError("No data loaded. Check data_dir and specific_run parameter.")

        self.x_data_raw = np.concatenate(all_data_x_processed, axis=0)
        self.y_data_raw = np.concatenate(all_data_y_delta, axis=0)
        
        # Remove rows where y_data might be NaN (especially the last one from each original df segment if not perfectly handled by slicing)
        # This should be handled by `[:-self.sequence_length]` if sequence_length correctly accounts for all shifts.
        # For FNN (seq_len=1), y_data_run_delta is N-1 non-NaN values if original df has N rows.
        # x_data_run_processed is N non-NaN values.
        # Slicing with [:-1] makes them both N-1 samples.
        valid_rows = ~np.isnan(self.y_data_raw).any(axis=1)
        self.x_data_raw = self.x_data_raw[valid_rows]
        self.y_data_raw = self.y_data_raw[valid_rows]

        # Standardization (Scaling)
        if scaler_params:
            self.input_means = np.array(scaler_params['input_means'])
            self.input_stds = np.array(scaler_params['input_stds'])
            self.target_means = np.array(scaler_params['target_means'])
            self.target_stds = np.array(scaler_params['target_stds'])
            print("Using provided scaler parameters.")
        else:
            # Calculate and store scaler parameters from the current data (training)
            self.input_means = np.mean(self.x_data_raw, axis=0)
            self.input_stds = np.std(self.x_data_raw, axis=0)
            self.target_means = np.mean(self.y_data_raw, axis=0)
            self.target_stds = np.std(self.y_data_raw, axis=0)
            # Avoid division by zero if a feature has zero std (is constant)
            self.input_stds[self.input_stds == 0] = 1.0
            self.target_stds[self.target_stds == 0] = 1.0
            print("Calculated and using new scaler parameters.")

        self.x_data = (self.x_data_raw - self.input_means) / self.input_stds
        self.y_data = (self.y_data_raw - self.target_means) / self.target_stds

        self.x_data = torch.tensor(self.x_data, dtype=torch.float32)
        self.y_data = torch.tensor(self.y_data, dtype=torch.float32)
        
        print(f"Loaded data from {len(run_folders)} runs. Total valid samples: {len(self.x_data)}")
        print(f"Input shape (scaled): {self.x_data.shape}, Target shape (scaled deltas): {self.y_data.shape}")

    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, idx):
        return self.x_data[idx], self.y_data[idx]

    def get_input_feature_names(self):
        return self.input_features_processed # Return names of features after processing
        
    def get_scaler_params(self):
        """Returns the calculated scaler parameters for inputs and targets."""
        return {
            'input_means': self.input_means.tolist(),
            'input_stds': self.input_stds.tolist(),
            'target_means': self.target_means.tolist(),
            'target_stds': self.target_stds.tolist()
        }

    def get_run_data(self, run_name):
        """Returns the original DataFrame for a specific run."""
        for run_info in self.run_indices:
            if run_info['run_name'] == run_name:
                # original_data now includes sin_yaw, cos_yaw
                return run_info['original_data'].copy() 
        return None
        
    def get_run_data_tensors_for_prediction(self, run_name):
        """
        Provides all necessary data for a single run for autoregressive prediction.
        Returns:
            initial_model_input (torch.Tensor): The very first input vector for the run.
            full_run_exogenous_inputs (torch.Tensor): All 'exogenous' inputs (ax, ay, wz, rudder_angle) for the whole run.
            initial_state_tensor (torch.Tensor): Initial state (vx_0, vy_0, yaw_0, pos_x_0, pos_y_0) for the run.
            actual_original_outputs_and_pos (torch.Tensor): Actual original outputs (vx, vy, yaw) and positions (pos_x, pos_y) for the whole run.
        """
        run_df_processed = self.get_run_data(run_name) # This df has sin_yaw, cos_yaw
        if run_df_processed is None:
            raise ValueError(f"Run {run_name} not found in dataset.")

        # Initial model input (unscaled, then will be scaled)
        initial_model_input_np_raw = run_df_processed[self.input_features_processed].iloc[0].values
        initial_model_input_scaled = (initial_model_input_np_raw - self.input_means) / self.input_stds
        initial_model_input_tensor = torch.tensor(initial_model_input_scaled, dtype=torch.float32)

        # Exogenous inputs (unscaled, as they are not directly fed to model if they are part of scaled input)
        # However, these specific ones (ax,ay,wz,rudder) WILL be part of the model input, so they need scaling eventually.
        # The current structure of `initial_model_input_tensor` already includes these scaled if they were part of `input_features_processed`
        # `full_run_exogenous_inputs` here are for reference or if model structure changes to take them separately.
        # For now, we only need them to reconstruct the full model input at each step *before* scaling that specific step's input vector.
        exogenous_input_features_raw = ['accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
        full_run_exogenous_inputs_np_raw = run_df_processed[exogenous_input_features_raw].values
        full_run_exogenous_inputs_tensor_raw = torch.tensor(full_run_exogenous_inputs_np_raw, dtype=torch.float32)
        
        initial_state_features = ['vel_x', 'vel_y', 'yaw', 'pos_x', 'pos_y'] 
        initial_state_np = run_df_processed[initial_state_features].iloc[0].values
        initial_state_tensor = torch.tensor(initial_state_np, dtype=torch.float32)

        actual_original_outputs_np = run_df_processed[self.output_features_original + ['pos_x', 'pos_y']].values
        actual_original_outputs_and_pos_tensor = torch.tensor(actual_original_outputs_np, dtype=torch.float32)

        # Return raw exogenous so predict.py can assemble the full input vector at each step, then scale it.
        return initial_model_input_tensor, full_run_exogenous_inputs_tensor_raw, initial_state_tensor, actual_original_outputs_and_pos_tensor


if __name__ == '__main__':
    # Example usage:
    data_root_dir = '../../extracted_data_noNoise'  # Adjust path as needed
    
    # Test loading all data
    print("Testing loading all runs with delta targets and sin/cos yaw input:")
    try:
        dataset_all = VesselMotionDataset(data_dir=data_root_dir)
        print(f"Total samples: {len(dataset_all)}")
        if len(dataset_all) > 0:
            x_sample, y_sample = dataset_all[0]
            print(f"Sample X (processed): {x_sample.shape}, Names: {dataset_all.get_input_feature_names()}") 
            print(f"Sample Y (deltas): {y_sample.shape}, Names: {dataset_all.output_features_delta}")
        
        # Test DataLoader
        dataloader_all = DataLoader(dataset_all, batch_size=32, shuffle=True)
        x_batch, y_batch = next(iter(dataloader_all))
        print(f"Batch X shape: {x_batch.shape}, Batch Y shape (deltas): {y_batch.shape}")

    except Exception as e:
        print(f"Error loading all runs: {e}")
        import traceback
        traceback.print_exc()

    print("\nTesting loading a specific run (e.g., run4) for prediction tensors:")
    try:
        dataset_specific = VesselMotionDataset(data_dir=data_root_dir, specific_run='run4')
        if len(dataset_specific) > 0:
            initial_model_input, full_run_exogenous_inputs, initial_state, actual_orig_outputs_pos = dataset_specific.get_run_data_tensors_for_prediction('run4')
            print(f"Initial model input for run4: {initial_model_input.shape}") # Expected: torch.Size([8])
            print(f"Full run exogenous inputs for run4: {full_run_exogenous_inputs.shape}") # Expected: (num_timesteps, 4)
            print(f"Initial state (vx0,vy0,yaw0,px0,py0) for run4: {initial_state.shape}") # Expected: torch.Size([5])
            print(f"Actual original outputs (vx,vy,yaw) and pos (px,py) for run4: {actual_orig_outputs_pos.shape}") # Expected: (num_timesteps, 5)

    except Exception as e:
        print(f"Error loading specific run for prediction: {e}")
        import traceback
        traceback.print_exc()

    print("\nTesting Dataset with Standardization:")
    try:
        # Test 1: Initialize for training (calculate scalers)
        print("\n--- Initializing for Training (calculating scalers) ---")
        dataset_train = VesselMotionDataset(data_dir=data_root_dir)
        print(f"Total samples: {len(dataset_train)}")
        if len(dataset_train) > 0:
            x_sample, y_sample = dataset_train[0]
            print(f"Sample X (scaled): {x_sample}") 
            print(f"Sample Y (scaled deltas): {y_sample}")
            scaler_params_saved = dataset_train.get_scaler_params()
            print("Scaler params calculated:", scaler_params_saved)
            
            # Save scaler params to a temporary file for testing the load path
            temp_scaler_path = "./temp_scaler_params.json"
            with open(temp_scaler_path, 'w') as f:
                json.dump(scaler_params_saved, f)

        # Test 2: Initialize for prediction (load scalers)
        print("\n--- Initializing for Prediction (loading scalers for run4) ---")
        if os.path.exists(temp_scaler_path):
            with open(temp_scaler_path, 'r') as f:
                loaded_scaler_params = json.load(f)
            dataset_pred = VesselMotionDataset(data_dir=data_root_dir, specific_run='run4', scaler_params=loaded_scaler_params)
            if len(dataset_pred) > 0: # specific_run loads all data for that run for __init__, but __len__ uses scaled x_data
                x_pred_sample, y_pred_sample = dataset_pred[0]
                print(f"Sample X from run4 (scaled with loaded params): {x_pred_sample}")
                print(f"Sample Y from run4 (scaled deltas with loaded params): {y_pred_sample}")

                initial_model_input, exos, initial_state, actual_outputs = dataset_pred.get_run_data_tensors_for_prediction('run4')
                print(f"Initial model input (scaled): {initial_model_input.shape}")
                print(f"Exogenous inputs (raw): {exos.shape}")
                print(f"Initial state (raw): {initial_state.shape}")
                print(f"Actual original outputs & pos (raw): {actual_outputs.shape}")
            os.remove(temp_scaler_path)
        else:
            print("Skipping prediction test as temp_scaler_params.json not found.")

    except Exception as e:
        print(f"Error in testing: {e}")
        import traceback
        traceback.print_exc() 