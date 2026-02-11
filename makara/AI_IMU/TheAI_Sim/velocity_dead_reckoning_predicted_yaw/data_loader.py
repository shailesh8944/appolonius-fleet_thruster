import os
import glob
import pandas as pd
import torch
from torch.utils.data import Dataset, DataLoader
import numpy as np

class VesselMotionDataset(Dataset):
    def __init__(self, data_dir, sequence_length=1, specific_run=None):
        """
        Args:
            data_dir (str): Directory containing the runXX folders.
            sequence_length (int): The number of time steps for the input sequence.
                                   For a simple FNN predicting t+1 from t, this is 1.
            specific_run (str, optional): If specified, only loads data from this run (e.g., 'run4').
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

        self.x_data = np.concatenate(all_data_x_processed, axis=0)
        self.y_data = np.concatenate(all_data_y_delta, axis=0)
        
        # Remove rows where y_data might be NaN (especially the last one from each original df segment if not perfectly handled by slicing)
        # This should be handled by `[:-self.sequence_length]` if sequence_length correctly accounts for all shifts.
        # For FNN (seq_len=1), y_data_run_delta is N-1 non-NaN values if original df has N rows.
        # x_data_run_processed is N non-NaN values.
        # Slicing with [:-1] makes them both N-1 samples.
        valid_rows = ~np.isnan(self.y_data).any(axis=1)
        self.x_data = self.x_data[valid_rows]
        self.y_data = self.y_data[valid_rows]

        self.x_data = torch.tensor(self.x_data, dtype=torch.float32)
        self.y_data = torch.tensor(self.y_data, dtype=torch.float32)
        
        print(f"Loaded data from {len(run_folders)} runs. Total valid samples: {len(self.x_data)}")
        print(f"Input shape: {self.x_data.shape}, Target shape (deltas): {self.y_data.shape}")

    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, idx):
        return self.x_data[idx], self.y_data[idx]

    def get_input_feature_names(self):
        return self.input_features_processed # Return names of features after processing
        
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

        # Initial model input: ax_0, ay_0, wz_0, vx_0, vy_0, sin(yaw_0), cos(yaw_0), rudder_0
        initial_model_input_np = run_df_processed[self.input_features_processed].iloc[0].values
        initial_model_input = torch.tensor(initial_model_input_np, dtype=torch.float32)

        # Exogenous inputs for the entire run (ax, ay, wz, rudder_angle) - these don't change definition
        exogenous_input_features_raw = ['accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
        full_run_exogenous_inputs_np = run_df_processed[exogenous_input_features_raw].values
        full_run_exogenous_inputs = torch.tensor(full_run_exogenous_inputs_np, dtype=torch.float32)
        
        # Ground truth for initial state (velocities, yaw) and positions
        # Needed to start the prediction (vx_0, vy_0, yaw_0, pos_x_0, pos_y_0)
        initial_state_features = ['vel_x', 'vel_y', 'yaw', 'pos_x', 'pos_y'] 
        initial_state_np = run_df_processed[initial_state_features].iloc[0].values
        initial_state_tensor = torch.tensor(initial_state_np, dtype=torch.float32)

        # Ground truth for all original output features (vel_x, vel_y, yaw) and positions (pos_x, pos_y) for evaluation comparison
        actual_original_outputs_np = run_df_processed[self.output_features_original + ['pos_x', 'pos_y']].values
        actual_original_outputs_and_pos = torch.tensor(actual_original_outputs_np, dtype=torch.float32)

        return initial_model_input, full_run_exogenous_inputs, initial_state_tensor, actual_original_outputs_and_pos


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