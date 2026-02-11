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

        self.input_features = ['accel_x', 'accel_y', 'ang_vel_z', 'vel_x', 'vel_y', 'yaw', 'rudder_angle']
        self.output_features = ['vel_x', 'vel_y'] # These are target velocities at t+1
        
        # For later use, actual positions and yaw for evaluation and coordinate transformation
        self.ground_truth_eval_features = ['pos_x', 'pos_y', 'yaw']


        all_data_x = []
        all_data_y = []
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
                
                # Verify dt assumption if needed by checking df['timestamp'] differences
                # For now, we trust dt=0.1s

                # Ensure data is sorted by time, just in case
                df = df.sort_values(by='timestamp')

                # Features at time t
                x_data_run = df[self.input_features].values
                
                # Target velocities at time t+1
                # We shift vel_x and vel_y by -1 to get the t+1 values for current t
                y_data_run = df[self.output_features].shift(-1).values
                
                # We lose the last sample for y because there's no t+2 for it
                # And we lose the first sample for x if sequence_length > 1 (not for FNN)
                # For FNN (seq_len=1), input at t, output at t+1.
                # So, x_data_run should be [0 to N-2], y_data_run should be [1 to N-1] (after shift)
                
                num_samples_in_run = len(df) - self.sequence_length # -1 for y target, and seq_len-1 for x inputs
                                                                # For FNN (seq_len=1), this is len(df)-1
                
                if num_samples_in_run <= 0:
                    print(f"Skipping {csv_file} as it has insufficient data for sequence length {self.sequence_length}")
                    continue

                all_data_x.append(x_data_run[:-self.sequence_length]) # Inputs from t=0 to t=N-2
                all_data_y.append(y_data_run[:-self.sequence_length]) # Outputs from t=1 to t=N-1
                
                self.run_indices.append({
                    'run_name': os.path.basename(run_folder),
                    'start_idx': current_idx_offset,
                    'end_idx': current_idx_offset + num_samples_in_run,
                    'original_data': df # Store original df for this run for prediction/evaluation phase
                })
                current_idx_offset += num_samples_in_run
            else:
                print(f"Warning: {csv_file} not found in {run_folder}")

        if not all_data_x:
            raise ValueError("No data loaded. Check data_dir and specific_run parameter.")

        self.x_data = np.concatenate(all_data_x, axis=0)
        self.y_data = np.concatenate(all_data_y, axis=0)
        
        # Remove any rows with NaN resulting from the shift, especially the last row of y_data for each run
        # After concatenation, the NaNs will be at the end of each run's segment in y_data
        # This is handled by slicing with :-self.sequence_length before append if all files processed together,
        # but if runs have different lengths, better to clean after concatenation.
        # The current approach of slicing[:-self.sequence_length] on x_data_run and y_data_run handles this correctly.

        self.x_data = torch.tensor(self.x_data, dtype=torch.float32)
        self.y_data = torch.tensor(self.y_data, dtype=torch.float32)
        
        print(f"Loaded data from {len(run_folders)} runs. Total samples: {len(self.x_data)}")
        print(f"Input shape: {self.x_data.shape}, Target shape: {self.y_data.shape}")


    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, idx):
        # For FNN (sequence_length=1), input is features at time t
        # target is vel_x, vel_y at time t+1
        return self.x_data[idx], self.y_data[idx]

    def get_input_feature_names(self):
        return self.input_features
        
    def get_run_data(self, run_name):
        """Returns the original DataFrame for a specific run."""
        for run_info in self.run_indices:
            if run_info['run_name'] == run_name:
                return run_info['original_data'].copy()
        return None
        
    def get_run_data_tensors_for_prediction(self, run_name):
        """
        Provides all necessary data for a single run for autoregressive prediction.
        Returns:
            initial_input (torch.Tensor): The very first input vector for the run.
            full_run_inputs (torch.Tensor): All 'other' inputs (ax, ay, wz, yaw, rudder) for the whole run.
                                            Shape: (num_timesteps, num_other_features)
            actual_vx_vy_body (torch.Tensor): Actual body velocities for the run (for reference/teacher forcing if needed).
            actual_pos_xy_yaw (torch.Tensor): Actual global positions and yaw for the run.
        """
        run_df = self.get_run_data(run_name)
        if run_df is None:
            raise ValueError(f"Run {run_name} not found in dataset.")

        # Initial input: ax, ay, wz, vx, vy, yaw, rudder at t=0
        initial_input_np = run_df[self.input_features].iloc[0].values
        initial_input = torch.tensor(initial_input_np, dtype=torch.float32)

        # 'Other' inputs for the entire run (ax, ay, wz, yaw, rudder_angle)
        # These are known throughout the prediction process from the dataset.
        # We exclude vx, vy as these will be predicted.
        other_input_features = ['accel_x', 'accel_y', 'ang_vel_z', 'yaw', 'rudder_angle']
        full_run_other_inputs_np = run_df[other_input_features].values
        full_run_other_inputs = torch.tensor(full_run_other_inputs_np, dtype=torch.float32)
        
        # Actual vx, vy (body frame) for the whole run
        actual_vx_vy_body_np = run_df[self.output_features].values
        actual_vx_vy_body = torch.tensor(actual_vx_vy_body_np, dtype=torch.float32)
        
        # Actual pos_x, pos_y, yaw (global frame) for the whole run
        actual_pos_xy_yaw_np = run_df[self.ground_truth_eval_features].values # pos_x, pos_y, yaw
        actual_pos_xy_yaw = torch.tensor(actual_pos_xy_yaw_np, dtype=torch.float32)

        return initial_input, full_run_other_inputs, actual_vx_vy_body, actual_pos_xy_yaw


if __name__ == '__main__':
    # Example usage:
    data_root_dir = '../../extracted_data_noNoise'  # Adjust path as needed
    
    # Test loading all data
    print("Testing loading all runs:")
    try:
        dataset_all = VesselMotionDataset(data_dir=data_root_dir)
        print(f"Total samples (all runs): {len(dataset_all)}")
        if len(dataset_all) > 0:
            x_sample, y_sample = dataset_all[0]
            print(f"Sample X: {x_sample.shape}, Sample Y: {y_sample.shape}")
            print(f"Input features: {dataset_all.get_input_feature_names()}")
        
        # Test DataLoader
        dataloader_all = DataLoader(dataset_all, batch_size=32, shuffle=True)
        x_batch, y_batch = next(iter(dataloader_all))
        print(f"Batch X shape: {x_batch.shape}, Batch Y shape: {y_batch.shape}")

    except Exception as e:
        print(f"Error loading all runs: {e}")

    print("\nTesting loading a specific run (e.g., run4):")
    try:
        dataset_specific = VesselMotionDataset(data_dir=data_root_dir, specific_run='run4')
        print(f"Total samples (run4): {len(dataset_specific)}")
        if len(dataset_specific) > 0:
            x_sample_spec, y_sample_spec = dataset_specific[0]
            print(f"Sample X (run4): {x_sample_spec.shape}, Sample Y (run4): {y_sample_spec.shape}")
            
            initial_input, full_run_other_inputs, actual_vx_vy, actual_pos_yaw = dataset_specific.get_run_data_tensors_for_prediction('run4')
            print(f"Initial input for run4: {initial_input.shape}")
            print(f"Full run 'other' inputs for run4: {full_run_other_inputs.shape}")
            print(f"Full run actual_vx_vy_body for run4: {actual_vx_vy.shape}")
            print(f"Full run actual_pos_xy_yaw for run4: {actual_pos_yaw.shape}")
            
            # Example of accessing original dataframe
            # df_run4 = dataset_specific.get_run_data('run4')
            # print(f"Original run4 DataFrame shape: {df_run4.shape}")


    except FileNotFoundError as e:
        print(f"Error loading specific run: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}") 