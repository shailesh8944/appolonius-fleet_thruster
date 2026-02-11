import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import pickle
from typing import Dict, List, Tuple

from data_loader import DataLoader
from model import VesselPositionPredictor
from visualization import VesselVisualizer

def load_model_and_scalers(model_path: str, scalers_path: str) -> Tuple[VesselPositionPredictor, Dict]:
    """
    Load a trained model and scalers
    
    Args:
        model_path: Path to the trained model
        scalers_path: Path to the scalers
        
    Returns:
        model: Loaded model
        scalers: Loaded scalers
    """
    # Load model
    model = VesselPositionPredictor.load(model_path)
    
    # Load scalers
    with open(scalers_path, 'rb') as f:
        scalers = pickle.load(f)
    
    return model, scalers

def make_predictions(args: argparse.Namespace) -> None:
    """
    Make predictions with a trained model
    
    Args:
        args: Command line arguments
    """
    print(f"Making predictions with model from {args.model_path}...")
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Create visualizer
    visualizer = VesselVisualizer(save_dir=args.output_dir, show_plots=args.show_plots)
    
    # Load model
    if args.scalers_path:
        model, scalers = load_model_and_scalers(args.model_path, args.scalers_path)
    else:
        # If scalers path not provided, look in the same directory as the model
        model_dir = os.path.dirname(args.model_path)
        scalers_path = os.path.join(model_dir, "scalers.pkl")
        
        if os.path.exists(scalers_path):
            model, scalers = load_model_and_scalers(args.model_path, scalers_path)
        else:
            raise ValueError(f"Scalers not found. Please provide a path to the scalers file.")
    
    # Check if model expects position in input based on feature count
    n_features = model.n_features
    include_position = n_features >= 9  # At least IMU(7) + position(2)
    position_indices = (n_features - 2, n_features - 1) if include_position else None
    
    if include_position:
        print("Model uses position as part of input features")
    else:
        print("Model does not use position as part of input features")
    
    # Load test data
    if args.test_data_path:
        # Load a specific test file
        df = pd.read_csv(args.test_data_path)
        
        # Configure input and output features
        if args.input_cols:
            input_cols = args.input_cols.split(',')
        else:
            input_cols = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'yaw', 'ang_vel_z', 'rudder_angle']
            
        if args.output_cols:
            output_cols = args.output_cols.split(',')
        else:
            output_cols = ['pos_x', 'pos_y']
        
        # Add position to input if model expects it
        if include_position and 'pos_x' not in input_cols and 'pos_y' not in input_cols:
            input_cols = input_cols + ['pos_x', 'pos_y']
            print(f"Added position to input features: {input_cols}")
        
        # Create sequences from the data
        X_sequences = []
        y_sequences = []
        
        for i in range(len(df) - model.sequence_length):
            X_seq = df[input_cols].iloc[i:i+model.sequence_length].values
            y_seq = df[output_cols].iloc[i+model.sequence_length].values
            
            X_sequences.append(X_seq)
            y_sequences.append(y_seq)
        
        X = np.array(X_sequences)
        y = np.array(y_sequences)
        
        # Scale inputs
        X_flat = X.reshape(-1, X.shape[-1])
        X_scaled_flat = scalers['X'].transform(X_flat)
        X_scaled = X_scaled_flat.reshape(X.shape)
        
        # Make predictions
        y_pred_scaled = model.predict(X_scaled)
        
        # Inverse scale predictions
        y_pred = scalers['y'].inverse_transform(y_pred_scaled)
        
        # Plot trajectory
        visualizer.plot_trajectory(
            y, y_pred,
            title="Model Trajectory Prediction",
            save_name="trajectory_prediction"
        )
        
        # Plot time series
        visualizer.plot_time_series_prediction(
            y, y_pred,
            title="Model Position Prediction",
            save_name="time_series_prediction"
        )
        
        # Calculate error
        error = y - y_pred
        
        # Plot error distribution
        visualizer.plot_error_distribution(
            error,
            title="Prediction Error Distribution",
            save_name="error_distribution"
        )
        
        # Print metrics
        mse = np.mean(np.square(error))
        mae = np.mean(np.abs(error))
        
        print(f"Test set MSE: {mse}")
        print(f"Test set MAE: {mae}")
        
        # Make autoregressive predictions
        if args.autoregressive:
            # Get a test sequence
            initial_sequence = X_scaled[0]
            
            # Make autoregressive predictions
            ar_steps = min(50, len(y))
            ar_predictions = model.predict_sequence(
                initial_sequence, 
                n_steps=ar_steps, 
                scaler_y=scalers['y'],
                scaler_X=scalers['X'],
                position_indices=position_indices
            )
            
            # Get ground truth
            truth_for_ar = y[:model.sequence_length + ar_steps]
            
            # Plot autoregressive predictions
            visualizer.plot_autoregressive_prediction(
                truth_for_ar,
                ar_predictions,
                title=f"Autoregressive Prediction ({ar_steps} steps)",
                save_name="autoregressive_prediction"
            )
            
            # Save predictions to CSV
            pred_df = pd.DataFrame(ar_predictions, columns=output_cols)
            pred_df.to_csv(os.path.join(args.output_dir, "ar_predictions.csv"), index=False)
    else:
        # Load a set of runs for evaluation
        loader = DataLoader(args.data_dir, sequence_length=model.sequence_length)
        
        # Configure input and output features
        if args.input_cols:
            input_cols = args.input_cols.split(',')
        else:
            input_cols = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'yaw', 'ang_vel_z', 'rudder_angle']
            
        if args.output_cols:
            output_cols = args.output_cols.split(',')
        else:
            output_cols = ['pos_x', 'pos_y']
        
        # Prepare dataset with position in input if needed
        dataset = loader.prepare_dataset(
            input_cols=input_cols,
            output_cols=output_cols,
            limit_runs=args.limit_runs,
            include_position_input=include_position
        )
        
        # Make predictions on test set
        y_pred_scaled = model.predict(dataset['X_test'])
        
        # Inverse scale predictions and ground truth
        y_test = loader.inverse_scale_y(dataset['y_test'])
        y_pred = loader.inverse_scale_y(y_pred_scaled)
        
        # Choose a random sample from the test set
        sample_idx = np.random.randint(0, len(dataset['X_test']))
        trajectory_length = 100
        
        # Get sample trajectory that is long enough
        while sample_idx + trajectory_length >= len(dataset['X_test']):
            sample_idx = np.random.randint(0, len(dataset['X_test']))
        
        # Extract sample trajectory
        truth_sample = y_test[sample_idx:sample_idx + trajectory_length]
        pred_sample = y_pred[sample_idx:sample_idx + trajectory_length]
        
        # Plot trajectory
        visualizer.plot_trajectory(
            truth_sample, pred_sample,
            title="Model Trajectory Prediction",
            save_name="trajectory_prediction"
        )
        
        # Plot time series
        visualizer.plot_time_series_prediction(
            truth_sample, pred_sample,
            title="Model Position Prediction",
            save_name="time_series_prediction"
        )
        
        # Calculate error for the entire test set
        error = y_test - y_pred
        
        # Plot error distribution
        visualizer.plot_error_distribution(
            error,
            title="Prediction Error Distribution",
            save_name="error_distribution"
        )
        
        # Calculate metrics
        mse = np.mean(np.square(error))
        mae = np.mean(np.abs(error))
        
        print(f"Test set MSE: {mse}")
        print(f"Test set MAE: {mae}")
        
        # Make autoregressive predictions
        if args.autoregressive:
            # Get a test sequence
            ar_start_idx = sample_idx
            initial_sequence = dataset['X_test'][ar_start_idx]
            
            # Make autoregressive predictions
            ar_steps = min(50, trajectory_length)
            ar_predictions = model.predict_sequence(
                initial_sequence, 
                n_steps=ar_steps, 
                scaler_y=loader.scalers['y'],
                scaler_X=loader.scalers['X'],
                position_indices=position_indices
            )
            
            # Get ground truth
            truth_for_ar = y_test[ar_start_idx:ar_start_idx + model.sequence_length + ar_steps]
            
            # Plot autoregressive predictions
            visualizer.plot_autoregressive_prediction(
                truth_for_ar,
                ar_predictions,
                title=f"Autoregressive Prediction ({ar_steps} steps)",
                save_name="autoregressive_prediction"
            )
            
            # Save predictions to CSV
            pred_df = pd.DataFrame(ar_predictions, columns=output_cols)
            pred_df.to_csv(os.path.join(args.output_dir, "ar_predictions.csv"), index=False)

def main():
    parser = argparse.ArgumentParser(description='Make predictions with a trained model')
    
    # Model parameters
    parser.add_argument('--model_path', type=str, required=True,
                        help='Path to the trained model')
    parser.add_argument('--scalers_path', type=str, default=None,
                        help='Path to the scalers, if not provided will look in the same directory as the model')
    
    # Data parameters
    parser.add_argument('--data_dir', type=str, default='../extracted_data_noNoise',
                        help='Directory containing the synchronized.csv files')
    parser.add_argument('--test_data_path', type=str, default=None,
                        help='Path to a specific test file to use instead of the data directory')
    parser.add_argument('--limit_runs', type=int, default=None,
                        help='Limit the number of runs to load')
    parser.add_argument('--input_cols', type=str, default=None,
                        help='Comma-separated list of input column names')
    parser.add_argument('--output_cols', type=str, default=None,
                        help='Comma-separated list of output column names')
    
    # Output parameters
    parser.add_argument('--output_dir', type=str, default='./predictions',
                        help='Directory to save predictions and visualizations')
    parser.add_argument('--show_plots', action='store_true',
                        help='Display plots interactively (default: save only)')
    
    # Evaluation parameters
    parser.add_argument('--autoregressive', action='store_true',
                        help='Whether to perform autoregressive predictions')
    
    args = parser.parse_args()
    
    # Make predictions
    make_predictions(args)

if __name__ == '__main__':
    main() 