import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from typing import Dict, List, Tuple
import pickle

from data_loader import DataLoader
from model import VesselPositionPredictor
from visualization import VesselVisualizer

def train_model(args: argparse.Namespace) -> Tuple[VesselPositionPredictor, Dict, Dict]:
    """
    Train the vessel position prediction model
    
    Args:
        args: Command line arguments
        
    Returns:
        model: Trained model
        dataset: Dataset dictionary
        metrics: Evaluation metrics
    """
    print(f"Training vessel position prediction model with {args.model_type} architecture...")
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(args.output_dir, f"{args.model_type}_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    
    # Configure input and output features
    input_cols = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'yaw', 'ang_vel_z', 'rudder_angle']
    output_cols = ['pos_x', 'pos_y']
    
    # Include position in input if requested
    include_position = not args.no_position_input
    
    print(f"Using input features: {input_cols}{' + position' if include_position else ''}")
    print(f"Predicting: {output_cols}")
    
    # Load and prepare dataset
    loader = DataLoader(args.data_dir, sequence_length=args.sequence_length)
    dataset = loader.prepare_dataset(
        input_cols=input_cols,
        output_cols=output_cols,
        limit_runs=args.limit_runs,
        include_position_input=include_position
    )
    
    # Get the actual number of features after possibly adding position
    n_features = dataset['X_train'].shape[2]
    
    # Create model
    model = VesselPositionPredictor(
        model_type=args.model_type,
        sequence_length=args.sequence_length,
        n_features=n_features,
        output_dim=len(output_cols)
    )
    
    # Train model
    model_path = os.path.join(output_dir, "model")
    history = model.train(
        dataset['X_train'], dataset['y_train'],
        dataset['X_val'], dataset['y_val'],
        batch_size=args.batch_size,
        epochs=args.epochs,
        patience=args.patience,
        model_path=model_path
    )
    
    # Evaluate model
    metrics = model.evaluate(dataset['X_test'], dataset['y_test'])
    print(f"Evaluation metrics: {metrics}")
    
    # Create visualizer
    visualizer = VesselVisualizer(save_dir=output_dir, show_plots=args.show_plots)
    
    # Plot training history
    visualizer.plot_training_history(history, save_name="training_history")
    
    # Make predictions on test set
    y_pred = model.predict(dataset['X_test'])
    
    # Convert predictions back to original scale
    y_test_orig = loader.inverse_scale_y(dataset['y_test'])
    y_pred_orig = loader.inverse_scale_y(y_pred)
    
    # Calculate errors
    errors = y_test_orig - y_pred_orig
    
    # Plot error distribution
    visualizer.plot_error_distribution(errors, save_name="error_distribution")
    
    # Plot trajectory for a sample from the test set
    sample_idx = np.random.randint(0, len(dataset['X_test']))
    trajectory_length = 100
    
    # Get sample trajectory that is long enough
    while sample_idx + trajectory_length >= len(dataset['X_test']):
        sample_idx = np.random.randint(0, len(dataset['X_test']))
    
    # Extract sample trajectory
    truth_sample = y_test_orig[sample_idx:sample_idx + trajectory_length]
    pred_sample = y_pred_orig[sample_idx:sample_idx + trajectory_length]
    
    # Plot trajectory
    visualizer.plot_trajectory(truth_sample, pred_sample, 
                            title=f"{args.model_type.upper()} Model Trajectory Prediction",
                            save_name="trajectory_sample")
    
    # Plot time series
    visualizer.plot_time_series_prediction(truth_sample, pred_sample,
                                        title=f"{args.model_type.upper()} Model Position Prediction",
                                        save_name="time_series_sample")
    
    # Save scalers
    scalers_path = os.path.join(output_dir, "scalers.pkl")
    with open(scalers_path, 'wb') as f:
        pickle.dump(loader.scalers, f)
    print(f"Scalers saved to {scalers_path}")
    
    # Make autoregressive predictions
    if args.autoregressive:
        # Get a test sequence
        ar_start_idx = sample_idx
        initial_sequence = dataset['X_test'][ar_start_idx]
        
        # Determine position indices if position is in input
        position_indices = None
        if include_position:
            position_indices = (n_features - 2, n_features - 1)
        
        # Make autoregressive predictions
        ar_steps = min(50, trajectory_length)
        ar_predictions = model.predict_sequence(
            initial_sequence, 
            n_steps=ar_steps, 
            scaler_y=loader.scalers['y'],
            scaler_X=loader.scalers['X'],
            position_indices=position_indices
        )
        
        # Plot autoregressive predictions
        truth_for_ar = y_test_orig[ar_start_idx:ar_start_idx + args.sequence_length + ar_steps]
        visualizer.plot_autoregressive_prediction(
            truth_for_ar,
            ar_predictions,
            title=f"{args.model_type.upper()} Autoregressive Prediction ({ar_steps} steps)",
            save_name="autoregressive_prediction"
        )
    
    # Save the model
    model.save(model_path)
    print(f"Model saved to {model_path}")
    
    return model, dataset, metrics

def main():
    parser = argparse.ArgumentParser(description='Train vessel position prediction model')
    
    # Data parameters
    parser.add_argument('--data_dir', type=str, default='../extracted_data_noNoise',
                        help='Directory containing the synchronized.csv files')
    parser.add_argument('--limit_runs', type=int, default=None,
                        help='Limit the number of runs to load')
    
    # Model parameters
    parser.add_argument('--model_type', type=str, default='lstm',
                        choices=['lstm', 'bilstm', 'gru', 'cnn_lstm'],
                        help='Type of model to use')
    parser.add_argument('--sequence_length', type=int, default=10,
                        help='Number of timesteps to include in each sequence for training')
    parser.add_argument('--no_position_input', action='store_true',
                        help='Do not include position in the input features')
    
    # Training parameters
    parser.add_argument('--batch_size', type=int, default=32,
                        help='Batch size for training')
    parser.add_argument('--epochs', type=int, default=100,
                        help='Maximum number of epochs to train for')
    parser.add_argument('--patience', type=int, default=10,
                        help='Patience for early stopping')
    
    # Output parameters
    parser.add_argument('--output_dir', type=str, default='./results',
                        help='Directory to save results')
    parser.add_argument('--show_plots', action='store_true',
                        help='Display plots interactively (default: save only)')
    
    # Evaluation parameters
    parser.add_argument('--autoregressive', action='store_true',
                        help='Whether to perform autoregressive predictions')
    
    args = parser.parse_args()
    
    # Train the model
    train_model(args)

if __name__ == '__main__':
    main() 