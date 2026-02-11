import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from typing import Dict, List, Tuple, Any
import pandas as pd
import pickle

from data_loader import DataLoader
from model import VesselPositionPredictor
from visualization import VesselVisualizer
from train import train_model

def compare_models(args: argparse.Namespace) -> Dict[str, Any]:
    """
    Compare different model architectures
    
    Args:
        args: Command line arguments
        
    Returns:
        Dictionary with results for each model
    """
    print("Comparing different model architectures...")
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(args.output_dir, f"comparison_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    
    # Create visualizer
    visualizer = VesselVisualizer(save_dir=output_dir, show_plots=args.show_plots)
    
    # Create a dictionary to store results
    results = {}
    
    # Loop through all model types
    for model_type in args.model_types:
        # Create a new args object with the current model type
        model_args = argparse.Namespace(
            data_dir=args.data_dir,
            limit_runs=args.limit_runs,
            model_type=model_type,
            sequence_length=args.sequence_length,
            no_position_input=args.no_position_input,
            batch_size=args.batch_size,
            epochs=args.epochs,
            patience=args.patience,
            output_dir=output_dir,
            show_plots=args.show_plots,
            autoregressive=args.autoregressive
        )
        
        # Train the model
        print(f"\nTraining {model_type.upper()} model...")
        model, dataset, metrics = train_model(model_args)
        
        # Store results
        results[model_type] = {
            'model': model,
            'metrics': metrics
        }
    
    # Compare models performance
    compare_performance(results, output_dir, show_plots=args.show_plots)
    
    # Compare predictions on a common test sequence
    if len(results) > 1:
        # Load a sample dataset if not already loaded
        if 'dataset' not in locals():
            loader = DataLoader(args.data_dir, sequence_length=args.sequence_length)
            dataset = loader.prepare_dataset(
                limit_runs=args.limit_runs,
                test_size=0.2,
                val_size=0.1,
                include_position_input=not args.no_position_input
            )
        
        # Choose a random sample from the test set
        sample_idx = np.random.randint(0, len(dataset['X_test']))
        trajectory_length = 100
        
        # Get sample trajectory that is long enough
        while sample_idx + trajectory_length >= len(dataset['X_test']):
            sample_idx = np.random.randint(0, len(dataset['X_test']))
        
        # Get ground truth
        y_test_orig = loader.inverse_scale_y(dataset['y_test'])
        truth_sample = y_test_orig[sample_idx:sample_idx + trajectory_length]
        
        # Get predictions from each model
        predictions = {}
        for model_type, model_results in results.items():
            model = model_results['model']
            y_pred = model.predict(dataset['X_test'][sample_idx:sample_idx + trajectory_length])
            pred_sample = loader.inverse_scale_y(y_pred)
            predictions[model_type.upper()] = pred_sample
        
        # Plot comparison
        visualizer.plot_comparison(
            truth_sample, 
            predictions,
            title="Model Architecture Comparison",
            save_name="model_comparison"
        )
        
        # Compare autoregressive predictions
        if args.autoregressive:
            ar_steps = min(50, trajectory_length)
            ar_predictions = {}
            
            # Get initial sequence
            initial_sequence = dataset['X_test'][sample_idx]
            
            # Determine position indices if position is in input
            position_indices = None
            n_features = dataset['X_test'].shape[2]
            if not args.no_position_input:
                position_indices = (n_features - 2, n_features - 1)
            
            # Make autoregressive predictions for each model
            for model_type, model_results in results.items():
                model = model_results['model']
                ar_pred = model.predict_sequence(
                    initial_sequence, 
                    n_steps=ar_steps, 
                    scaler_y=loader.scalers['y'],
                    scaler_X=loader.scalers['X'],
                    position_indices=position_indices
                )
                ar_predictions[model_type.upper()] = ar_pred
            
            # Get ground truth for AR prediction
            truth_for_ar = y_test_orig[sample_idx:sample_idx + args.sequence_length + ar_steps]
            
            # Plot comparison
            visualizer.plot_comparison(
                truth_for_ar,
                ar_predictions,
                title=f"Autoregressive Prediction Comparison ({ar_steps} steps)",
                save_name="ar_model_comparison"
            )
    
    return results

def compare_performance(results: Dict[str, Any], output_dir: str, show_plots: bool = False) -> None:
    """
    Compare performance metrics of different models
    
    Args:
        results: Dictionary with results for each model
        output_dir: Directory to save results
        show_plots: Whether to show plots interactively
    """
    # Extract metrics
    metrics_dict = {model_type: result['metrics'] for model_type, result in results.items()}
    
    # Convert to DataFrame
    metrics_df = pd.DataFrame.from_dict(metrics_dict, orient='index')
    
    # Save to CSV
    metrics_file = os.path.join(output_dir, "metrics_comparison.csv")
    metrics_df.to_csv(metrics_file)
    print(f"Metrics comparison saved to {metrics_file}")
    
    # Print table
    print("\nPerformance Comparison:")
    print(metrics_df)
    
    # Create bar plot of MSE and MAE
    plt.figure(figsize=(12, 6))
    
    metrics_df.plot(kind='bar', figsize=(12, 6))
    plt.title('Model Performance Comparison')
    plt.ylabel('Metric Value')
    plt.xlabel('Model Type')
    plt.grid(True, axis='y')
    plt.tight_layout()
    
    # Save figure
    plt.savefig(os.path.join(output_dir, "metrics_comparison.png"), dpi=300)
    
    # Close figure if not showing plots
    if not show_plots:
        plt.close()

def main():
    parser = argparse.ArgumentParser(description='Compare vessel position prediction models')
    
    # Data parameters
    parser.add_argument('--data_dir', type=str, default='../extracted_data_noNoise',
                        help='Directory containing the synchronized.csv files')
    parser.add_argument('--limit_runs', type=int, default=5,
                        help='Limit the number of runs to load')
    
    # Model parameters
    parser.add_argument('--model_types', type=str, nargs='+', 
                        default=['lstm', 'bilstm', 'gru', 'cnn_lstm'],
                        help='Types of models to compare')
    parser.add_argument('--sequence_length', type=int, default=10,
                        help='Number of timesteps to include in each sequence for training')
    parser.add_argument('--no_position_input', action='store_true',
                        help='Do not include position in the input features')
    
    # Training parameters
    parser.add_argument('--batch_size', type=int, default=32,
                        help='Batch size for training')
    parser.add_argument('--epochs', type=int, default=50,
                        help='Maximum number of epochs to train for')
    parser.add_argument('--patience', type=int, default=5,
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
    
    # Compare models
    compare_models(args)

if __name__ == '__main__':
    main() 