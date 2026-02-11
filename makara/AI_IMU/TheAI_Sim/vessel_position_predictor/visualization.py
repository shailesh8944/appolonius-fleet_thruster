import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from typing import Dict, List, Optional, Tuple
import os

class VesselVisualizer:
    def __init__(self, save_dir: Optional[str] = None, show_plots: bool = False):
        """
        Initialize the vessel visualizer
        
        Args:
            save_dir: Directory to save visualizations, if None, don't save
            show_plots: Whether to show plots interactively (set to False for autonomous running)
        """
        self.save_dir = save_dir
        self.show_plots = show_plots
        if save_dir and not os.path.exists(save_dir):
            os.makedirs(save_dir)
    
    def plot_training_history(self, history: Dict, metrics: Optional[List[str]] = None, 
                             figsize: Tuple[int, int] = (12, 5), save_name: Optional[str] = None) -> None:
        """
        Plot training history
        
        Args:
            history: Training history from model.fit
            metrics: List of metrics to plot
            figsize: Figure size
            save_name: Name to save the figure as
        """
        if metrics is None:
            metrics = ['loss', 'mae']
        
        fig, axes = plt.subplots(1, len(metrics), figsize=figsize)
        if len(metrics) == 1:
            axes = [axes]
        
        for i, metric in enumerate(metrics):
            axes[i].plot(history[metric], label=f'Training {metric}')
            axes[i].plot(history[f'val_{metric}'], label=f'Validation {metric}')
            axes[i].set_title(f'Model {metric.upper()}')
            axes[i].set_xlabel('Epoch')
            axes[i].set_ylabel(metric.upper())
            axes[i].legend()
            axes[i].grid(True)
        
        plt.tight_layout()
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_trajectory(self, truth: np.ndarray, prediction: np.ndarray, 
                       title: str = "Vessel Trajectory", figsize: Tuple[int, int] = (10, 8),
                       save_name: Optional[str] = None) -> None:
        """
        Plot vessel trajectory
        
        Args:
            truth: Ground truth positions (N, 2)
            prediction: Predicted positions (N, 2)
            title: Plot title
            figsize: Figure size
            save_name: Name to save the figure as
        """
        fig = plt.figure(figsize=figsize)
        
        # Plot ground truth
        plt.plot(truth[:, 0], truth[:, 1], 'b-', label='Ground Truth', linewidth=2)
        
        # Plot prediction
        plt.plot(prediction[:, 0], prediction[:, 1], 'r--', label='Prediction', linewidth=2)
        
        # Add start and end markers
        plt.scatter(truth[0, 0], truth[0, 1], c='g', marker='o', s=100, label='Start')
        plt.scatter(truth[-1, 0], truth[-1, 1], c='k', marker='x', s=100, label='End')
        
        plt.title(title)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Equal aspect ratio
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_time_series_prediction(self, truth: np.ndarray, prediction: np.ndarray,
                                  timesteps: Optional[np.ndarray] = None,
                                  title: str = "Position Over Time", figsize: Tuple[int, int] = (12, 10),
                                  save_name: Optional[str] = None) -> None:
        """
        Plot time series of position predictions
        
        Args:
            truth: Ground truth positions (N, 2)
            prediction: Predicted positions (N, 2)
            timesteps: Time values (if None, use indices)
            title: Plot title
            figsize: Figure size
            save_name: Name to save the figure as
        """
        if timesteps is None:
            timesteps = np.arange(len(truth))
        
        fig, axes = plt.subplots(2, 1, figsize=figsize, sharex=True)
        
        # X position
        axes[0].plot(timesteps, truth[:, 0], 'b-', label='Ground Truth', linewidth=2)
        axes[0].plot(timesteps, prediction[:, 0], 'r--', label='Prediction', linewidth=2)
        axes[0].set_title(f"{title} - X Position")
        axes[0].set_ylabel('X Position')
        axes[0].grid(True)
        axes[0].legend()
        
        # Y position
        axes[1].plot(timesteps, truth[:, 1], 'b-', label='Ground Truth', linewidth=2)
        axes[1].plot(timesteps, prediction[:, 1], 'r--', label='Prediction', linewidth=2)
        axes[1].set_title(f"{title} - Y Position")
        axes[1].set_xlabel('Time')
        axes[1].set_ylabel('Y Position')
        axes[1].grid(True)
        axes[1].legend()
        
        plt.tight_layout()
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_error_distribution(self, errors: np.ndarray, 
                              title: str = "Prediction Error Distribution", figsize: Tuple[int, int] = (12, 5),
                              save_name: Optional[str] = None) -> None:
        """
        Plot error distribution
        
        Args:
            errors: Error values (can be 1D or 2D)
            title: Plot title
            figsize: Figure size
            save_name: Name to save the figure as
        """
        if len(errors.shape) > 1 and errors.shape[1] > 1:
            # For 2D errors (position error in x, y)
            fig, axes = plt.subplots(1, 2, figsize=figsize)
            
            axes[0].hist(errors[:, 0], bins=30, alpha=0.7)
            axes[0].set_title(f"{title} - X")
            axes[0].set_xlabel('Error')
            axes[0].set_ylabel('Frequency')
            axes[0].grid(True)
            
            axes[1].hist(errors[:, 1], bins=30, alpha=0.7)
            axes[1].set_title(f"{title} - Y")
            axes[1].set_xlabel('Error')
            axes[1].set_ylabel('Frequency')
            axes[1].grid(True)
        else:
            # For 1D errors (e.g., Euclidean distance)
            fig = plt.figure(figsize=figsize)
            plt.hist(errors, bins=30, alpha=0.7)
            plt.title(title)
            plt.xlabel('Error')
            plt.ylabel('Frequency')
            plt.grid(True)
        
        plt.tight_layout()
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_autoregressive_prediction(self, truth: np.ndarray, ar_prediction: np.ndarray,
                                     title: str = "Autoregressive Prediction", figsize: Tuple[int, int] = (10, 8),
                                     save_name: Optional[str] = None) -> None:
        """
        Plot autoregressive predictions
        
        Args:
            truth: Ground truth positions (N, 2)
            ar_prediction: Autoregressive predicted positions (M, 2)
            title: Plot title
            figsize: Figure size
            save_name: Name to save the figure as
        """
        fig = plt.figure(figsize=figsize)
        
        # Plot ground truth
        plt.plot(truth[:, 0], truth[:, 1], 'b-', label='Ground Truth', linewidth=2)
        
        # Get the start point for AR prediction (last point of truth data used for initial sequence)
        start_idx = len(truth) - len(ar_prediction)
        
        # Plot autoregressive prediction
        plt.plot(ar_prediction[:, 0], ar_prediction[:, 1], 'r--', label='AR Prediction', linewidth=2)
        
        # Connect ground truth and prediction
        plt.plot([truth[start_idx-1, 0], ar_prediction[0, 0]], 
                [truth[start_idx-1, 1], ar_prediction[0, 1]], 'g--', linewidth=1)
        
        # Add markers
        plt.scatter(truth[0, 0], truth[0, 1], c='g', marker='o', s=100, label='Start')
        plt.scatter(truth[-1, 0], truth[-1, 1], c='k', marker='x', s=100, label='Truth End')
        plt.scatter(ar_prediction[-1, 0], ar_prediction[-1, 1], c='r', marker='x', s=100, label='Prediction End')
        
        plt.title(title)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Equal aspect ratio
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_comparison(self, truth: np.ndarray, predictions: Dict[str, np.ndarray],
                      title: str = "Model Comparison", figsize: Tuple[int, int] = (12, 10),
                      save_name: Optional[str] = None) -> None:
        """
        Plot comparison of multiple models
        
        Args:
            truth: Ground truth positions (N, 2)
            predictions: Dictionary of model name to predictions
            title: Plot title
            figsize: Figure size
            save_name: Name to save the figure as
        """
        fig = plt.figure(figsize=figsize)
        
        # Plot ground truth
        plt.plot(truth[:, 0], truth[:, 1], 'k-', label='Ground Truth', linewidth=3)
        
        # Plot predictions for each model
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        for i, (name, pred) in enumerate(predictions.items()):
            color = colors[i % len(colors)]
            plt.plot(pred[:, 0], pred[:, 1], f'{color}--', label=name, linewidth=2)
        
        # Add start and end markers
        plt.scatter(truth[0, 0], truth[0, 1], c='g', marker='o', s=100, label='Start')
        plt.scatter(truth[-1, 0], truth[-1, 1], c='k', marker='x', s=100, label='End')
        
        plt.title(title)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Equal aspect ratio
        
        if self.save_dir and save_name:
            plt.savefig(os.path.join(self.save_dir, f"{save_name}.png"), dpi=300)
        
        if self.show_plots:
            plt.show()
        else:
            plt.close(fig) 