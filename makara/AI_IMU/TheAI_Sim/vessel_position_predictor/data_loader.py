import os
import pandas as pd
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import glob
import pickle
from typing import Tuple, List, Dict, Optional

class DataLoader:
    def __init__(self, data_dir: str, sequence_length: int = 10):
        """
        Initialize the data loader
        
        Args:
            data_dir: Directory containing the synchronized.csv files
            sequence_length: Number of timesteps to include in each sequence for training
        """
        self.data_dir = data_dir
        self.sequence_length = sequence_length
        self.scalers = {}
        self.df_list = []
        self.available_runs = []
    
    def discover_data(self) -> List[str]:
        """
        Find all synchronized.csv files in the data directory
        
        Returns:
            List of file paths
        """
        csv_files = glob.glob(os.path.join(self.data_dir, "run*/synchronized.csv"))
        self.available_runs = sorted([os.path.basename(os.path.dirname(f)) for f in csv_files])
        return csv_files
    
    def load_data(self, limit_runs: Optional[int] = None) -> List[pd.DataFrame]:
        """
        Load all synchronized.csv files
        
        Args:
            limit_runs: Optional limit on number of runs to load
            
        Returns:
            List of DataFrames
        """
        csv_files = self.discover_data()
        
        if limit_runs is not None:
            csv_files = csv_files[:limit_runs]
        
        self.df_list = []
        for file in csv_files:
            df = pd.read_csv(file)
            df['run'] = os.path.basename(os.path.dirname(file))
            self.df_list.append(df)
            
        print(f"Loaded {len(self.df_list)} runs")
        return self.df_list
    
    def prepare_sequences(self, 
                          df_list: Optional[List[pd.DataFrame]] = None, 
                          input_cols: Optional[List[str]] = None, 
                          output_cols: Optional[List[str]] = None,
                          include_position_input: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Prepare sequences for training
        
        Args:
            df_list: List of DataFrames, uses self.df_list if None
            input_cols: List of column names to use as input features
            output_cols: List of column names to use as output features
            include_position_input: Whether to include position in input features
            
        Returns:
            X, y: Tuple of input and output sequences
        """
        if df_list is None:
            df_list = self.df_list
        
        if input_cols is None:
            # Default to using IMU and rudder data as inputs
            input_cols = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'yaw', 'ang_vel_z', 'rudder_angle']
        
        if output_cols is None:
            # Default to predicting next position
            output_cols = ['pos_x', 'pos_y']
        
        # Add position to input features if requested
        if include_position_input and 'pos_x' not in input_cols and 'pos_y' not in input_cols:
            input_cols = input_cols + ['pos_x', 'pos_y']
        
        X_sequences = []
        y_sequences = []
        
        for df in df_list:
            for col in input_cols + output_cols:
                if col not in df.columns:
                    raise ValueError(f"Column {col} not found in dataframe")
            
            # Create sequences
            for i in range(len(df) - self.sequence_length):
                X_seq = df[input_cols].iloc[i:i+self.sequence_length].values
                y_seq = df[output_cols].iloc[i+self.sequence_length].values
                
                X_sequences.append(X_seq)
                y_sequences.append(y_seq)
        
        return np.array(X_sequences), np.array(y_sequences)
    
    def scale_data(self, X: np.ndarray, y: np.ndarray, 
                   fit_scalers: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Scale data using StandardScaler
        
        Args:
            X: Input sequences
            y: Output sequences
            fit_scalers: Whether to fit scalers or use existing ones
            
        Returns:
            X_scaled, y_scaled: Scaled input and output sequences
        """
        if fit_scalers:
            # Create and fit scalers
            X_flat = X.reshape(-1, X.shape[-1])
            y_flat = y.reshape(-1, y.shape[-1])
            
            self.scalers['X'] = StandardScaler().fit(X_flat)
            self.scalers['y'] = StandardScaler().fit(y_flat)
        
        # Scale the data
        X_flat = X.reshape(-1, X.shape[-1])
        X_scaled_flat = self.scalers['X'].transform(X_flat)
        X_scaled = X_scaled_flat.reshape(X.shape)
        
        y_scaled = self.scalers['y'].transform(y)
        
        return X_scaled, y_scaled
    
    def inverse_scale_y(self, y_scaled: np.ndarray) -> np.ndarray:
        """
        Inverse scale predictions
        
        Args:
            y_scaled: Scaled output sequences
            
        Returns:
            y: Original scale output sequences
        """
        return self.scalers['y'].inverse_transform(y_scaled)
    
    def split_data(self, X: np.ndarray, y: np.ndarray, 
                   test_size: float = 0.2, 
                   val_size: float = 0.1) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Split data into train, validation, and test sets
        
        Args:
            X: Input sequences
            y: Output sequences
            test_size: Fraction of data to use for testing
            val_size: Fraction of data to use for validation
            
        Returns:
            X_train, X_val, X_test, y_train, y_val, y_test
        """
        # First split into train+val and test
        X_train_val, X_test, y_train_val, y_test = train_test_split(X, y, test_size=test_size, random_state=42)
        
        # Then split train+val into train and val
        val_ratio = val_size / (1 - test_size)
        X_train, X_val, y_train, y_val = train_test_split(X_train_val, y_train_val, test_size=val_ratio, random_state=42)
        
        return X_train, X_val, X_test, y_train, y_val, y_test
    
    def prepare_dataset(self, 
                        input_cols: Optional[List[str]] = None, 
                        output_cols: Optional[List[str]] = None,
                        limit_runs: Optional[int] = None,
                        test_size: float = 0.2,
                        val_size: float = 0.1,
                        include_position_input: bool = True) -> Dict[str, np.ndarray]:
        """
        Complete data preparation pipeline
        
        Args:
            input_cols: List of column names to use as input features
            output_cols: List of column names to use as output features
            limit_runs: Optional limit on number of runs to load
            test_size: Fraction of data to use for testing
            val_size: Fraction of data to use for validation
            include_position_input: Whether to include position in input features
            
        Returns:
            Dictionary containing all dataset splits
        """
        # Load data
        self.load_data(limit_runs=limit_runs)
        
        # Create sequences
        X, y = self.prepare_sequences(input_cols=input_cols, output_cols=output_cols, include_position_input=include_position_input)
        
        # Scale data
        X_scaled, y_scaled = self.scale_data(X, y, fit_scalers=True)
        
        # Split data
        X_train, X_val, X_test, y_train, y_val, y_test = self.split_data(X_scaled, y_scaled, test_size=test_size, val_size=val_size)
        
        # Save scalers for later use
        os.makedirs('model_output', exist_ok=True)
        with open(os.path.join('model_output', 'scalers.pkl'), 'wb') as f:
            pickle.dump(self.scalers, f)
        
        return {
            "X_train": X_train,
            "X_val": X_val,
            "X_test": X_test,
            "y_train": y_train,
            "y_val": y_val,
            "y_test": y_test,
            "input_cols": input_cols,
            "output_cols": output_cols
        }

if __name__ == "__main__":
    # Test data loading
    data_dir = "../extracted_data_noNoise"
    loader = DataLoader(data_dir, sequence_length=10)
    dataset = loader.prepare_dataset(limit_runs=5)
    
    print(f"Input shape: {dataset['X_train'].shape}")
    print(f"Output shape: {dataset['y_train'].shape}")
    print(f"Available runs: {loader.available_runs[:5]}") 