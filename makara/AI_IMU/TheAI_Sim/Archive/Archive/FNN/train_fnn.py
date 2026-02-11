import os
import argparse
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Input, Dropout, BatchNormalization
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
import joblib
import glob

# Define features and target
INPUT_FEATURES = ['pos_x', 'pos_y', 'vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
OUTPUT_FEATURES = ['next_pos_x', 'next_pos_y', 'next_vel_x', 'next_vel_y']
OUTPUT_FEATURES_COUNT = len(OUTPUT_FEATURES)

def load_and_preprocess_data(base_data_directory):
    dt = 0.1 # Constant dt
    if not os.path.isdir(base_data_directory):
        raise FileNotFoundError(f"Base data directory not found: {base_data_directory}")

    search_pattern = os.path.join(base_data_directory, '**', 'synchronized.csv')
    csv_files = glob.glob(search_pattern, recursive=True)

    if not csv_files:
        raise FileNotFoundError(f"No 'synchronized.csv' files found in {base_data_directory} or its subdirectories.")

    print(f"Found {len(csv_files)} synchronized.csv files to process for FNN Position & Velocity Predictor.")

    all_X_data_list = []
    all_y_data_list = []

    for csv_file_path in csv_files:
        df = pd.read_csv(csv_file_path)

        if len(df) < 3: # Need at least 3 points for vel_i and target next_vel_{i+1}
            print(f"Warning: Not enough data in {csv_file_path} (requires >= 3 rows). Skipping.")
            continue
            
        # Calculate current velocities (vel_i) - used for input X
        df['vel_x'] = (df['pos_x'].shift(-1) - df['pos_x']) / dt
        df['vel_y'] = (df['pos_y'].shift(-1) - df['pos_y']) / dt
        
        # Define target next positions (pos_{i+1})
        df['next_pos_x'] = df['pos_x'].shift(-1)
        df['next_pos_y'] = df['pos_y'].shift(-1)
        
        # Define target next velocities (vel_{i+1})
        # These are calculated from pos_{i+2} and pos_{i+1}
        df['next_vel_x'] = (df['pos_x'].shift(-2) - df['pos_x'].shift(-1)) / dt
        df['next_vel_y'] = (df['pos_y'].shift(-2) - df['pos_y'].shift(-1)) / dt

        # Prepare data frame with all necessary columns for inputs and outputs
        # Note: pos_x and pos_y are original, not shifted, for input X at step i
        # accel_x, accel_y, ang_vel_z, rudder_angle are also original for step i
        
        temp_df = df[['pos_x', 'pos_y', 'vel_x', 'vel_y', 
                      'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle', # Input features
                      'next_pos_x', 'next_pos_y', 'next_vel_x', 'next_vel_y' # Output features
                     ]].copy()

        # Replace potential infinities from division by zero dt (unlikely with constant dt but good practice)
        temp_df.replace([np.inf, -np.inf], np.nan, inplace=True)
        
        # Drop rows with any NaN values (results from shifting)
        # This aligns inputs (pos_i, vel_i, accel_i) with targets (pos_{i+1}, vel_{i+1})
        temp_df.dropna(inplace=True)

        if temp_df.empty:
            print(f"Warning: Not enough valid data after processing {csv_file_path} for Position-Velocity FNN. Skipping.")
            continue
            
        all_X_data_list.append(temp_df[INPUT_FEATURES])
        all_y_data_list.append(temp_df[OUTPUT_FEATURES])

    if not all_X_data_list:
        raise ValueError("No data loaded for Position-Velocity FNN.")

    X_combined = pd.concat(all_X_data_list, ignore_index=True).values
    y_combined = pd.concat(all_y_data_list, ignore_index=True).values

    X_train_val, X_test_orig, y_train_val, y_test_orig = train_test_split(
        X_combined, y_combined, test_size=0.15, random_state=42, shuffle=False
    )
    X_train_orig, X_val_orig, y_train_orig, y_val_orig = train_test_split(
        X_train_val, y_train_val, test_size=0.2, random_state=42, shuffle=True
    )

    scaler_X = StandardScaler()
    X_train_scaled = scaler_X.fit_transform(X_train_orig)
    X_val_scaled = scaler_X.transform(X_val_orig)
    X_test_scaled = scaler_X.transform(X_test_orig)

    scaler_Y = StandardScaler() # Single scaler for all 4 output features
    y_train_scaled = scaler_Y.fit_transform(y_train_orig)
    y_val_scaled = scaler_Y.transform(y_val_orig)
    y_test_scaled = scaler_Y.transform(y_test_orig)
    
    return (
        X_train_scaled, X_val_scaled, X_test_scaled,
        y_train_scaled, y_val_scaled, y_test_scaled,
        scaler_X, scaler_Y
    )

def build_fnn_model(input_dim, output_dim): # Changed from build_single_fnn_model
    model = Sequential([
        Input(shape=(input_dim,), name='input_state'),
        Dense(128, activation='relu', name='dense1'),
        BatchNormalization(name='bn1'),
        Dropout(0.2, name='dropout1'),
        Dense(128, activation='relu', name='dense2'),
        BatchNormalization(name='bn2'),
        Dropout(0.2, name='dropout2'),
        Dense(64, activation='relu', name='dense3'),
        BatchNormalization(name='bn3'),
        Dense(32, activation='relu', name='dense4'),
        BatchNormalization(name='bn4'),
        Dense(output_dim, name='output_next_state') # Predicts 4 values: next_pos_x, next_pos_y, next_vel_x, next_vel_y
    ], name='model_pos_vel_predictor')
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.0005), loss='mse', metrics=['mae'])
    return model

def main():
    parser = argparse.ArgumentParser(description="Train a single FNN model for next_position and next_velocity prediction.")
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise',
                        help='Path to the base directory containing run data.')
    parser.add_argument('--epochs', type=int, default=100, help='Number of training epochs.')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size for training.')
    parser.add_argument('--model_save_dir', type=str, default='./FNN/model_pos_vel_predictor', # New save directory
                        help='Directory to save trained FNN pos-vel model and scalers.')
    
    args = parser.parse_args()
    os.makedirs(args.model_save_dir, exist_ok=True)

    print("Loading and preprocessing data for FNN Position-Velocity Predictor...")
    (
        X_train_scaled, X_val_scaled, X_test_scaled,
        y_train_scaled, y_val_scaled, y_test_scaled,
        scaler_X, scaler_Y # Updated scaler names
    ) = load_and_preprocess_data(args.data_dir)
    
    if X_train_scaled.size == 0:
        print("No training data available after processing. Exiting.")
        return

    input_dim = X_train_scaled.shape[1]
    output_dim = y_train_scaled.shape[1] # Should be 4

    print("\nBuilding and training FNN for next_position and next_velocity...")
    model = build_fnn_model(input_dim, output_dim) # Single model
    model.summary()
    
    callbacks = [
        EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True),
        ModelCheckpoint(os.path.join(args.model_save_dir, 'fnn_pos_vel_best_model.keras'), # New model name
                        save_best_only=True, monitor='val_loss')
    ]
    
    model.fit(X_train_scaled, y_train_scaled, 
              epochs=args.epochs, batch_size=args.batch_size, 
              validation_data=(X_val_scaled, y_val_scaled), 
              callbacks=callbacks, verbose=1)
              
    joblib.dump(scaler_X, os.path.join(args.model_save_dir, 'fnn_scaler_X_pos_vel.joblib')) # New scaler name
    joblib.dump(scaler_Y, os.path.join(args.model_save_dir, 'fnn_scaler_Y_pos_vel.joblib')) # New scaler name

    if X_test_scaled.size > 0 and y_test_scaled.size > 0:
        test_loss, test_mae = model.evaluate(X_test_scaled, y_test_scaled, verbose=0)
        print(f"FNN Position-Velocity Test Loss: {test_loss:.4f}, MAE: {test_mae:.4f}")

    print(f"\nFNN Position-Velocity predictor model and scalers saved to {args.model_save_dir}")

if __name__ == '__main__':
    main() 