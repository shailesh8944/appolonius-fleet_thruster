import os
import argparse
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Input, Bidirectional, Dropout
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
import joblib
import glob

# Define features for the LSTM model input
INPUT_FEATURES = ['vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
DT = 0.1 # Constant time step

def create_sequences(X_data, y_data_single_target, sequence_length):
    X_seq, y_seq = [], []
    # Ensure we predict the target at the END of the sequence
    for i in range(len(X_data) - sequence_length):
        X_seq.append(X_data[i:(i + sequence_length)])
        y_seq.append(y_data_single_target[i + sequence_length -1]) 
    return np.array(X_seq), np.array(y_seq)

def load_and_preprocess_data(base_data_directory, sequence_length):
    if not os.path.isdir(base_data_directory):
        raise FileNotFoundError(f"Base data directory not found: {base_data_directory}")

    search_pattern = os.path.join(base_data_directory, '**', 'synchronized.csv')
    csv_files = glob.glob(search_pattern, recursive=True)

    if not csv_files:
        raise FileNotFoundError(f"No 'synchronized.csv' files found in {base_data_directory} or its subdirectories.")
    
    print(f"Found {len(csv_files)} synchronized.csv files to process for Decoupled LSTM (Velocity Predictor).")

    all_X_input_features_list = [] 
    all_y_target_vx_list = []
    all_y_target_vy_list = []

    for csv_file_path in csv_files:
        print(f"Processing file for Decoupled LSTM (Velocity Predictor): {csv_file_path}")
        df = pd.read_csv(csv_file_path)
        
        if len(df) < sequence_length + 2: # Need enough points for initial vels, target vels, and sequence
            print(f"Warning: Not enough data in {csv_file_path} (requires >= {sequence_length + 2} rows). Skipping.")
            continue

        # Calculate vel_x and vel_y (current velocities for input)
        df['vel_x'] = (df['pos_x'].shift(-1) - df['pos_x']) / DT
        df['vel_y'] = (df['pos_y'].shift(-1) - df['pos_y']) / DT
        
        # Calculate target_vel_x and target_vel_y (next velocities for output)
        # target_vel_x at time t is vel_x at time t+1
        df['target_vel_x'] = df['vel_x'].shift(-1)
        df['target_vel_y'] = df['vel_y'].shift(-1)
        
        # Columns needed: vel_x, vel_y, accel_x, accel_y, ang_vel_z, rudder_angle (for X)
        # target_vel_x, target_vel_y (for Y)
        # Original accel_x etc. are used as they are concurrent with the *calculated* vel_x, vel_y.
        
        # Select all necessary columns and drop NaNs resulting from shifts
        cols_to_keep = INPUT_FEATURES + ['target_vel_x', 'target_vel_y']
        temp_df = df[cols_to_keep].copy()
        temp_df.replace([np.inf, -np.inf], np.nan, inplace=True) # Handle potential division by zero if dt was variable
        temp_df.dropna(inplace=True)

        if len(temp_df) < sequence_length + 1: # Need enough for at least one sequence after NaN drop
            print(f"Warning: Not enough valid data after processing {csv_file_path}. Skipping.")
            continue
            
        all_X_input_features_list.append(temp_df[INPUT_FEATURES])
        all_y_target_vx_list.append(temp_df['target_vel_x'])
        all_y_target_vy_list.append(temp_df['target_vel_y'])

    if not all_X_input_features_list:
        raise ValueError("No data loaded for LSTM Velocity Predictor. Check CSV paths and content.")

    X_combined_raw = pd.concat(all_X_input_features_list, ignore_index=True).values
    y_vx_combined_raw = pd.concat(all_y_target_vx_list, ignore_index=True).values.reshape(-1, 1)
    y_vy_combined_raw = pd.concat(all_y_target_vy_list, ignore_index=True).values.reshape(-1, 1)

    # Split data before scaling and sequencing
    X_train_val, X_test_orig, \
    y_vx_train_val, y_vx_test_orig, \
    y_vy_train_val, y_vy_test_orig = train_test_split(
        X_combined_raw, y_vx_combined_raw, y_vy_combined_raw,
        test_size=0.15, random_state=42, shuffle=False
    )
    
    X_train_orig, X_val_orig, \
    y_vx_train_orig, y_vx_val_orig, \
    y_vy_train_orig, y_vy_val_orig = train_test_split(
        X_train_val, y_vx_train_val, y_vy_train_val,
        test_size=0.2, random_state=42, shuffle=True # Shuffle training data
    )

    scaler_X = StandardScaler()
    X_train_scaled = scaler_X.fit_transform(X_train_orig)
    X_val_scaled = scaler_X.transform(X_val_orig)
    X_test_scaled = scaler_X.transform(X_test_orig)

    scaler_y_vel_x = StandardScaler()
    y_vx_train_scaled = scaler_y_vel_x.fit_transform(y_vx_train_orig)
    y_vx_val_scaled = scaler_y_vel_x.transform(y_vx_val_orig)
    y_vx_test_scaled = scaler_y_vel_x.transform(y_vx_test_orig)

    scaler_y_vel_y = StandardScaler()
    y_vy_train_scaled = scaler_y_vel_y.fit_transform(y_vy_train_orig)
    y_vy_val_scaled = scaler_y_vel_y.transform(y_vy_val_orig)
    y_vy_test_scaled = scaler_y_vel_y.transform(y_vy_test_orig)

    # Create sequences
    X_train_seq, y_vx_train_seq = create_sequences(X_train_scaled, y_vx_train_scaled, sequence_length)
    X_val_seq, y_vx_val_seq = create_sequences(X_val_scaled, y_vx_val_scaled, sequence_length)
    X_test_seq, y_vx_test_seq = create_sequences(X_test_scaled, y_vx_test_scaled, sequence_length)

    # Reuse X_..._seq for the vy model as inputs are the same
    _, y_vy_train_seq = create_sequences(X_train_scaled, y_vy_train_scaled, sequence_length)
    _, y_vy_val_seq = create_sequences(X_val_scaled, y_vy_val_scaled, sequence_length)
    _, y_vy_test_seq = create_sequences(X_test_scaled, y_vy_test_scaled, sequence_length)

    return (
        X_train_seq, X_val_seq, X_test_seq,
        y_vx_train_seq, y_vx_val_seq, y_vx_test_seq,
        y_vy_train_seq, y_vy_val_seq, y_vy_test_seq,
        scaler_X, scaler_y_vel_x, scaler_y_vel_y
    )

# Generic model builder for a single next_velocity component
def build_single_lstm_model(sequence_length, num_features, component_name):
    model = Sequential([
        Input(shape=(sequence_length, num_features), name=f'input_vel_hist_{component_name}'),
        Bidirectional(LSTM(128, return_sequences=True, activation='tanh', recurrent_dropout=0.2), name=f'bilstm1_vel_{component_name}'),
        Bidirectional(LSTM(128, activation='tanh', recurrent_dropout=0.2), name=f'bilstm2_vel_{component_name}'),
        Dense(64, activation='relu', name=f'dense1_vel_{component_name}'),
        Dropout(0.5, name=f'dropout_vel_{component_name}'), 
        Dense(1, name=f'output_next_vel_{component_name}') # Predicts a single next velocity value
    ], name=f'model_next_vel_{component_name}')
    optimizer = tf.keras.optimizers.Adam(learning_rate=0.0002, beta_1=0.5, beta_2=0.999)
    model.compile(optimizer=optimizer, loss='mse', metrics=['mae'])
    return model

def main():
    parser = argparse.ArgumentParser(description="Train Decoupled Bidirectional LSTM models for next_vel_x and next_vel_y prediction.")
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise',
                        help='Path to the base directory containing run data.')
    parser.add_argument('--epochs', type=int, default=100, help='Number of training epochs.')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size for training.')
    parser.add_argument('--sequence_length', type=int, default=10,
                        help='Sequence length for LSTM model.')
    parser.add_argument('--model_save_dir', type=str, default='./Model/BiLSTM_vel_predictor_decoupled',
                        help='Base directory to save trained LSTM velocity models and scalers.')
    
    args = parser.parse_args()
    os.makedirs(args.model_save_dir, exist_ok=True) # Save directly into this dir

    print("Loading and preprocessing data for Decoupled LSTM Velocity Predictors...")
    (
        X_train_seq, X_val_seq, X_test_seq,
        y_vx_train_seq, y_vx_val_seq, y_vx_test_seq,
        y_vy_train_seq, y_vy_val_seq, y_vy_test_seq,
        scaler_X, scaler_y_vel_x, scaler_y_vel_y
    ) = load_and_preprocess_data(args.data_dir, args.sequence_length)
    
    if X_train_seq.size == 0: 
        print("No training data available after processing for LSTM Velocity Predictor. Exiting.")
        return

    num_features = X_train_seq.shape[2]

    print("Building and training LSTM for next_vel_x...")
    model_vx = build_single_lstm_model(args.sequence_length, num_features, 'vx')
    model_vx.summary()
    vx_callbacks = [
        EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True),
        ModelCheckpoint(os.path.join(args.model_save_dir, 'lstm_next_vx_best_model.keras'), save_best_only=True, monitor='val_loss')
    ]
    model_vx.fit(X_train_seq, y_vx_train_seq, epochs=args.epochs, batch_size=args.batch_size, validation_data=(X_val_seq, y_vx_val_seq), callbacks=vx_callbacks, verbose=1)
    joblib.dump(scaler_y_vel_x, os.path.join(args.model_save_dir, 'lstm_next_vx_scaler_y.joblib'))
    if X_test_seq.size > 0 and y_vx_test_seq.size > 0:
        test_loss_vx, test_mae_vx = model_vx.evaluate(X_test_seq, y_vx_test_seq, verbose=0)
        print(f"Next_Vel_X LSTM Test Loss: {test_loss_vx:.4f}, MAE: {test_mae_vx:.4f}")

    print("\nBuilding and training LSTM for next_vel_y...")
    model_vy = build_single_lstm_model(args.sequence_length, num_features, 'vy')
    model_vy.summary()
    vy_callbacks = [
        EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True),
        ModelCheckpoint(os.path.join(args.model_save_dir, 'lstm_next_vy_best_model.keras'), save_best_only=True, monitor='val_loss')
    ]
    model_vy.fit(X_train_seq, y_vy_train_seq, epochs=args.epochs, batch_size=args.batch_size, validation_data=(X_val_seq, y_vy_val_seq), callbacks=vy_callbacks, verbose=1)
    joblib.dump(scaler_y_vel_y, os.path.join(args.model_save_dir, 'lstm_next_vy_scaler_y.joblib'))
    if X_test_seq.size > 0 and y_vy_test_seq.size > 0:
        test_loss_vy, test_mae_vy = model_vy.evaluate(X_test_seq, y_vy_test_seq, verbose=0)
        print(f"Next_Vel_Y LSTM Test Loss: {test_loss_vy:.4f}, MAE: {test_mae_vy:.4f}")

    joblib.dump(scaler_X, os.path.join(args.model_save_dir, 'lstm_scaler_X_vel_input.joblib'))
    print(f"\nDecoupled LSTM velocity predictor models and scalers saved to {args.model_save_dir}")

if __name__ == '__main__':
    main() 