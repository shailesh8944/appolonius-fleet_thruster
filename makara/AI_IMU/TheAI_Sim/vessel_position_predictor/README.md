# Vessel Position Predictor

A machine learning model for predicting vessel position based on IMU (accelerations, velocities) and rudder data. The model is designed to learn vessel dynamics and IMU noise to perform dead reckoning and predict future positions.

## Overview

This project implements several neural network architectures to predict vessel position based on time series data from IMU sensors and rudder inputs. The goal is to create a model that can accurately predict vessel position even in the absence of GPS signals, by learning the relationship between the vessel's movements and its position changes.

## Problem Statement

Marine vessels typically rely on GPS for position information, but GPS signals may not always be available or reliable due to signal interference, jamming, or other factors. Dead reckoning is a traditional approach where the vessel's position is estimated based on its previous position, speed, direction, and time elapsed. However, this approach accumulates errors over time.

This project uses machine learning to create a more accurate dead reckoning system by learning the vessel's dynamics model and accounting for sensor noise.

## Data Structure

The training data consists of synchronized measurements from various sensors:

- **Input Features (Body Frame)**:
  - `vel_x`: Forward velocity component (m/s)
  - `vel_y`: Lateral velocity component (m/s)
  - `accel_x`: Forward acceleration component (m/s²)
  - `accel_y`: Lateral acceleration component (m/s²)
  - `yaw`: Vessel orientation (radians)
  - `ang_vel_z`: Angular velocity around the z-axis (rad/s)
  - `rudder_angle`: Rudder position (radians)
  - `pos_x`: X-coordinate in the global frame (m) (included in input for improved prediction)
  - `pos_y`: Y-coordinate in the global frame (m) (included in input for improved prediction)

- **Output Features (Global Frame)**:
  - `pos_x`: X-coordinate in the global frame at the next time step (m)
  - `pos_y`: Y-coordinate in the global frame at the next time step (m)

## Position Input Enhancement

The model now includes current position in the input features, which significantly improves prediction accuracy, especially for autoregressive predictions. This makes the model more aligned with how traditional dead reckoning works - by starting from a known position and predicting the next position based on movement data.

Including position in the input allows the model to:
1. Learn the relationship between current position and next position
2. Better handle coordinate frame transformations
3. Produce more accurate long-term trajectory predictions
4. Reduce prediction drift in autoregressive mode

The model can be trained with or without position input using the `--no_position_input` flag.

## Model Architectures

The project implements several recurrent neural network architectures specifically designed for time series prediction:

### 1. LSTM (Long Short-Term Memory)

LSTM networks are well-suited for learning from sequential data with long-term dependencies. Our implementation uses:

```
Sequential model:
├── LSTM(64, return_sequences=True)
├── Dropout(0.2)
├── LSTM(32)
├── Dropout(0.2)
└── Dense(2)
```

- The first LSTM layer processes sequences of length `sequence_length` (default: 10)
- Each sequence contains 7-9 features (7 IMU features + optional 2 position features)
- The return_sequences=True parameter allows the second LSTM layer to process the full sequence
- Dropout layers help prevent overfitting
- The final Dense layer outputs 2 values: predicted pos_x and pos_y

### 2. Bidirectional LSTM (BiLSTM)

BiLSTM processes the input sequence in both forward and backward directions, capturing patterns from both past and future states:

```
Sequential model:
├── Bidirectional(LSTM(64, return_sequences=True))
├── Dropout(0.2)
├── Bidirectional(LSTM(32))
├── Dropout(0.2)
└── Dense(2)
```

- The bidirectional wrapper processes each sequence in both directions
- This approach can improve performance when both past and future context is relevant

### 3. GRU (Gated Recurrent Unit)

GRU is a simplified version of LSTM with fewer parameters, often resulting in faster training:

```
Sequential model:
├── GRU(64, return_sequences=True)
├── Dropout(0.2)
├── GRU(32)
├── Dropout(0.2)
└── Dense(2)
```

- GRU cells have fewer gates than LSTM cells, making them computationally more efficient
- They often perform comparably to LSTM for many tasks

### 4. CNN-LSTM

This hybrid architecture combines convolutional layers for feature extraction with LSTM layers for sequence processing:

```
Functional model:
├── Input(shape=(sequence_length, n_features))
├── Conv1D(filters=64, kernel_size=3, activation='relu')
├── MaxPooling1D(pool_size=2)
├── Conv1D(filters=32, kernel_size=3, activation='relu')
├── LSTM(32)
├── Dropout(0.2)
└── Dense(2)
```

- Conv1D layers extract features from each time window
- MaxPooling1D reduces dimensionality
- LSTM processes the extracted features as a sequence
- This architecture can better capture both local patterns and temporal dependencies

## Training Process

1. **Data Loading**: The `DataLoader` class loads synchronized.csv files containing vessel sensor data.
2. **Sequence Creation**: Data is transformed into overlapping sequences of length `sequence_length`.
3. **Data Scaling**: Features are normalized using `StandardScaler` to have zero mean and unit variance.
4. **Data Splitting**: Data is split into training (70%), validation (10%), and test (20%) sets.
5. **Model Training**: The model is trained with the Adam optimizer, using Mean Squared Error (MSE) as the loss function.
6. **Early Stopping**: Training stops when validation loss stops improving after a specified patience.
7. **Model Saving**: The best model (with lowest validation loss) is saved.

## Inference and Prediction

### Standard Prediction

During standard prediction, the model takes a sequence of `sequence_length` time steps and predicts the position for the next time step. This is useful for real-time position updates.

### Autoregressive Prediction

For longer-term forecasting, the model can perform autoregressive prediction:

1. Starting with an initial sequence of `sequence_length` time steps
2. Predicting the next position
3. Incorporating the prediction into the input sequence by sliding the window and updating the position features
4. Repeating the process to predict multiple steps ahead

This approach is useful for trajectory planning and longer-term position forecasting.

## Evaluation Metrics

The models are evaluated using:

- **Mean Squared Error (MSE)**: Measures the average squared difference between predicted and actual positions
- **Mean Absolute Error (MAE)**: Measures the average absolute difference between predicted and actual positions
- **Trajectory Visualization**: Visual comparison of predicted vs. actual vessel trajectories
- **Error Distribution Analysis**: Histograms of prediction errors to assess error patterns

## Visualizations

The `VesselVisualizer` class provides several visualization tools:

- **Training History**: Loss and metrics over epochs
- **Trajectory Plots**: Comparing predicted vs. actual trajectories
- **Time Series Plots**: Position components over time
- **Error Distribution**: Histograms of prediction errors
- **Autoregressive Prediction**: Long-term prediction visualization
- **Model Comparison**: Side-by-side comparison of different model architectures

All visualizations are saved to disk without displaying them by default, enabling autonomous running of the pipeline.

## Project Structure

- `data_loader.py`: Handles loading and preprocessing vessel data
- `model.py`: Contains model architectures and training logic
- `visualization.py`: Provides visualization utilities for model evaluation
- `train.py`: Script for training a single model
- `compare_models.py`: Script for comparing different model architectures
- `predict.py`: Script for making predictions with a trained model
- `run_pipeline.sh`: Shell script to run the complete pipeline

## Model Selection Considerations

When choosing between model architectures:

- **LSTM**: Good general-purpose choice with memory for long-term dependencies
- **BiLSTM**: Potentially better accuracy but higher computational cost
- **GRU**: Faster training with similar performance to LSTM
- **CNN-LSTM**: Best for capturing both local features and temporal patterns

The appropriate model depends on vessel dynamics complexity, available training data, and desired inference speed.

## Usage

### Training a Model

```bash
# Train with position as input (recommended)
python3 train.py --data_dir ../extracted_data_noNoise --model_type lstm --sequence_length 10 --batch_size 32 --epochs 100 --output_dir ./model_output

# Train without position as input
python3 train.py --data_dir ../extracted_data_noNoise --model_type lstm --sequence_length 10 --no_position_input --batch_size 32 --epochs 100 --output_dir ./model_output
```

### Comparing Models

```bash
# Compare models with position as input (recommended)
python3 compare_models.py --data_dir ../extracted_data_noNoise --model_types lstm gru cnn_lstm bilstm --sequence_length 10 --batch_size 32 --epochs 50 --output_dir ./model_comparison

# Compare models without position as input
python3 compare_models.py --data_dir ../extracted_data_noNoise --model_types lstm gru cnn_lstm bilstm --sequence_length 10 --no_position_input --batch_size 32 --epochs 50 --output_dir ./model_comparison
```

### Making Predictions

```bash
# Make predictions with saved model (automatically detects if position is needed as input)
python3 predict.py --model_path ./model_output/lstm_*/model --test_data_path ../extracted_data_noNoise/run17/synchronized.csv --output_dir ./predictions --autoregressive
```

### Running the Complete Pipeline

```bash
./run_pipeline.sh
```

## Command Line Arguments

### train.py
- `--data_dir`: Directory containing the synchronized.csv files
- `--limit_runs`: Limit the number of runs to load
- `--model_type`: Type of model to use ('lstm', 'bilstm', 'gru', 'cnn_lstm')
- `--sequence_length`: Number of timesteps in each sequence
- `--no_position_input`: Do not include position in the input features
- `--batch_size`: Batch size for training
- `--epochs`: Maximum number of epochs
- `--patience`: Patience for early stopping
- `--output_dir`: Directory to save results
- `--show_plots`: Display plots interactively (default: save only)
- `--autoregressive`: Enable autoregressive predictions

### compare_models.py
- `--data_dir`: Directory containing the synchronized.csv files
- `--limit_runs`: Limit the number of runs to load
- `--model_types`: List of model types to compare
- `--sequence_length`: Number of timesteps in each sequence
- `--no_position_input`: Do not include position in the input features
- `--batch_size`: Batch size for training
- `--epochs`: Maximum number of epochs
- `--patience`: Patience for early stopping
- `--output_dir`: Directory to save results
- `--show_plots`: Display plots interactively (default: save only)
- `--autoregressive`: Enable autoregressive predictions

### predict.py
- `--model_path`: Path to the trained model
- `--scalers_path`: Path to the scalers
- `--data_dir`: Directory containing the synchronized.csv files
- `--test_data_path`: Path to a specific test file
- `--limit_runs`: Limit the number of runs to load
- `--input_cols`: Comma-separated list of input column names
- `--output_cols`: Comma-separated list of output column names
- `--output_dir`: Directory to save predictions
- `--show_plots`: Display plots interactively (default: save only)
- `--autoregressive`: Enable autoregressive predictions

## Requirements

- TensorFlow 2.x
- NumPy
- Pandas
- Matplotlib
- scikit-learn
- tqdm

## Future Improvements

- Integration of uncertainty estimation in predictions
- Exploration of attention mechanisms for better feature relevance
- Testing of transformer-based architectures for improved sequence modeling
- Implementation of physics-informed neural networks to incorporate vessel dynamics knowledge
- Adaptive sequence length based on vessel maneuvers 