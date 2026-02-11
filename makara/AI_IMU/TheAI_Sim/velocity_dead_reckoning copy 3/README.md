# 2D Marine Vessel Dead Reckoning using Feedforward Neural Network

## Project Objective

The primary goal of this project is to perform 2D dead reckoning for a marine vessel using only IMU data (accelerations, angular velocity) and rudder input. A Feedforward Neural Network (FNN) is trained to predict the vessel's body-frame velocities (vx, vy) at the next timestep. These predicted velocities are then used in an auto-regressive manner to estimate the vessel's 2D trajectory (global x, y positions) over time.

## Directory Structure

```
TheAI_Sim/velocity_dead_reckoning/
├── data_loader.py          # PyTorch Dataset and DataLoader for vessel motion data
├── model.py                # Defines the FNN architecture for velocity prediction
├── train.py                # Script to train the FNN model
├── predict.py              # Script to perform auto-regressive trajectory prediction
├── evaluate.py             # Script to evaluate predictions and generate plots
├── utils.py                # Utility functions (e.g., coordinate transformations)
├── trained_models/         # Directory to save trained model weights (created by train.py)
│   └── velocity_predictor_fnn.pth (example)
├── predictions/            # Directory to save prediction CSV files (created by predict.py)
│   └── runX_predictions.csv (example)
├── plots/                  # Directory to save evaluation plots (created by evaluate.py)
│   └── runX_body_vx_comparison.png (example)
│   └── runX_body_vy_comparison.png (example)
│   └── runX_trajectory_comparison.png (example)
└── README.md               # This file
```

## File Descriptions

### `utils.py`
Contains helper functions used across the project:
*   `body_to_global_velocity(vx_body, vy_body, yaw)`: Converts velocities from the vessel's body frame to the global reference frame using the current yaw angle.
*   `integrate_position(x_prev, y_prev, vx_global, vy_global, dt)`: Integrates global frame velocities over a timestep `dt` to update the vessel's global (x, y) position.

### `data_loader.py`
Handles loading and preprocessing of the vessel motion data.
*   **`VesselMotionDataset(Dataset)` Class**:
    *   Reads `synchronized.csv` files from specified `runXX` subdirectories within the main data folder (`extracted_data_noNoise`).
    *   Input features for the FNN at time `t`: `ax_t, ay_t, wz_t, vx_t, vy_t, yaw_t, rudder_angle_t`.
    *   Target features for the FNN: `vx_{t+1}, vy_{t+1}` (body frame velocities at the next timestep).
    *   Concatenates data from multiple runs if not loading a specific run.
    *   Converts data to PyTorch tensors.
    *   Provides methods:
        *   `get_input_feature_names()`: Returns a list of input feature names.
        *   `get_run_data(run_name)`: Returns the original pandas DataFrame for a specific run.
        *   `get_run_data_tensors_for_prediction(run_name)`: Prepares and returns all necessary tensors for a single run for the auto-regressive prediction loop in `predict.py`. This includes the initial input vector, all "other" inputs (IMU, yaw, rudder) for the entire run, actual body velocities, and actual global positions/yaw for evaluation.

### `model.py`
Defines the neural network architecture.
*   **`VelocityPredictorFNN(nn.Module)` Class**:
    *   A simple Feedforward Neural Network.
    *   **Input**: 7 features (ax, ay, wz, vx_body, vy_body, yaw, rudder_angle at time `t`).
    *   **Output**: 2 features (predicted vx_body, vy_body at time `t+1`).
    *   Architecture: Input -> Linear(64) -> ReLU -> Linear(32) -> ReLU -> Linear(2) -> Output.

### `train.py`
Script responsible for training the `VelocityPredictorFNN` model.
*   Initializes `VesselMotionDataset` and `DataLoader`.
*   Optionally splits data into training and validation sets.
*   Initializes the `VelocityPredictorFNN` model and moves it to GPU if available.
*   Defines `MSELoss` as the loss function and `Adam` as the optimizer.
*   Implements the training loop: iterates over epochs and batches, performs forward/backward passes, and updates model weights.
*   Prints training (and validation) loss per epoch.
*   Saves the trained model's state dictionary (e.g., to `trained_models/velocity_predictor_fnn.pth`).
*   Accepts command-line arguments for data directory, epochs, batch size, learning rate, model save path, and validation split ratio.

### `predict.py`
Performs dead reckoning using the trained model to predict a vessel's trajectory for a specific run.
*   Loads a pre-trained `VelocityPredictorFNN` model.
*   Uses `VesselMotionDataset` to fetch the data for a specified `run_id`.
*   Implements an **auto-regressive inference loop** (detailed in "Prediction Methodology" below).
*   Stores the sequence of predicted body velocities and predicted global positions.
*   Saves the actual and predicted data (velocities, positions) to a CSV file (e.g., `predictions/runX_predictions.csv`).
*   Accepts command-line arguments for data directory, run ID, model path, and output directory.

### `evaluate.py`
Evaluates the performance of the dead reckoning predictions.
*   Loads the prediction results CSV file generated by `predict.py`.
*   Calculates evaluation metrics:
    *   Mean Squared Error (MSE) for predicted `vx_body` and `vy_body`.
    *   Final position error (Euclidean distance between actual and predicted final global x, y).
*   Generates and saves plots using `matplotlib`:
    *   Actual vs. Predicted body velocities (`vx`, `vy`) over time.
    *   Actual vs. Predicted trajectory (global `x` vs. `y`).
*   Accepts command-line arguments for the predictions CSV path and the directory to save plots.

## Prediction Methodology (Auto-Regressive)

The `predict.py` script employs an auto-regressive approach for trajectory estimation:

1.  **Initialization**:
    *   The process starts with the actual (ground truth) state of the vessel at the very first timestep (`t=0`) of a specific run. This includes accelerations (`ax_0, ay_0`), angular velocity (`wz_0`), body-frame velocities (`vx_0, vy_0`), yaw angle (`yaw_0`), and rudder angle (`rudder_angle_0`).
    *   The initial global position (`x_0, y_0`) is also taken from the ground truth.

2.  **Iterative Loop**: For each subsequent timestep `t`:
    *   **Input Construction**: An input vector is formed using:
        *   Actual IMU readings (`ax_t, ay_t, wz_t`), actual `yaw_t`, and actual `rudder_angle_t` from the dataset for the current timestep `t`.
        *   The body-frame velocities (`vx_body_t_pred, vy_body_t_pred`) that were *predicted by the model in the previous step*. (For the first prediction, the ground truth `vx_0, vy_0` are used).
    *   **Model Prediction**: This input vector is fed to the trained FNN, which outputs the predicted body-frame velocities for the *next* timestep: `vx_body_{t+1}_pred, vy_body_{t+1}_pred`.
    *   **State Update & Integration**:
        *   The predicted `vx_body_{t+1}_pred, vy_body_{t+1}_pred` are converted to global frame velocities using the *actual* yaw angle `yaw_{t+1}` from the dataset.
        *   These global velocities are integrated (using `Δt = 0.1s`) with the previously predicted global position (`x_t_pred, y_t_pred`) to get the new predicted global position (`x_{t+1}_pred, y_{t+1}_pred`).
    *   The predicted `vx_body_{t+1}_pred, vy_body_{t+1}_pred` are then used as inputs for the next iteration.

This cycle continues, with the model's own velocity predictions being fed back into its input for subsequent steps, allowing it to "roll out" a trajectory.

## Setup and Dependencies

Ensure you have Python 3.x installed. The primary dependencies are:
*   PyTorch
*   Pandas
*   NumPy
*   Matplotlib
*   Scikit-learn

You can install these using pip:
```bash
pip install torch pandas numpy matplotlib scikit-learn
```
If you have a CUDA-enabled GPU, ensure you install the appropriate PyTorch version with CUDA support for faster training and inference.

## How to Run

All scripts are intended to be run from the `TheAI_Sim/velocity_dead_reckoning/` directory.

1.  **Train the Model:**
    ```bash
    python train.py --data_dir ../../extracted_data_noNoise --epochs 100 --batch_size 64 --lr 0.001 --save_path ./trained_models/my_vessel_model.pth --val_split 0.1
    ```
    *   `--data_dir`: Path to the root directory containing `runXX` folders (e.g., `../../extracted_data_noNoise`).
    *   `--epochs`: Number of training epochs.
    *   `--batch_size`: Batch size for training.
    *   `--lr`: Learning rate.
    *   `--save_path`: Path where the trained model weights will be saved.
    *   `--val_split`: Proportion of data to use for validation (e.g., 0.1 for 10%). Set to 0 for no validation.

2.  **Make Predictions:**
    After training, use `predict.py` to generate a trajectory for a specific run:
    ```bash
    python predict.py --data_dir ../../extracted_data_noNoise --run_id run4 --model_path ./trained_models/my_vessel_model.pth --output_dir ./predictions
    ```
    *   `--run_id`: The ID of the run to predict (e.g., `run4`, `run10`).
    *   `--model_path`: Path to the trained model file.
    *   `--output_dir`: Directory where the prediction CSV will be saved.

3.  **Evaluate Predictions:**
    Analyze the generated predictions:
    ```bash
    python evaluate.py --predictions_csv ./predictions/run4_predictions.csv --plots_dir ./plots
    ```
    *   `--predictions_csv`: Path to the CSV file generated by `predict.py`.
    *   `--plots_dir`: Directory where evaluation plots will be saved.

## Data Format

The system expects `synchronized.csv` files within each `runXX` subfolder. The CSV files should have the following columns (order matters for default parsing, though names are used):

*   `timestamp`: Timestamp of the reading.
*   `pos_x`: Global X position.
*   `pos_y`: Global Y position.
*   `vel_x`: Body-frame velocity along the vessel's x-axis (forward).
*   `vel_y`: Body-frame velocity along the vessel's y-axis (starboard/right).
*   `yaw`: Yaw angle of the vessel (radians).
*   `accel_x`: Body-frame acceleration along the vessel's x-axis.
*   `accel_y`: Body-frame acceleration along the vessel's y-axis.
*   `ang_vel_z`: Body-frame angular velocity around the vessel's z-axis (yaw rate).
*   `rudder_angle`: Rudder angle.

A fixed timestep `Δt = 0.1s` is assumed between consecutive rows for integration purposes.

## Potential Improvements / Future Work

*   **More Complex Models**: Explore Recurrent Neural Networks (RNNs) like LSTMs or GRUs, or Transformer models, which might better capture temporal dependencies.
*   **Yaw Prediction**: Extend the model to also predict yaw rate or yaw angle changes, instead of relying on the ground truth yaw during prediction for coordinate transformation.
*   **Data Augmentation**: If dataset size is limited, explore augmentation techniques suitable for time-series sensor data.
*   **State Estimation/Filtering**: Combine the neural network predictions with a traditional filter (e.g., Kalman Filter, Particle Filter) for potentially more robust state estimation.
*   **Hyperparameter Optimization**: Perform systematic hyperparameter tuning for the neural network architecture, learning rate, batch size, etc.
*   **Error Analysis**: More in-depth analysis of prediction errors across different runs and conditions.
*   **Resampling/Interpolation**: If `dt` is not perfectly constant in the source data, implement resampling or use the exact `dt` between samples. 