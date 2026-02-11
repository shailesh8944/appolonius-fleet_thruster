# 2D Marine Vessel Dead Reckoning using Feedforward Neural Network

## Project Objective

The primary goal of this project is to perform 2D dead reckoning for a marine vessel using IMU data (accelerations, angular velocity) and rudder input. A Feedforward Neural Network (FNN) is trained to predict changes (deltas) in the vessel's body-frame velocities (`vx, vy`) and `yaw` angle at the next timestep. These predicted deltas are used to update the vessel's state, and the process is repeated auto-regressively to estimate the vessel's 2D trajectory (global `x, y` positions) and orientation (`yaw`) over time. The project incorporates cyclical representation for yaw input (`sin(yaw), cos(yaw)`) and standardization of input and target (delta) features.

## Directory Structure

```
TheAI_Sim/velocity_dead_reckoning/
├── data_loader.py          # PyTorch Dataset and DataLoader
├── model.py                # FNN architecture definition
├── train.py                # Script to train the FNN model
├── predict.py              # Script for auto-regressive trajectory prediction
├── evaluate.py             # Script to evaluate predictions and generate plots
├── utils.py                # Utility functions (coordinate transformations, integration)
├── trained_models/         # Directory for trained model weights and scaler params
│   ├── state_delta_predictor_fnn.pth (example)
│   └── state_delta_predictor_fnn_scaler_params.json (example)
├── predictions/            # Directory for prediction CSV files
│   └── runX_delta_predictions.csv (example)
├── plots/                  # Directory for evaluation plots
│   └── runX_body_vx_comparison.png (example)
│   └── runX_body_vy_comparison.png (example)
│   └── runX_yaw_comparison.png (example)
│   └── runX_trajectory_comparison.png (example)
└── README.md               # This file
```

## File Descriptions

### `utils.py`
Contains helper functions:
*   `body_to_global_velocity(vx_body, vy_body, yaw)`: Converts body-frame velocities to the global frame.
*   `integrate_position(x_prev, y_prev, vx_global, vy_global, dt)`: Updates global position using global velocities.

### `data_loader.py`
Handles data loading, preprocessing, and standardization.
*   **`VesselMotionDataset(Dataset)` Class**:
    *   Reads `synchronized.csv` from `runXX` folders.
    *   **Input Features (processed, at time `t`)**: `accel_x, accel_y, ang_vel_z, vel_x, vel_y, sin(yaw_t), cos(yaw_t), rudder_angle`. (8 features)
    *   **Target Features (deltas, from `t` to `t+1`)**: `delta_vel_x, delta_vel_y, delta_yaw`. (3 features)
    *   **Standardization**:
        *   **Training Mode** (if `scaler_params` not provided to `__init__`):
            *   Calculates means and standard deviations for the 8 input features and the 3 delta target features from the training dataset.
            *   Stores these as `self.input_means`, `self.input_stds`, `self.target_means`, `self.target_stds`.
            *   Applies standardization: `(value - mean) / std` to both input data and target delta data.
        *   **Prediction Mode** (if `scaler_params` provided to `__init__`):
            *   Uses the provided means and stds for standardization.
    *   Provides methods:
        *   `get_input_feature_names()`: Returns processed input feature names.
        *   `get_scaler_params()`: Returns a dictionary of calculated `input_means`, `input_stds`, `target_means`, `target_stds` (for deltas) as lists, used for saving.
        *   `get_run_data_tensors_for_prediction(run_name)`: For a specific run, returns:
            *   `initial_model_input_tensor`: The first input vector (scaled using the dataset's scalers).
            *   `full_run_exogenous_inputs_tensor_raw`: Unscaled `ax, ay, wz, rudder_angle` for the entire run.
            *   `initial_state_tensor`: Raw `vx_0, vy_0, yaw_0, pos_x_0, pos_y_0`.
            *   `actual_original_outputs_and_pos_tensor`: Raw ground truth `vx, vy, yaw, pos_x, pos_y` for the run.

### `model.py`
Defines the neural network.
*   **`StatePredictorFNN(nn.Module)` Class**:
    *   Feedforward Neural Network.
    *   **Input**: 8 features (scaled: `ax, ay, wz, vx, vy, sin(yaw), cos(yaw), rudder_angle`).
    *   **Output**: 3 features (predicted *scaled* deltas: `delta_vx, delta_vy, delta_yaw`).
    *   Architecture: Example: Input -> Linear(64) -> ReLU -> Linear(32) -> ReLU -> Linear(3) -> Output.

### `train.py`
Trains the `StatePredictorFNN` model.
*   Initializes `VesselMotionDataset` (which calculates scaler parameters as `scaler_params` are not passed).
*   Calls `dataset.get_scaler_params()` and saves these parameters to a JSON file (e.g., `trained_models/state_delta_predictor_fnn_scaler_params.json`).
*   The `DataLoader` provides batches of scaled inputs and scaled delta targets.
*   Initializes `StatePredictorFNN`.
*   Uses `MSELoss` and `Adam` optimizer.
*   Trains the model using scaled data. The model learns to predict scaled deltas from scaled inputs.
*   Includes early stopping based on validation loss.
*   Saves the trained model (e.g., `trained_models/state_delta_predictor_fnn.pth`).
*   Command-line arguments for configuration.

### `predict.py`
Performs auto-regressive dead reckoning.
*   Loads a pre-trained `StatePredictorFNN` model and its corresponding `*_scaler_params.json` file.
*   The scaler parameters (input means/stds, target delta means/stds) are loaded and converted to tensors.
*   Initializes `VesselMotionDataset` for a specific `run_id`, passing the loaded `scaler_params`. This ensures `get_run_data_tensors_for_prediction` uses these scalers.
*   Retrieves `initial_model_input_t0` (already scaled by the dataset), `full_run_exogenous_inputs_raw` (unscaled), and initial state values.
*   Implements an **auto-regressive inference loop** (see "Prediction Methodology").
*   Saves actual and predicted absolute values (velocities, yaw, positions) to a CSV (e.g., `predictions/runX_delta_predictions.csv`).
*   Command-line arguments for configuration.

### `evaluate.py`
Evaluates prediction performance.
*   Loads `*_delta_predictions.csv`.
*   Calculates MSE for predicted `vx_body`, `vy_body`, and `yaw`.
*   Calculates final position error.
*   Generates and saves plots:
    *   Actual vs. Predicted body velocities (`vx`, `vy`) and `yaw` over time.
    *   Actual vs. Predicted trajectory (global `x` vs. `y`).
*   Command-line arguments for paths.

## Prediction Methodology (Auto-Regressive with Deltas and Standardization)

The `predict.py` script uses the following auto-regressive approach:

1.  **Initialization**:
    *   Load the trained model and scaler parameters (`input_means`, `input_stds`, `target_delta_means`, `target_delta_stds`).
    *   Start with the actual (ground truth) state at `t=0` for the chosen run: `vx_0, vy_0, yaw_0, pos_x_0, pos_y_0`.
    *   The very first input to the model (`initial_model_input_t0`) is obtained from `VesselMotionDataset` (which uses the loaded scalers, so this input is already scaled). It's typically based on ground truth values at `t=0`.

2.  **Iterative Loop**: For each subsequent timestep `t` (to predict for `t+1`):
    *   **Current Input Tensor**: The `current_model_input_tensor` (scaled) is fed to the model.
    *   **Model Prediction**: The FNN outputs *scaled* predicted deltas: `scaled_delta_vx_{t+1}, scaled_delta_vy_{t+1}, scaled_delta_yaw_{t+1}`.
    *   **Unscale Deltas**: These scaled deltas are unscaled using the loaded `target_delta_means` and `target_delta_stds`:
        `delta_unscaled = delta_scaled * target_stds + target_means`.
    *   **Update Absolute State**: The unscaled deltas are added to the current absolute state:
        *   `current_vx_body_{t+1} = current_vx_body_t + unscaled_delta_vx_{t+1}`
        *   `current_vy_body_{t+1} = current_vy_body_t + unscaled_delta_vy_{t+1}`
        *   `current_yaw_{t+1} = current_yaw_t + unscaled_delta_yaw_{t+1}`
        *   `current_yaw_{t+1}` is normalized to `[-π, π]`.
    *   **Store Predictions**: The new absolute `current_vx_body_{t+1}`, `current_vy_body_{t+1}`, `current_yaw_{t+1}` are stored.
    *   **Global Position Integration**:
        *   The `current_vx_body_{t+1}` and `current_vy_body_{t+1}` are converted to global velocities using the newly predicted `current_yaw_{t+1}`.
        *   These global velocities are integrated to get `current_x_global_{t+1}, current_y_global_{t+1}`. These are also stored.
    *   **Prepare Input for Next Step (if not the last step)**:
        1.  Get raw exogenous inputs for `t+1`: `ax_{t+1}_raw, ay_{t+1}_raw, wz_{t+1}_raw, rudder_{t+1}_raw` from `full_run_exogenous_inputs_raw`.
        2.  Construct the *raw* feature vector for the next model input using:
            *   `ax_{t+1}_raw, ay_{t+1}_raw, wz_{t+1}_raw`
            *   `current_vx_body_{t+1}` (predicted absolute)
            *   `current_vy_body_{t+1}` (predicted absolute)
            *   `sin(current_yaw_{t+1})` (from predicted absolute yaw)
            *   `cos(current_yaw_{t+1})` (from predicted absolute yaw)
            *   `rudder_{t+1}_raw`
        3.  **Scale Input**: This raw feature vector is then scaled using the loaded `input_means` and `input_stds`:
            `next_model_input_scaled = (next_model_input_raw - input_means) / input_stds`.
        4.  This `next_model_input_scaled` becomes the `current_model_input_tensor` for the next iteration.

This cycle continues, feeding back processed versions of its own predictions.

## Setup and Dependencies

Ensure you have Python 3.x installed. The primary dependencies are:
*   PyTorch
*   Pandas
*   NumPy
*   Matplotlib
(Scikit-learn might not be strictly necessary if only used for splitting, which can be done with PyTorch utils, but it's a common ML library).

You can install these using pip:
```bash
pip install torch pandas numpy matplotlib
```
If you have a CUDA-enabled GPU, ensure you install the appropriate PyTorch version.

## How to Run

All scripts are intended to be run from the `TheAI_Sim/velocity_dead_reckoning/` directory.

1.  **Train the Model:**
    ```bash
    python train.py \\
        --data_dir ../../extracted_data_noNoise \\
        --epochs 100 \\
        --batch_size 64 \\
        --lr 0.001 \\
        --save_path ./trained_models/state_delta_predictor_fnn.pth \\
        --val_split 0.1
    ```
    *   This will also save `state_delta_predictor_fnn_scaler_params.json` in the same directory as the model.

2.  **Make Predictions:**
    ```bash
    python predict.py \\
        --data_dir ../../extracted_data_noNoise \\
        --run_id run4 \\
        --model_path ./trained_models/state_delta_predictor_fnn.pth \\
        --scaler_params_path ./trained_models/state_delta_predictor_fnn_scaler_params.json \\
        --output_dir ./predictions
    ```
    *   `--scaler_params_path`: Path to the JSON file with scaler parameters saved during training.
    *   Prediction output will be e.g., `predictions/run4_delta_predictions.csv`.

3.  **Evaluate Predictions:**
    ```bash
    python evaluate.py \\
        --predictions_csv ./predictions/run4_delta_predictions.csv \\
        --plots_dir ./plots
    ```

## Data Format

The system expects `synchronized.csv` files with `dt = 0.1s` assumed. Key columns:
`timestamp, pos_x, pos_y, vel_x, vel_y, yaw, accel_x, accel_y, ang_vel_z, rudder_angle`.
(Refer to `data_loader.py` for exact features used).

## Potential Improvements / Future Work

*   **More Complex Models**: Explore Recurrent Neural Networks (RNNs) like LSTMs or GRUs, or Transformer models.
*   **Direct Position Prediction**: Train the model to predict position deltas or absolute positions directly, potentially simplifying the integration step or error accumulation.
*   **Data Augmentation**: Explore augmentation for time-series sensor data.
*   **State Estimation/Filtering**: Combine NN predictions with a Kalman Filter.
*   **Hyperparameter Optimization**: Systematic tuning for network architecture and training parameters.
*   **Uncertainty Quantification**: Estimate prediction uncertainty.
*   **Variable Timestep Handling**: If `dt` is not constant, incorporate it into the model or data processing. 