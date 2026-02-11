import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
import os
import argparse
import json # For saving scaler params

from data_loader import VesselMotionDataset 
from model import StatePredictorFNN

# Define the scaler params path next to the model path
def get_scaler_path(model_save_path):
    base, ext = os.path.splitext(model_save_path)
    return base + "_scaler_params.json"

def train_model(data_dir, num_epochs=50, batch_size=32, learning_rate=0.001, 
                model_save_path='./trained_models/state_delta_predictor_fnn.pth', 
                val_split_ratio=0.1):
    """
    Trains the StatePredictorFNN model.

    Args:
        data_dir (str): Directory containing the runXX folders.
        num_epochs (int): Number of epochs to train for.
        batch_size (int): Batch size for training.
        learning_rate (float): Learning rate for the optimizer.
        model_save_path (str): Path to save the trained model.
        val_split_ratio (float): Ratio of data to use for validation (0 to 1).
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # Dataset initialized without scaler_params, so it calculates them
    full_dataset = VesselMotionDataset(data_dir=data_dir)
    
    if len(full_dataset) == 0:
        print("Dataset is empty. Exiting training.")
        return

    # Save scaler parameters calculated by the dataset
    scaler_params = full_dataset.get_scaler_params()
    scaler_save_path = get_scaler_path(model_save_path)
    save_dir = os.path.dirname(scaler_save_path)
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
    with open(scaler_save_path, 'w') as f:
        json.dump(scaler_params, f, indent=4)
    print(f"Scaler parameters saved to {scaler_save_path}")

    if val_split_ratio > 0 and val_split_ratio < 1.0:
        val_size = int(len(full_dataset) * val_split_ratio)
        train_size = len(full_dataset) - val_size
        if train_size <= 0 or val_size <=0:
            print(f"Train or validation size is 0 after split. Training on full dataset. Train: {train_size}, Val: {val_size}")
            train_loader = DataLoader(full_dataset, batch_size=batch_size, shuffle=True)
            val_loader = None
        else:
            train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])
            train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
            val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
            print(f"Training on {len(train_dataset)} samples, validating on {len(val_dataset)} samples.")
    else:
        train_loader = DataLoader(full_dataset, batch_size=batch_size, shuffle=True)
        val_loader = None
        print(f"Training on {len(full_dataset)} samples. No validation split (val_split_ratio: {val_split_ratio}).")

    # 2. Initialize Model
    # Get input and output feature sizes dynamically from the dataset
    num_input_features = len(full_dataset.get_input_feature_names()) # Should be 8
    num_output_features = len(full_dataset.output_features_delta) # Should be 3 (for deltas)
    
    print(f"Model Input Features: {num_input_features}, Output Features (Deltas): {num_output_features}")
    model = StatePredictorFNN(input_size=num_input_features, output_size=num_output_features).to(device)

    # 3. Define Loss Function and Optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    # 4. Training Loop
    print("Starting training for delta predictor with scaled data...")
    best_val_loss = float('inf')
    epochs_no_improve = 0
    patience = 10 # For early stopping

    for epoch in range(num_epochs):
        model.train() # Set model to training mode
        running_loss = 0.0
        for i, (scaled_inputs, scaled_delta_targets) in enumerate(train_loader):
            scaled_inputs, scaled_delta_targets = scaled_inputs.to(device), scaled_delta_targets.to(device)

            # Forward pass
            predicted_scaled_deltas = model(scaled_inputs)
            loss = criterion(predicted_scaled_deltas, scaled_delta_targets)

            # Backward pass and optimization
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
        
        avg_train_loss = running_loss / len(train_loader)
        
        if val_loader:
            model.eval() # Set model to evaluation mode
            val_loss = 0.0
            with torch.no_grad():
                for scaled_inputs_val, scaled_delta_targets_val in val_loader:
                    scaled_inputs_val, scaled_delta_targets_val = scaled_inputs_val.to(device), scaled_delta_targets_val.to(device)
                    predicted_scaled_deltas_val = model(scaled_inputs_val)
                    loss_val = criterion(predicted_scaled_deltas_val, scaled_delta_targets_val)
                    val_loss += loss_val.item()
            avg_val_loss = val_loss / len(val_loader)
            print(f"Epoch [{epoch+1}/{num_epochs}], Train Loss: {avg_train_loss:.6f}, Val Loss: {avg_val_loss:.6f}")

            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                model_save_dir = os.path.dirname(model_save_path)
                if model_save_dir and not os.path.exists(model_save_dir):
                    os.makedirs(model_save_dir, exist_ok=True)
                torch.save(model.state_dict(), model_save_path)
                print(f"Model improved and saved to {model_save_path}")
                epochs_no_improve = 0
            else:
                epochs_no_improve += 1
                if epochs_no_improve >= patience:
                    print(f"Early stopping triggered after {patience} epochs with no improvement.")
                    break 
        else:
            print(f"Epoch [{epoch+1}/{num_epochs}], Train Loss: {avg_train_loss:.6f}")
            # Save model at the end if no validation
            if epoch == num_epochs -1:
                model_save_dir = os.path.dirname(model_save_path)
                if model_save_dir and not os.path.exists(model_save_dir):
                    os.makedirs(model_save_dir, exist_ok=True)
                torch.save(model.state_dict(), model_save_path)
                print(f"Trained model saved to {model_save_path} (no validation set used for early stopping).")

    if not val_loader and num_epochs > 0 and (epochs_no_improve < patience or epoch < num_epochs-1) : 
        pass 
    elif val_loader and epochs_no_improve < patience:
         print(f"Training finished. Best model saved to {model_save_path} with Val Loss: {best_val_loss:.6f}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train State Delta Predictor FNN Model with Data Scaling')
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise', help='Directory containing the runXX folders')
    parser.add_argument('--epochs', type=int, default=50, help='Number of training epochs')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size for training')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    parser.add_argument('--save_path', type=str, default='./trained_models/state_delta_predictor_fnn.pth', help='Path to save the trained model')
    parser.add_argument('--val_split', type=float, default=0.1, help='Ratio of data for validation (0 for no validation, 0 < ratio < 1)')

    args = parser.parse_args()
    
    if not (0 <= args.val_split < 1):
        raise ValueError("val_split_ratio must be between 0 (inclusive) and 1 (exclusive).")

    train_model(
        data_dir=args.data_dir,
        num_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        model_save_path=args.save_path,
        val_split_ratio=args.val_split
    ) 