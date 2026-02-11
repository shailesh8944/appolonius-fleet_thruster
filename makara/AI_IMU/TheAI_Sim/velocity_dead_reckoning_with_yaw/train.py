import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
import os
import argparse

from data_loader import VesselMotionDataset # Assuming data_loader.py is in the same directory
from model import VelocityPredictorFNN       # Assuming model.py is in the same directory

def train_model(data_dir, num_epochs=50, batch_size=32, learning_rate=0.001, model_save_path='velocity_predictor_fnn.pth', val_split_ratio=0.1):
    """
    Trains the VelocityPredictorFNN model.

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

    # 1. Initialize Dataset and DataLoader
    full_dataset = VesselMotionDataset(data_dir=data_dir)
    
    if len(full_dataset) == 0:
        print("Dataset is empty. Exiting training.")
        return

    if val_split_ratio > 0:
        val_size = int(len(full_dataset) * val_split_ratio)
        train_size = len(full_dataset) - val_size
        train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
        print(f"Training on {len(train_dataset)} samples, validating on {len(val_dataset)} samples.")
    else:
        train_loader = DataLoader(full_dataset, batch_size=batch_size, shuffle=True)
        val_loader = None
        print(f"Training on {len(full_dataset)} samples. No validation split.")

    # 2. Initialize Model
    # Ensure input_size matches the number of features from the dataset
    num_input_features = len(full_dataset.get_input_feature_names())
    model = VelocityPredictorFNN(input_size=num_input_features).to(device)

    # 3. Define Loss Function and Optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    # 4. Training Loop
    print("Starting training...")
    for epoch in range(num_epochs):
        model.train() # Set model to training mode
        running_loss = 0.0
        for i, (inputs, targets) in enumerate(train_loader):
            inputs, targets = inputs.to(device), targets.to(device)

            # Forward pass
            outputs = model(inputs)
            loss = criterion(outputs, targets)

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
                for inputs_val, targets_val in val_loader:
                    inputs_val, targets_val = inputs_val.to(device), targets_val.to(device)
                    outputs_val = model(inputs_val)
                    loss_val = criterion(outputs_val, targets_val)
                    val_loss += loss_val.item()
            avg_val_loss = val_loss / len(val_loader)
            print(f"Epoch [{epoch+1}/{num_epochs}], Train Loss: {avg_train_loss:.6f}, Val Loss: {avg_val_loss:.6f}")
        else:
            print(f"Epoch [{epoch+1}/{num_epochs}], Train Loss: {avg_train_loss:.6f}")

    # 5. Save the Model
    # Ensure the directory for model_save_path exists
    save_dir = os.path.dirname(model_save_path)
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
        
    torch.save(model.state_dict(), model_save_path)
    print(f"Trained model saved to {model_save_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train Velocity Predictor FNN Model')
    parser.add_argument('--data_dir', type=str, default='../../extracted_data_noNoise', help='Directory containing the runXX folders')
    parser.add_argument('--epochs', type=int, default=50, help='Number of training epochs')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size for training')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    parser.add_argument('--save_path', type=str, default='./trained_models/velocity_predictor_fnn.pth', help='Path to save the trained model')
    parser.add_argument('--val_split', type=float, default=0.1, help='Ratio of data for validation (0 for no validation)')

    args = parser.parse_args()

    train_model(
        data_dir=args.data_dir,
        num_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        model_save_path=args.save_path,
        val_split_ratio=args.val_split
    ) 