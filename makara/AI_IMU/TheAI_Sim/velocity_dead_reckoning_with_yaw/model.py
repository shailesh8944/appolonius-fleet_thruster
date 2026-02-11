import torch
import torch.nn as nn

class VelocityPredictorFNN(nn.Module):
    def __init__(self, input_size, hidden_size1=64, hidden_size2=32, output_size=2):
        """
        Feedforward Neural Network to predict body frame velocities (vx, vy) at t+1.

        Args:
            input_size (int): Number of input features 
                                (ax, ay, wz, vx_t, vy_t, yaw_t, rudder_angle_t).
            hidden_size1 (int): Number of neurons in the first hidden layer.
            hidden_size2 (int): Number of neurons in the second hidden layer.
            output_size (int): Number of output features (vx_t+1, vy_t+1).
        """
        super(VelocityPredictorFNN, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size1)
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size1, hidden_size2)
        self.relu2 = nn.ReLU()
        self.fc3 = nn.Linear(hidden_size2, output_size)

    def forward(self, x):
        """
        Forward pass of the FNN.

        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, input_size).

        Returns:
            torch.Tensor: Output tensor of shape (batch_size, output_size), 
                          representing predicted (vx_t+1, vy_t+1).
        """
        out = self.fc1(x)
        out = self.relu1(out)
        out = self.fc2(out)
        out = self.relu2(out)
        out = self.fc3(out)
        return out

if __name__ == '__main__':
    # Example usage:
    # From data_loader.py, input_features = ['accel_x', 'accel_y', 'ang_vel_z', 'vel_x', 'vel_y', 'yaw', 'rudder_angle']
    num_input_features = 7 
    model = VelocityPredictorFNN(input_size=num_input_features)
    
    # Create a dummy input tensor (batch_size=10, num_features=7)
    dummy_input = torch.randn(10, num_input_features)
    
    # Get model output
    output = model(dummy_input)
    
    print(f"Model Input Shape: {dummy_input.shape}")
    print(f"Model Output Shape: {output.shape}")
    print("Model Architecture:")
    print(model) 