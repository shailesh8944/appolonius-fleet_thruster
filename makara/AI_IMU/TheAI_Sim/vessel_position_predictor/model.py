import tensorflow as tf
from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import Dense, LSTM, Bidirectional, Dropout, GRU, Conv1D, MaxPooling1D, Flatten, Input, concatenate
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
import numpy as np
from typing import Dict, Tuple, List, Optional
import os

class ModelFactory:
    @staticmethod
    def create_lstm_model(input_shape: Tuple[int, int], output_dim: int = 2) -> Model:
        """
        Create a simple LSTM model
        
        Args:
            input_shape: Shape of input (sequence_length, features)
            output_dim: Dimension of output
            
        Returns:
            Keras Model
        """
        model = Sequential()
        model.add(LSTM(64, input_shape=input_shape, return_sequences=True))
        model.add(Dropout(0.2))
        model.add(LSTM(32))
        model.add(Dropout(0.2))
        model.add(Dense(output_dim))
        
        model.compile(optimizer='adam', loss='mse', metrics=['mae'])
        return model
    
    @staticmethod
    def create_bidirectional_lstm_model(input_shape: Tuple[int, int], output_dim: int = 2) -> Model:
        """
        Create a bidirectional LSTM model
        
        Args:
            input_shape: Shape of input (sequence_length, features)
            output_dim: Dimension of output
            
        Returns:
            Keras Model
        """
        model = Sequential()
        model.add(Bidirectional(LSTM(64, return_sequences=True), input_shape=input_shape))
        model.add(Dropout(0.2))
        model.add(Bidirectional(LSTM(32)))
        model.add(Dropout(0.2))
        model.add(Dense(output_dim))
        
        model.compile(optimizer='adam', loss='mse', metrics=['mae'])
        return model
    
    @staticmethod
    def create_gru_model(input_shape: Tuple[int, int], output_dim: int = 2) -> Model:
        """
        Create a GRU model
        
        Args:
            input_shape: Shape of input (sequence_length, features)
            output_dim: Dimension of output
            
        Returns:
            Keras Model
        """
        model = Sequential()
        model.add(GRU(64, input_shape=input_shape, return_sequences=True))
        model.add(Dropout(0.2))
        model.add(GRU(32))
        model.add(Dropout(0.2))
        model.add(Dense(output_dim))
        
        model.compile(optimizer='adam', loss='mse', metrics=['mae'])
        return model
    
    @staticmethod
    def create_cnn_lstm_model(input_shape: Tuple[int, int], output_dim: int = 2) -> Model:
        """
        Create a CNN-LSTM model
        
        Args:
            input_shape: Shape of input (sequence_length, features)
            output_dim: Dimension of output
            
        Returns:
            Keras Model
        """
        # CNN part
        inputs = Input(shape=input_shape)
        x = Conv1D(filters=64, kernel_size=3, activation='relu')(inputs)
        x = MaxPooling1D(pool_size=2)(x)
        x = Conv1D(filters=32, kernel_size=3, activation='relu')(x)
        
        # LSTM part
        x = LSTM(32, return_sequences=False)(x)
        x = Dropout(0.2)(x)
        
        # Output
        outputs = Dense(output_dim)(x)
        
        model = Model(inputs=inputs, outputs=outputs)
        model.compile(optimizer='adam', loss='mse', metrics=['mae'])
        return model

class VesselPositionPredictor:
    def __init__(self, model_type: str = 'lstm', sequence_length: int = 10, n_features: int = 7, output_dim: int = 2):
        """
        Initialize the vessel position predictor
        
        Args:
            model_type: Type of model to use ('lstm', 'bilstm', 'gru', 'cnn_lstm')
            sequence_length: Number of timesteps in input sequence
            n_features: Number of features in input
            output_dim: Dimension of output
        """
        self.model_type = model_type
        self.sequence_length = sequence_length
        self.n_features = n_features
        self.output_dim = output_dim
        self.model = self._build_model()
        self.history = None
    
    def _build_model(self) -> Model:
        """
        Build the model
        
        Returns:
            Keras Model
        """
        input_shape = (self.sequence_length, self.n_features)
        
        if self.model_type == 'lstm':
            return ModelFactory.create_lstm_model(input_shape, self.output_dim)
        elif self.model_type == 'bilstm':
            return ModelFactory.create_bidirectional_lstm_model(input_shape, self.output_dim)
        elif self.model_type == 'gru':
            return ModelFactory.create_gru_model(input_shape, self.output_dim)
        elif self.model_type == 'cnn_lstm':
            return ModelFactory.create_cnn_lstm_model(input_shape, self.output_dim)
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
    
    def train(self, 
              X_train: np.ndarray, 
              y_train: np.ndarray, 
              X_val: np.ndarray, 
              y_val: np.ndarray,
              batch_size: int = 32,
              epochs: int = 100,
              patience: int = 10,
              model_path: str = "model",
              verbose: int = 1) -> Dict:
        """
        Train the model
        
        Args:
            X_train: Training data
            y_train: Training labels
            X_val: Validation data
            y_val: Validation labels
            batch_size: Batch size
            epochs: Maximum number of epochs
            patience: Patience for early stopping
            model_path: Path to save the model
            verbose: Verbosity level
            
        Returns:
            Training history
        """
        # Create callbacks
        callbacks = [
            EarlyStopping(monitor='val_loss', patience=patience, restore_best_weights=True),
            ReduceLROnPlateau(monitor='val_loss', factor=0.2, patience=patience//2, min_lr=1e-6),
        ]
        
        # Add model checkpoint if model_path is provided
        if model_path:
            os.makedirs(os.path.dirname(model_path), exist_ok=True)
            callbacks.append(ModelCheckpoint(model_path, monitor='val_loss', save_best_only=True))
        
        # Train the model
        self.history = self.model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            batch_size=batch_size,
            epochs=epochs,
            callbacks=callbacks,
            verbose=verbose
        )
        
        return self.history.history
    
    def predict(self, X: np.ndarray) -> np.ndarray:
        """
        Make predictions
        
        Args:
            X: Input data
            
        Returns:
            Predictions
        """
        return self.model.predict(X)
    
    def predict_sequence(self, initial_sequence: np.ndarray, n_steps: int, 
                          scaler_y, scaler_X=None, position_indices: Optional[Tuple[int, int]] = None) -> np.ndarray:
        """
        Make autoregressive predictions
        
        Args:
            initial_sequence: Initial sequence to start prediction from
            n_steps: Number of steps to predict
            scaler_y: Scaler for output data
            scaler_X: Scaler for input data (needed if predicting velocity)
            position_indices: Indices of position features in the input (pos_x, pos_y)
            
        Returns:
            Sequence of predictions
        """
        if len(initial_sequence.shape) == 2:
            initial_sequence = np.expand_dims(initial_sequence, axis=0)
            
        # Make a copy to avoid modifying the original
        sequence = initial_sequence.copy()
        
        predictions = []
        
        # If position indices not provided, assume the last two features are position
        if position_indices is None:
            # Check if the input features include position
            if self.n_features >= 9:  # At least IMU(7) + position(2)
                position_indices = (self.n_features - 2, self.n_features - 1)
            else:
                position_indices = None
        
        for _ in range(n_steps):
            # Predict next position
            next_pos = self.model.predict(sequence)
            
            # Store prediction
            predictions.append(next_pos[0])
            
            # Update sequence for next prediction (sliding window)
            # Remove the first timestep
            new_sequence = sequence[:, 1:, :].copy()
            
            # Create last timestep to append
            # This requires copying most recent values and updating position if applicable
            last_timestep = sequence[:, -1:, :].copy()
            
            # If we have position in the input, update it with the predicted position
            if position_indices is not None:
                # Scale the predicted position to match the input scale
                if scaler_X is not None:
                    # Create a dummy array with the predicted position
                    pos_update = np.zeros((1, self.n_features))
                    pos_update[0, position_indices[0]] = next_pos[0, 0]  # pos_x
                    pos_update[0, position_indices[1]] = next_pos[0, 1]  # pos_y
                    
                    # Scale the position
                    pos_update_scaled = scaler_X.transform(pos_update)
                    
                    # Update the position in the last timestep
                    last_timestep[0, 0, position_indices[0]] = pos_update_scaled[0, position_indices[0]]  # pos_x
                    last_timestep[0, 0, position_indices[1]] = pos_update_scaled[0, position_indices[1]]  # pos_y
                else:
                    # If no scaler provided, assume the scales match
                    last_timestep[0, 0, position_indices[0]] = next_pos[0, 0]  # pos_x
                    last_timestep[0, 0, position_indices[1]] = next_pos[0, 1]  # pos_y
            
            # Append the updated last timestep
            sequence = np.concatenate([new_sequence, last_timestep], axis=1)
        
        # Convert predictions to original scale
        return scaler_y.inverse_transform(np.array(predictions))
    
    def evaluate(self, X_test: np.ndarray, y_test: np.ndarray) -> Dict:
        """
        Evaluate the model
        
        Args:
            X_test: Test data
            y_test: Test labels
            
        Returns:
            Evaluation metrics
        """
        metrics = self.model.evaluate(X_test, y_test, verbose=0)
        return {name: value for name, value in zip(self.model.metrics_names, metrics)}
    
    def save(self, filepath: str) -> None:
        """
        Save the model
        
        Args:
            filepath: Path to save the model
        """
        self.model.save(filepath)
    
    @classmethod
    def load(cls, filepath: str) -> 'VesselPositionPredictor':
        """
        Load a saved model
        
        Args:
            filepath: Path to load the model from
            
        Returns:
            VesselPositionPredictor instance
        """
        model = tf.keras.models.load_model(filepath)
        
        instance = cls()
        instance.model = model
        instance.sequence_length = model.input_shape[1]
        instance.n_features = model.input_shape[2]
        instance.output_dim = model.output_shape[1]
        
        return instance 