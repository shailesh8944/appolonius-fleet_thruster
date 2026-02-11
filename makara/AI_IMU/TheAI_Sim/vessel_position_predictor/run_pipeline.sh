#!/bin/bash

# Create output directories
mkdir -p ./model_output
mkdir -p ./model_comparison
mkdir -p ./predictions

# Compare different models
echo "Comparing different model architectures..."
python3 compare_models.py --data_dir ../extracted_data_noNoise \
                        --model_types lstm gru cnn_lstm bilstm \
                        --sequence_length 10 \
                        --batch_size 32 \
                        --epochs 25 \
                        --output_dir ./model_comparison \
                        --autoregressive

# Train the best model with more epochs
echo "Training the best model (LSTM) with more epochs..."
python3 train.py --data_dir ../extracted_data_noNoise \
               --model_type lstm \
               --sequence_length 10 \
               --batch_size 32 \
               --epochs 100 \
               --output_dir ./model_output \
               --autoregressive

# Make predictions with the trained model
echo "Making predictions using the trained model..."
python3 predict.py --model_path ./model_output/lstm_*/model \
                 --data_dir ../extracted_data_noNoise \
                 --output_dir ./predictions \
                 --autoregressive

echo "Pipeline completed successfully!" 