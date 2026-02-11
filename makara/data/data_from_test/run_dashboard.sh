#!/bin/bash

# Make the script executable
chmod +x run_dashboard.sh

# Check if Python is installed
if command -v python3 &>/dev/null; then
    echo "Python found, proceeding..."
else
    echo "Python not found. Please install Python 3.x"
    exit 1
fi

# Install required packages if they are not already installed
echo "Installing required packages..."
pip install -r requirements.txt

# Run the Streamlit app
echo "Starting the Waypoint Tracking Analysis Dashboard..."
streamlit run app.py 