#!/bin/bash

# Script to build the MAV Simulator documentation
echo "Building MAV Simulator documentation..."

# Check if Quarto is installed
if ! command -v quarto &> /dev/null; then
    echo "Error: Quarto is not installed. Please install it from https://quarto.org/docs/getting-started/installation.html"
    exit 1
fi

# Navigate to the docs directory (in case script is run from elsewhere)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Clean previous build
if [ -d "_book" ]; then
    echo "Cleaning previous build..."
    rm -rf ./docs
fi

# Build the documentation
echo "Previewing documentation..."
quarto preview --port 8500

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Documentation previewed successfully!"
    echo "View the documentation by opening http://localhost:8500 in your browser"
else
    echo "Error previewing documentation."
    exit 1
fi 
