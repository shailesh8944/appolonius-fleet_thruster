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
echo "Rendering documentation to HTML..."
quarto render

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Documentation built successfully!"
    echo "View the documentation by opening _book/index.html in your browser"
    echo "or by running ./view_docs.sh"
else
    echo "Error building documentation."
    exit 1
fi 
