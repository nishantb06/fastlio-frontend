#!/bin/bash

# Create a temporary directory for packaging
mkdir -p package

# Install dependencies to the package directory
pip install --target ./package -r requirements.txt

# Copy your application files
cp main.py lambda_function.py ./package/

# Create deployment package
cd package
zip -r ../deployment.zip .
cd ..

# Clean up
rm -rf package 