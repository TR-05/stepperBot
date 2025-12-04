#!/bin/bash
# A script to cross-compile the C++ application statically for ARM and deploy it via SSH (scp).

# --- Configuration ---
SOURCE_FILE="main.cpp" # UPDATED: Now refers to the new main file
OUTPUT_DIR="build"
OUTPUT_FILE="t" # Set to 't' as requested
TARGET_HOST="10.0.0.53" # Set to 10.0.0.53 as requested
TARGET_USER="root"
TARGET_PATH="/root/$OUTPUT_FILE" # The final location and name on the BTT Pi

# --- Dependency Check and Installation ---
echo "--- Checking and installing required dependencies (64-bit ARM toolchain)..."

# Check for cmake
if ! command -v cmake &> /dev/null
then
    echo "cmake not found. Attempting to install cmake..."
    sudo apt update
    sudo apt install -y cmake
    if [ $? -ne 0 ]; then
        echo "FATAL: Failed to install cmake. Please check your internet connection or package manager."
        exit 1
    fi
fi

# Check for the **64-bit** ARM cross-compiler (aarch64)
if ! command -v aarch64-linux-gnu-g++ &> /dev/null
then
    echo "64-bit ARM cross-compiler (g++-aarch64-linux-gnu) not found. Attempting to install it..."
    sudo apt update
    # Install the 64-bit ARM cross-compiler package
    sudo apt install -y g++-aarch64-linux-gnu
    if [ $? -ne 0 ]; then
        echo "FATAL: Failed to install g++-aarch64-linux-gnu. Check repository access."
        exit 1
    fi
fi

# Check for scp (often part of ssh or openssh-client)
if ! command -v scp &> /dev/null
then
    echo "scp (OpenSSH client) not found. Attempting to install it..."
    sudo apt update
    sudo apt install -y openssh-client
    if [ $? -ne 0 ]; then
        echo "FATAL: Failed to install openssh-client."
        exit 1
    fi
fi

echo "All required dependencies are installed."
# --- End Dependency Check ---


# --- Step 1: Static Cross-Compile using CMake ---
echo "--- 1. Starting static cross-compilation for 64-bit ARM using CMake..."

# 1a. Clean up previous build directory
rm -rf $OUTPUT_DIR

# 1b. Create and enter build directory
mkdir $OUTPUT_DIR
cd $OUTPUT_DIR

# 1c. Run CMake, explicitly specifying the **64-bit** ARM cross-compiler path.
echo "Running cmake configuration with 64-bit compiler..."
cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++

if [ $? -ne 0 ]; then
    echo "CMake configuration failed. Aborting deployment."
    cd ..
    exit 1
fi

# 1d. Run Make to compile the target executable ('t' in this case)
echo "Running make compilation..."
make $OUTPUT_FILE

if [ $? -ne 0 ]; then
    echo "Compilation failed. Aborting deployment."
    cd ..
    exit 1
fi

# Go back to the root directory for SCP
cd ..

# --- Step 2: Transfer Executable via SCP ---
# The compiled executable is located inside the build directory.
DEPLOY_FILE="$OUTPUT_DIR/$OUTPUT_FILE"

echo "--- 2. Deploying $DEPLOY_FILE to $TARGET_USER@$TARGET_HOST:$TARGET_PATH..."

# NOTE: This scp command will automatically use SSH keys if they are set up 
# (via ssh-keygen and ssh-copy-id) for passwordless deployment.
scp $DEPLOY_FILE $TARGET_USER@$TARGET_HOST:$TARGET_PATH

# Check the exit status of scp
if [ $? -eq 0 ]; then
    echo "Deployment successful (Passwordless if keys are configured)."
    echo "Next steps on BTT Pi: Execute remotely via SSH."
    echo "ssh $TARGET_USER@$TARGET_HOST 'chmod +x $TARGET_PATH && $TARGET_PATH'"
else
    echo "Deployment failed. Check IP, SSH keys, or manually enter the password when prompted."
    exit 1
fi

echo "--- Automation complete. ---"

# Cleanup: remove the build directory if deployment was successful
# rm -rf $OUTPUT_DIR