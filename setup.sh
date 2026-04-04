#!/bin/bash

# THE TRANSMUTER - Environment Provisioning Script
# This script prepares a fresh Ubuntu machine for 5-axis Kinematic development.

echo "--- Initializing The Transmuter Environment ---"

# 1. Update Package Lists
sudo apt update

# 2. Install Build Essentials & C++23 Capable Compiler
# Note: g++-15 or newer is preferred for full C++23 support
sudo apt install -y build-essential git pkg-config

# 3. Install Math & Kinematics (Eigen)
sudo apt install -y libeigen3-dev

# 4. Install Graphics Suite (OpenGL, GLU, GLEW, GLFW)
sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libglfw3-dev mesa-utils

echo "--- Setup Complete. ---"
