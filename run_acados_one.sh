#!/bin/bash
# set -e

# # 0. Set environment variables (do this before running model_payload.py)
# export ACADOS_SOURCE_DIR=$HOME/acados-install
# export ACADOS_INSTALL_DIR=$HOME/acados-install
# export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH

# Check if acados is installed
if [ ! -d "$ACADOS_INSTALL_DIR" ]; then
    echo "Error: Acados is not installed in $ACADOS_INSTALL_DIR"
    exit 1
fi

# Remove c_generated_code and build folders entirely
# rm -rf c_generated_code build

# 1. Generate acados solver code
echo ">>> Running model_payload.py..."
python3 src/python/model_payload.py

# Verify c_generated_code directory exists
if [ ! -d "c_generated_code" ]; then
    echo "Error: c_generated_code directory not found."
    exit 1
fi

# 2. Create build directory if it doesn't exist
# mkdir -p build
cd build

# # 3. Configure the build with cmake
# echo ">>> Running CMake..."
# cmake .. -Dacados_DIR=$HOME/acados-install/cmake

# 4. Build the project
echo ">>> Building..."
make -j

# 5. Run the executable
echo ">>> Running AcadosOneDrone..."
./AcadosOneDrone  # Full path to executable
