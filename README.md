# OMPL Drones – Multi-Drone Payload Motion Planning

## Overview
This repository provides a motion planning setup using **OMPL (Open Motion Planning Library)** to coordinate multiple drones carrying a payload suspended by cables. The primary planner implemented is the kinodynamic RRT for a system consisting of 4 drones.

## Project Structure
```
OMPL_drones/
├── build               # (generated) Build directory
├── include             # Header files
├── src                 # Source code files
├── other_stuff         # Additional resources
├── CMakeLists.txt      # CMake configuration file
└── README.md           # Project description and instructions
```

**Note:** The `build` directory is not included when cloning; you must create it manually.

## Prerequisites

### Libraries
- [**OMPL**](https://ompl.kavrakilab.org/) (Tested with [OMPL 1.6.0](https://github.com/ompl/ompl/releases/tag/1.6.0))
- [**Boost**](https://www.boost.org/) (numeric/[odeint](https://www.boost.org/doc/libs/release/libs/numeric/odeint/) and [filesystem](https://www.boost.org/doc/libs/release/libs/filesystem/) components)
- [**Eigen3**](https://eigen.tuxfamily.org/)

### Install Dependencies on Ubuntu
```bash
sudo apt-get update
sudo apt-get install libompl-dev libboost-all-dev libeigen3-dev cmake build-essential
```

### Optional Python dependencies for visualization:
- Python 3
- Matplotlib (`pip install matplotlib`)
- Numpy (`pip install numpy`)

## Build and Run Instructions

Clone this repository and navigate to its root:
```bash
git clone <your_repository_url>
cd OMPL_drones
```

Create and navigate to the `build` directory:
```bash
mkdir build
cd build
```

Configure the build environment with CMake:
```bash
cmake ..
```

Compile the project:
```bash
make
```

Run the executable (`PayloadFourDrones`):
```bash
./PayloadFourDrones
```

## Usage
Upon execution, the planner will:

- Plan a trajectory for a payload carried by 4 drones.
- Output the solution path to `solution_path.txt`.
- Visualize the trajectory using a Python visualization script.

### Customization
- You can modify start and goal states, drone count, payload parameters, and other system configurations directly in the source files (`PayloadFourDrones.h`, `PayloadFourDrones.cpp`, `PayloadFourDemo.cpp`).

## Visualization
The trajectory visualization script is located in the `src/python` directory and is automatically executed at runtime. Ensure Python dependencies are installed.

**Visualization script execution:**
```bash
python3 ../src/python/plot_trajectories_3D.py
```

## Authors
- Adapted for multi-drone system by **Mark Moll** (Original implementation: Rice University).

## License
This project is distributed under the **Rice University Software Distribution License**.