# Multi-Drone Payload Motion Planning with OMPL

## Overview
This repository provides a motion planning setup using **OMPL (Open Motion Planning Library)** to coordinate multiple drones carrying a payload suspended by cables. The primary planner implemented is the kinodynamic RRT for a system consisting of 4 drones.

## Prerequisites

### Libraries
- [**OMPL**](https://ompl.kavrakilab.org/) (Tested with [OMPL 1.6.0](https://github.com/ompl/ompl/releases/tag/1.6.0))
- [**Boost**](https://www.boost.org/) (numeric/[odeint](https://www.boost.org/doc/libs/release/libs/numeric/odeint/) and [filesystem](https://www.boost.org/doc/libs/release/libs/filesystem/) components)
- [**Eigen3**](https://eigen.tuxfamily.org/)

### Optional Python dependencies for visualization:
- Python 3
- Matplotlib (`pip install matplotlib`)
- Numpy (`pip install numpy`)

## Build and Run Instructions

Clone this repository and navigate to its root:
```bash
git clone https://github.com/dolevfr/OMPL_DRONES.git
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