#!/usr/bin/env python

import sys
from os.path import abspath, dirname, join

from ompl import base as ob
from ompl import app as oa
from ompl.geometric import RRTstar

# plan in SE(3)
setup = oa.SE3RigidBodyPlanning()

# load the robot and the environment
setup.setRobotMesh('../meshes/geometric_collision.dae')
setup.setEnvironmentMesh('../meshes/Apartment_env.dae')

# tweak the SpaceInformation for finer resolution
si = setup.getSpaceInformation()
si.setStateValidityCheckingResolution(0.01)      # 0.5% resolution for collision checks

# set up RRT* with small extension range
planner = RRTstar(si)
planner.setGoalBias(0.05)
planner.setRewireFactor(1.10)
planner.setPruneThreshold(0.05)
planner.setRange(0.0)
setup.setPlanner(planner)

# define start state
start = ob.State(si)
start().setX(-10)
start().setY(-40)
start().setZ(20)
start().rotation().setIdentity()

goal = ob.State(si)
goal().setX(60)
goal().setY(-15)
goal().setZ(15)
goal().rotation().setIdentity()

# set the start & goal states
setup.setStartAndGoalStates(start, goal)

# do setup (for debugging/info)
setup.setup()
print(setup)

# solve for up to 10 seconds
if setup.solve(10):
    setup.simplifySolution(1.0)  # optional smoothing for 1 second
    path = setup.getSolutionPath()
    path.interpolate(1000)        # densify to 200 waypoints post-solution
    print(path.printAsMatrix())
    print("Path valid?", path.check())
else:
    print("No solution found within time limit.")

# Save the path to a file
with open("geometric_path.txt", "w") as f:
    f.write(path.printAsMatrix())
print("Solution saved to geometric_path.txt")


import matplotlib.pyplot as plt
import numpy as np

if setup.haveSolutionPath():
    path = setup.getSolutionPath()
    n_states = path.getStateCount()
    xyz = np.zeros((n_states, 3))
    for i in range(n_states):
        s = path.getState(i)
        xyz[i, 0] = s.getX()
        xyz[i, 1] = s.getY()
        xyz[i, 2] = s.getZ()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], c='b', s=10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Planned Path Points')
    plt.show()


