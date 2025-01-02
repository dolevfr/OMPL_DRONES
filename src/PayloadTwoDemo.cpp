/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Mark Moll (adapted for 4 drones) */

#include <ompl/control/planners/rrt/RRT.h>
#include <omplapp/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "PayloadTwoDrones.h"

using namespace ompl;

void payloadSystemSetup(app::PayloadSystem &setup)
{
    // Retrieve the state space
    base::StateSpacePtr stateSpace = setup.getStateSpace();

    // Define bounds for SE3 subspaces (Payload and Drones)
    base::RealVectorBounds bounds(3);
    bounds.setLow(-200);
    bounds.setHigh(500);

    // Set bounds for each drone and the payload
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(i * 2)->setBounds(bounds); // Drone bounds
    }
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(setup.getRobotCount() * 2)->setBounds(bounds); // Payload bounds

    // Create the start state
    base::ScopedState<> start(stateSpace);

    // Set the payload start position and orientation
    auto *payloadStart = start->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(setup.getRobotCount() * 2);
    payloadStart->setX(125);  // X-coordinate
    payloadStart->setY(125);  // Y-coordinate
    payloadStart->setZ(-150); // Z-coordinate
    payloadStart->rotation().setIdentity(); // Set the payload's initial rotation (trivial quaternion)

    // Set drones' start positions above payload corners
    double cableLength = setup.getCableLength(); // Cable length
    for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    {
        // Get the corner position in the payload's local frame
        Eigen::Vector3d cornerLocal = setup.getPayloadCorner(d);

        // Transform the corner to the world frame (trivial rotation for payload)
        Eigen::Vector3d cornerWorld = Eigen::Vector3d(125, 125, -150) + cornerLocal;

        // Drone position is directly above the corner at a distance of `cableLength`
        Eigen::Vector3d droneStartPos = cornerWorld + Eigen::Vector3d(0, 0, cableLength);

        // Set the drone's SE3 start position
        auto *droneStart = start->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(d * 2);
        droneStart->setX(droneStartPos.x());
        droneStart->setY(droneStartPos.y());
        droneStart->setZ(droneStartPos.z());
        droneStart->rotation().setIdentity(); // Drones start with trivial quaternion

        // Set the drone's velocity to zero
        auto *velocityStart = start->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(d * 2 + 1);
        for (unsigned int j = 0; j < 6; ++j)
        {
            velocityStart->values[j] = 0.0; // Initialize all velocity components to 0
        }
    }

    // Set trivial start velocities for the payload
    auto *payloadVelocityStart = start->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(setup.getRobotCount() * 2 + 1);
    for (unsigned int j = 0; j < 6; ++j)
    {
        payloadVelocityStart->values[j] = 0.0; // Initialize all payload velocity components to 0
    }

    // Create the goal state
    base::ScopedState<> goal(stateSpace);

    // Set the payload goal position and orientation (only X, Y are changed)
    auto *payloadGoal = goal->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(setup.getRobotCount() * 2);
    payloadGoal->setX(375);  // X-coordinate
    payloadGoal->setY(375);  // Y-coordinate
    payloadGoal->setZ(-150); // Z-coordinate (same as start)
    payloadGoal->rotation().setIdentity(); // Same trivial quaternion as the start state

    // Set drones' goal positions above payload corners
    for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    {
        // Get the corner position in the payload's local frame
        Eigen::Vector3d cornerLocal = setup.getPayloadCorner(d);

        // Transform the corner to the world frame for the goal position
        Eigen::Vector3d cornerWorld = Eigen::Vector3d(375, 375, -150) + cornerLocal;

        // Drone position is directly above the corner at a distance of `cableLength`
        Eigen::Vector3d droneGoalPos = cornerWorld + Eigen::Vector3d(0, 0, cableLength);

        // Set the drone's SE3 goal position
        auto *droneGoal = goal->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(d * 2);
        droneGoal->setX(droneGoalPos.x());
        droneGoal->setY(droneGoalPos.y());
        droneGoal->setZ(droneGoalPos.z());
        droneGoal->rotation().setIdentity(); // Same trivial quaternion as the start state

        // Set the drone's velocity to zero
        auto *velocityGoal = goal->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(d * 2 + 1);
        for (unsigned int j = 0; j < 6; ++j)
        {
            velocityGoal->values[j] = 0.0; // Initialize all velocity components to 0
        }
    }

    // Set trivial goal velocities for the payload
    auto *payloadVelocityGoal = goal->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(setup.getRobotCount() * 2 + 1);
    for (unsigned int j = 0; j < 6; ++j)
    {
        payloadVelocityGoal->values[j] = 0.0; // Initialize all payload velocity components to 0
    }

    // Convert start and goal to full states
    auto fullStart = setup.getFullStateFromGeometricComponent(start);
    auto fullGoal = setup.getFullStateFromGeometricComponent(goal);

    setup.setStateValidityChecker([&setup](const ompl::base::State *state) -> bool {
        // Perform your validity checks here (e.g., bounds, collisions, etc.)
        return true;
    });


    // Check if the start and goal states are valid
    if (!setup.getSpaceInformation()->isValid(fullStart.get()))
    {
        std::cerr << "Error: Start state is invalid.\n";
        return;
    }
    if (!setup.getSpaceInformation()->isValid(fullGoal.get()))
    {
        std::cerr << "Error: Goal state is invalid.\n";
        return;
    }
    // Set start and goal states in the planner
    setup.setStartAndGoalStates(fullStart, fullGoal, 0.5);
}


void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // Set up the planner
    auto planner = std::make_shared<control::RRT>(setup.getSpaceInformation());
    planner->setGoalBias(0.05); // Example: Adjust goal bias
    setup.setPlanner(planner);

    // Open file for writing and ensure it works
    std::ofstream outFile("solution_path.txt", std::ios::out | std::ios::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Unable to open file for writing solution path.\n";
        return;
    }

    // Wrap the state propagator to log every propagated state
    auto originalPropagator = setup.getSpaceInformation()->getStatePropagator();
    setup.getSpaceInformation()->setStatePropagator([&setup, originalPropagator, &outFile](
                                                        const ompl::base::State *from,
                                                        const ompl::control::Control *control,
                                                        double duration,
                                                        ompl::base::State *to) {
        // Perform state propagation
        originalPropagator->propagate(from, control, duration, to);

        // Log the propagated state
        const auto *compoundState = to->as<ompl::base::CompoundState>();
        if (!compoundState)
        {
            std::cerr << "Error: Propagated state is invalid.\n";
            return;
        }

        std::ostringstream stateLine;
        for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
        {
            const auto *droneState = compoundState->as<ompl::base::SE3StateSpace::StateType>(d * 2);

            // Log drone position and quaternion
            stateLine << droneState->getX() << " " << droneState->getY() << " " << droneState->getZ() << " ";
            stateLine << droneState->rotation().x << " " << droneState->rotation().y << " "
                      << droneState->rotation().z << " " << droneState->rotation().w << " ";
        }

        const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(setup.getRobotCount() * 2);

        // Log payload position and quaternion
        stateLine << payloadState->getX() << " " << payloadState->getY() << " " << payloadState->getZ() << " ";
        stateLine << payloadState->rotation().x << " " << payloadState->rotation().y << " "
                  << payloadState->rotation().z << " " << payloadState->rotation().w << " ";

        outFile << stateLine.str() << "\n";
        outFile.flush(); // Ensure the data is written immediately
    });

    // Solve the planning problem
    if (setup.solve(3))
    {
        std::cout << "Planning completed successfully.\n";
        control::PathControl &path(setup.getSolutionPath());
        path.printAsMatrix(std::cout); // Optionally print the solution matrix

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found within the time limit.\n";
    }

    // Ensure the file is properly closed
    outFile.close();
    if (!outFile)
    {
        std::cerr << "Error: Failed to write to solution_path.txt.\n";
    }
    else
    {
        std::cout << "All propagated states saved to solution_path.txt.\n";
    }
}











void saveSolutionPath(const ompl::control::PathControl &path, int numDrones, const std::string &filename = "solution_path.txt")
{
    if (path.getStateCount() == 0)
    {
        std::cerr << "Error: Solution path is empty.\n";
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile)
    {
        std::cerr << "Error: Unable to open file for writing solution path.\n";
        return;
    }

    // Write a header (optional)
    outFile << "# Solution Path for " << numDrones << " drones\n";
    outFile << "# Format: x y z qx qy qz qw (for each drone, repeated per state)\n";

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const ompl::base::State *state = path.getState(i);
        const auto *compoundState = state->as<ompl::base::CompoundState>();

        if (!compoundState)
        {
            std::cerr << "Error: State is not a CompoundState at index " << i << ".\n";
            continue;
        }

        for (int d = 0; d < numDrones; ++d)
        {
            // Access the SE3 state for each drone
            const auto *droneState = compoundState->as<ompl::base::SE3StateSpace::StateType>(d * 2);
            if (!droneState)
            {
                std::cerr << "Error: Drone state is null for drone " << d << " at state " << i << ".\n";
                continue;
            }

            // Extract position and orientation
            outFile << droneState->getX() << " " << droneState->getY() << " " << droneState->getZ() << " ";
            outFile << droneState->rotation().x << " " << droneState->rotation().y << " "
                    << droneState->rotation().z << " " << droneState->rotation().w << " ";
        }
        outFile << "\n"; // Newline for each state
    }

    outFile.close();
    std::cout << "Solution path saved to " << filename << ".\n";
}


int main(int argc, char ** /*unused*/)
{
    // Create MultiDronePlanning instance
    app::PayloadSystem multiDrone;

    // Setup MultiDrone planning environment
    payloadSystemSetup(multiDrone);

    // Run MultiDrone planning demo
    payloadSystemDemo(multiDrone);

    

    // Save solution path to file
    // saveSolutionPath(multiDrone.getSolutionPath(), multiDrone.getRobotCount());

    system("python3 /home/dolev/Desktop/Research/OMPL_drones/src/python/plot_trajectories.py");
}
