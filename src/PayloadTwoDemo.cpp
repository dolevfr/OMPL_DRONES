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
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // Define start state
    base::ScopedState<base::CompoundStateSpace> start(stateSpace);

    // Set payload position to (125, 125, -150)
    start->as<base::SE3StateSpace::StateType>(0)->setXYZ(125.0, 125.0, -150.0);

    // Set payload orientation (quaternion identity)
    start->as<base::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    // Set payload velocities to zero
    for (unsigned int i = 0; i < 6; ++i)
    {
        start->as<base::RealVectorStateSpace::StateType>(1)->values[i] = 0.0;
    }

    // Set drone states
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        unsigned int baseIndex = 2 + i * 3; // Drone SO3 starts at index 2 + i * 3

        // Set drone orientation (quaternion identity)
        start->as<base::SO3StateSpace::StateType>(baseIndex)->setIdentity();

        // Set drone velocities to zero
        for (unsigned int j = 0; j < 3; ++j)
        {
            start->as<base::RealVectorStateSpace::StateType>(baseIndex + 1)->values[j] = 0.0;
        }

        // Set cable angles and velocities to zero
        for (unsigned int j = 0; j < 4; ++j)
        {
            start->as<base::RealVectorStateSpace::StateType>(baseIndex + 2)->values[j] = 0.0;
        }
    }

    // Define goal state
    base::ScopedState<base::CompoundStateSpace> goal(stateSpace);

    // Set payload position to (375, 375, -150)
    goal->as<base::SE3StateSpace::StateType>(0)->setXYZ(375.0, 375.0, -150.0);

    // Set payload orientation (quaternion identity)
    goal->as<base::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    // Set payload velocities to zero
    for (unsigned int i = 0; i < 6; ++i)
    {
        goal->as<base::RealVectorStateSpace::StateType>(1)->values[i] = 0.0;
    }

    // Set drone states (similar to start state)
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        unsigned int baseIndex = 2 + i * 3;

        // Set drone orientation (quaternion identity)
        goal->as<base::SO3StateSpace::StateType>(baseIndex)->setIdentity();

        // Set drone velocities to zero
        for (unsigned int j = 0; j < 3; ++j)
        {
            goal->as<base::RealVectorStateSpace::StateType>(baseIndex + 1)->values[j] = 0.0;
        }

        // Set cable angles and velocities to zero
        for (unsigned int j = 0; j < 4; ++j)
        {
            goal->as<base::RealVectorStateSpace::StateType>(baseIndex + 2)->values[j] = 0.0;
        }
    }

    // Set start and goal states with a threshold of 0.5
    setup.setStartAndGoalStates(start, goal, 0.5);
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
    std::cout << "Writing propagated states to solution_path.txt...\n";
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

        // Log payload state
        const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
        stateLine << payloadState->getX() << " " << payloadState->getY() << " " << payloadState->getZ() << " ";
        stateLine << payloadState->rotation().x << " " << payloadState->rotation().y << " "
              << payloadState->rotation().z << " " << payloadState->rotation().w << " ";

        // Log payload velocities
        const auto *payloadVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
        for (unsigned int i = 0; i < 6; ++i)
        {
            stateLine << payloadVelocity->values[i] << " ";
        }

        // Log drone states
        for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
        {
            unsigned int baseIndex = 2 + d * 3;

            // Log drone orientation
            const auto *droneOrientation = compoundState->as<ompl::base::SO3StateSpace::StateType>(baseIndex);
            stateLine << droneOrientation->x << " " << droneOrientation->y << " "
                  << droneOrientation->z << " " << droneOrientation->w << " ";

            // Log drone velocities
            const auto *droneVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
            for (unsigned int i = 0; i < 3; ++i)
            {
            stateLine << droneVelocity->values[i] << " ";
            }

            // Log cable angles and velocities
            const auto *cableState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
            for (unsigned int i = 0; i < 4; ++i)
            {
            stateLine << cableState->values[i] << " ";
            }
        }

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
