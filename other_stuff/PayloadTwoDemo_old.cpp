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
    base::StateSpacePtr stateSpace = setup.getStateSpace();
    unsigned int droneCount = setup.getRobotCount();

    // Set bounds for payload position
    base::RealVectorBounds bounds(3);
    bounds.setLow(-300);
    bounds.setHigh(600);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(bounds);

    // Define start and goal states
    base::ScopedState<base::CompoundStateSpace> startState(stateSpace), goalState(stateSpace);

    // Payload start state
    auto *payloadStart = startState->as<base::SE3StateSpace::StateType>(0);
    payloadStart->setX(125.0); payloadStart->setY(125.0); payloadStart->setZ(-150.0);
    payloadStart->rotation().setIdentity();

    // Payload goal state
    auto *payloadGoal = goalState->as<base::SE3StateSpace::StateType>(0);
    payloadGoal->setX(375.0); payloadGoal->setY(375.0); payloadGoal->setZ(-150.0);
    payloadGoal->rotation().setIdentity();

    // Drones and cables: set rotations to identity
    for (unsigned int i = 0; i < droneCount; ++i)
    {
        // Drone quaternion
        startState->as<base::SO3StateSpace::StateType>(1 + i * 2)->setIdentity();
        goalState->as<base::SO3StateSpace::StateType>(1 + i * 2)->setIdentity();

        // Cable quaternion
        startState->as<base::SO3StateSpace::StateType>(2 + i * 2)->setIdentity();
        goalState->as<base::SO3StateSpace::StateType>(2 + i * 2)->setIdentity();
    }

    // Set the start and goal states
    setup.setStartAndGoalStates(startState, goalState, 0.5);
}




void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // Force state space and space information setup
    auto stateSpace = setup.getStateSpace();
    setup.getStateSpace()->setup();
    setup.getSpaceInformation()->setup();

    // Debug: Print state space structure
    auto *compoundSpace = stateSpace->as<ompl::base::CompoundStateSpace>();
    std::cout << "State Space Structure:\n";
    for (unsigned int i = 0; i < compoundSpace->getSubspaceCount(); ++i)
    {
        std::cout << "Subspace " << i << " - Dimension: "
                  << compoundSpace->getSubspace(i)->getDimension() << std::endl;
    }

    // Set up manual state validity checker (placeholder, assumes all states are valid)
    setup.getSpaceInformation()->setStateValidityChecker(
        [](const ompl::base::State *state) {
            // Replace with your validity logic
            return true; // Assume all states are valid for debugging
        });

    setup.getSpaceInformation()->setup();

    // Set up the planner
    auto planner = std::make_shared<ompl::control::RRT>(setup.getSpaceInformation());
    planner->setGoalBias(0.05);
    setup.setPlanner(planner);

    // Solve the planning problem
    if (setup.solve(20)) // Time limit of 20 seconds
    {
        control::PathControl &path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
        else
        {
            std::cout << "Exact solution found!\n";
        }
    }
    else
    {
        std::cout << "No solution found within the time limit.\n";
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

    // system("cd .. && python3 plot_trajectories.py");
}
