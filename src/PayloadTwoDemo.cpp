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
#include <ompl/control/planners/kpiece/KPIECE1.h>
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

    // Set zero initial derivatives for payload velocities
    auto *payloadVelocityStart = startState->as<base::RealVectorStateSpace::StateType>(1 + droneCount * 2);
    auto *payloadVelocityGoal = goalState->as<base::RealVectorStateSpace::StateType>(1 + droneCount * 2);
    for (unsigned int i = 0; i < 3; ++i)
    {
        payloadVelocityStart->values[i] = 0.0;
        payloadVelocityGoal->values[i] = 0.0;
    }

    // Set zero initial derivatives for payload quaternion derivatives
    auto *payloadQuatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(2 + droneCount * 2);
    auto *payloadQuatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(2 + droneCount * 2);
    for (unsigned int i = 0; i < 4; ++i)
    {
        payloadQuatDerivStart->values[i] = 0.0;
        payloadQuatDerivGoal->values[i] = 0.0;
    }

    // Drones and cables: set rotations to identity and quaternion derivatives to zero
    for (unsigned int i = 0; i < droneCount; ++i)
    {
        // Drone quaternion
        startState->as<base::SO3StateSpace::StateType>(1 + i * 2)->setIdentity();
        goalState->as<base::SO3StateSpace::StateType>(1 + i * 2)->setIdentity();

        // Cable quaternion
        startState->as<base::SO3StateSpace::StateType>(2 + i * 2)->setIdentity();
        goalState->as<base::SO3StateSpace::StateType>(2 + i * 2)->setIdentity();

        // Drone quaternion derivatives
        auto *droneQuatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(3 + droneCount * 2 + i * 2);
        auto *droneQuatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(3 + droneCount * 2 + i * 2);
        for (unsigned int j = 0; j < 4; ++j)
        {
            droneQuatDerivStart->values[j] = 0.0;
            droneQuatDerivGoal->values[j] = 0.0;
        }

        // Cable quaternion derivatives
        auto *cableQuatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(4 + droneCount * 2 + i * 2);
        auto *cableQuatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(4 + droneCount * 2 + i * 2);
        for (unsigned int j = 0; j < 4; ++j)
        {
            cableQuatDerivStart->values[j] = 0.0;
            cableQuatDerivGoal->values[j] = 0.0;
        }
    }


    // Set up manual state validity checker (placeholder, assumes all states are valid)
    setup.setStateValidityChecker([](const ompl::base::State *state) -> bool {
        // Basic bounds check (example; customize as needed)
        const auto *compoundState = state->as<ompl::base::CompoundState>();
        if (!compoundState)
            return false;

        const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
        if (payloadState->getX() < -300 || payloadState->getX() > 600)
            return false; // Example bounds check for X-coordinate

        // Add additional checks for payload, drones, etc.
        return true;
    });

    // Set the start and goal states
    setup.setStartAndGoalStates(startState, goalState, 0.5);

    // // Print start and goal states
    // std::cout << "Start state: ";
    // startState.print(std::cout);
    // std::cout << "Goal state: ";
    // goalState.print(std::cout);
}





void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // Force state space and space information setup
    auto stateSpace = setup.getStateSpace();
    setup.getStateSpace()->setup();
    setup.getSpaceInformation()->setup();

    // Set up the planner
    auto planner = std::make_shared<ompl::control::RRT>(setup.getSpaceInformation());
    planner->setGoalBias(0.05);
    setup.setPlanner(planner);

    // Validate planner setup
    if (!setup.getPlanner())
    {
        std::cerr << "Error: Planner is not initialized.\n";
        return;
    }

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
        originalPropagator->propagate(from, control, duration, to);

        const auto *compoundState = to->as<ompl::base::CompoundState>();
        if (!compoundState)
        {
            std::cerr << "Error: Propagated state is invalid.\n";
            return;
        }

        std::ostringstream stateLine;
        stateLine << "Propagated State: ";
        for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
        {
            const auto *droneState = compoundState->as<ompl::base::SE3StateSpace::StateType>(d * 2);
            stateLine << droneState->getX() << " " << droneState->getY() << " " << droneState->getZ() << " ";
        }

        const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(setup.getRobotCount() * 2);
        stateLine << payloadState->getX() << " " << payloadState->getY() << " " << payloadState->getZ() << " ";

        outFile << stateLine.str() << "\n";
        std::cout << stateLine.str() << "\n";
        outFile.flush();
    });

    // Solve the planning problem
    auto terminationCondition = ompl::base::timedPlannerTerminationCondition(3.0); // 3 seconds
    std::cout << "Solving the problem...\n";

    try
    {
        if (setup.solve(terminationCondition))
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
    catch (const std::exception &e)
    {
        std::cerr << "Error during planning: " << e.what() << std::endl;
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
