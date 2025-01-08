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

    auto stateSpace = setup.getStateSpace();
    auto *compoundSpace = stateSpace->as<base::CompoundStateSpace>();
    unsigned int droneCount = setup.getRobotCount();

    // setup.inferEnvironmentBounds();

    // Set default validity checker to accept all states
    setup.getSpaceInformation()->setStateValidityChecker([](const ompl::base::State * /*state*/) -> bool {
        return true;
    });

    // base::RealVectorBounds bounds(3);
    // bounds.setLow(-300);
    // bounds.setHigh(600);

    // // Apply bounds to the payload position (first subspace)
    // stateSpace->as<ompl::base::CompoundStateSpace>()
    //         ->getSubspace(0) // First subspace is payload position
    //         ->as<ompl::base::RealVectorStateSpace>()
    //         ->setBounds(bounds);

    // Ensure SpaceInformation is fully configured
    setup.getSpaceInformation()->setup();

    // Define start and goal states
    base::ScopedState<base::CompoundStateSpace> startState(stateSpace), goalState(stateSpace);

    // Payload position and orientation (SE3StateSpace)
    auto *payloadStart = startState->as<base::SE3StateSpace::StateType>(0);
    auto *payloadGoal = goalState->as<base::SE3StateSpace::StateType>(0);

    // Set payload start position
    payloadStart->setX(125.0);
    payloadStart->setY(125.0);
    payloadStart->setZ(-150.0);
    payloadStart->rotation().setIdentity(); // Quaternion: [0 0 0 1]

    // Set payload goal position
    payloadGoal->setX(375.0);
    payloadGoal->setY(375.0);
    payloadGoal->setZ(-150.0);
    payloadGoal->rotation().setIdentity(); // Quaternion: [0 0 0 1]

    unsigned int index = 1; // Start after payload (SE3)

    // Set quaternions for drones and cables to [0 0 0 1]
    for (unsigned int i = 0; i < 2 * droneCount; ++i)
    {
        auto *quatStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *quatGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        quatStart->values[0] = quatGoal->values[0] = 1.0;
        quatStart->values[1] = quatGoal->values[1] = 0.0;
        quatStart->values[2] = quatGoal->values[2] = 0.0;
        quatStart->values[3] = quatGoal->values[3] = 0.0;
        ++index;
    }

    // Set payload velocity to [0 0 0]
    auto *payloadVelocityStart = startState->as<base::RealVectorStateSpace::StateType>(index);
    auto *payloadVelocityGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
    for (unsigned int i = 0; i < 3; ++i)
        payloadVelocityStart->values[i] = payloadVelocityGoal->values[i] = 0.0;
    ++index;

    // Set quaternion derivatives (payload, drones, cables) to [0 0 0 0]
    for (unsigned int i = 0; i < 1 + 2 * droneCount; ++i)
    {
        auto *quatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *quatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        for (unsigned int j = 0; j < 4; ++j)
            quatDerivStart->values[j] = quatDerivGoal->values[j] = 0.0;
        ++index;
    }


    // Set the start and goal states
    setup.setStartAndGoalStates(startState, goalState, 0.5);

    std::cout << "Start state: ";
    startState.print(std::cout);
    std::cout << "Goal state: ";
    goalState.print(std::cout);
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

        // Payload position and quaternion
        const auto *payloadSE3 = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
        stateLine << payloadSE3->getX() << " " << payloadSE3->getY() << " " << payloadSE3->getZ() << " ";
        stateLine << payloadSE3->rotation().x << " " << payloadSE3->rotation().y << " "
                << payloadSE3->rotation().z << " " << payloadSE3->rotation().w << " ";

        unsigned int index = 1; // Start after payload (SE3)

        // Drone and cable quaternions
        for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
        {
            const auto *droneQuat = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            const auto *cableQuat = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << droneQuat->values[0] << " " << droneQuat->values[1] << " "
                    << droneQuat->values[2] << " " << droneQuat->values[3] << " ";
            stateLine << cableQuat->values[0] << " " << cableQuat->values[1] << " "
                    << cableQuat->values[2] << " " << cableQuat->values[3] << " ";
        }

        // Payload velocity and quaternion derivatives
        const auto *payloadVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
        const auto *payloadQuatDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
        stateLine << payloadVelocity->values[0] << " " << payloadVelocity->values[1] << " " << payloadVelocity->values[2] << " ";
        stateLine << payloadQuatDeriv->values[0] << " " << payloadQuatDeriv->values[1] << " "
                << payloadQuatDeriv->values[2] << " " << payloadQuatDeriv->values[3] << " ";

        // Drone and cable quaternion derivatives
        for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
        {
            const auto *droneQuatDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            const auto *cableQuatDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << droneQuatDeriv->values[0] << " " << droneQuatDeriv->values[1] << " "
                    << droneQuatDeriv->values[2] << " " << droneQuatDeriv->values[3] << " ";
            stateLine << cableQuatDeriv->values[0] << " " << cableQuatDeriv->values[1] << " "
                    << cableQuatDeriv->values[2] << " " << cableQuatDeriv->values[3] << " ";
        }

        // Log state to file
        outFile << stateLine.str() << "\n";
        outFile.flush();
    });



    // // Solve the planning problem
    // auto terminationCondition = ompl::base::timedPlannerTerminationCondition(3.0); // 3 seconds
    // std::cout << "Solving the problem...\n";

    try
    {
        if (setup.solve(1))
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
