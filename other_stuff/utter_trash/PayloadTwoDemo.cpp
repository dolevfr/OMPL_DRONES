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

    // Set default validity checker to accept all states
    // setup.getSpaceInformation()->setStateValidityChecker([](const ompl::base::State * /*state*/) -> bool {
    //     return true;
    // });

    // Ensure SpaceInformation is fully configured
    // setup.getSpaceInformation()->setup();

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
    payloadGoal->setX(125.0);
    payloadGoal->setY(125.0);
    payloadGoal->setZ(-300.0);
    payloadGoal->rotation().setIdentity(); // Quaternion: [0 0 0 1]

    unsigned int index = 1; // Start after payload (SE3)

    // Initialize quaternions for drones and spherical coordinates for cables
    for (unsigned int i = 0; i < droneCount; ++i)
    {
        // Initialize drone quaternion to [0 0 0 1]
        auto *droneQuatStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *droneQuatGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        droneQuatStart->values[0] = droneQuatGoal->values[0] = 0.0;
        droneQuatStart->values[1] = droneQuatGoal->values[1] = 0.0;
        droneQuatStart->values[2] = droneQuatGoal->values[2] = 0.0;
        droneQuatStart->values[3] = droneQuatGoal->values[3] = 1.0;
        ++index;

        // Initialize cable spherical coordinates (theta, phi)
        auto *cableAnglesStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *cableAnglesGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        cableAnglesStart->values[0] = cableAnglesGoal->values[0] = 0.0; // theta
        cableAnglesStart->values[1] = cableAnglesGoal->values[1] = 0.0; // phi
        ++index;
    }

    // Set payload velocity to [0 0 0]
    auto *payloadVelocityStart = startState->as<base::RealVectorStateSpace::StateType>(index);
    auto *payloadVelocityGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
    for (unsigned int i = 0; i < 3; ++i)
        payloadVelocityStart->values[i] = payloadVelocityGoal->values[i] = 0.0;
    ++index;

    // Set quaternion derivatives for payload, drones, and cable spherical coordinates to [0 0 0 0]
    auto *payloadQuatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(index);
    auto *payloadQuatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
    for (unsigned int i = 0; i < 4; ++i)
        payloadQuatDerivStart->values[i] = payloadQuatDerivGoal->values[i] = 0.0;
    ++index;

    for (unsigned int i = 0; i < droneCount; ++i)
    {
        // Set drone quaternion derivatives to [0 0 0 0]
        auto *droneQuatDerivStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *droneQuatDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        for (unsigned int j = 0; j < 4; ++j)
            droneQuatDerivStart->values[j] = droneQuatDerivGoal->values[j] = 0.0;
        ++index;

        // Set cable spherical coordinate derivatives (theta_dot, phi_dot) to [0 0]
        auto *cableAnglesDerivStart = startState->as<base::RealVectorStateSpace::StateType>(index);
        auto *cableAnglesDerivGoal = goalState->as<base::RealVectorStateSpace::StateType>(index);
        for (unsigned int j = 0; j < 2; ++j)
            cableAnglesDerivStart->values[j] = cableAnglesDerivGoal->values[j] = 0.0;
        ++index;
    }

    // Set the start and goal states
    setup.setStartAndGoalStates(startState, goalState, 0.5);

    // std::cout << "Start state: ";
    // startState.print(std::cout);
    // std::cout << "\nGoal state: ";
    // goalState.print(std::cout);
    // std::cout << std::endl;
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

        // Drone and cable states
        for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
        {
            // Get drone quaternion
            const auto *droneQuat = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << droneQuat->values[0] << " " << droneQuat->values[1] << " "
                    << droneQuat->values[2] << " " << droneQuat->values[3] << " ";

            // Get cable spherical coordinates (theta, phi)
            const auto *cableAngles = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << cableAngles->values[0] << " " << cableAngles->values[1] << " ";
        }

        // Payload velocity and quaternion derivatives
        const auto *payloadVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
        const auto *payloadQuatDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
        stateLine << payloadVelocity->values[0] << " " << payloadVelocity->values[1] << " " << payloadVelocity->values[2] << " ";
        stateLine << payloadQuatDeriv->values[0] << " " << payloadQuatDeriv->values[1] << " "
                << payloadQuatDeriv->values[2] << " " << payloadQuatDeriv->values[3] << " ";

        // Drone quaternion and cable spherical coordinate derivatives
        for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
        {
            // Get drone quaternion derivatives
            const auto *droneQuatDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << droneQuatDeriv->values[0] << " " << droneQuatDeriv->values[1] << " "
                    << droneQuatDeriv->values[2] << " " << droneQuatDeriv->values[3] << " ";

            // Get cable spherical coordinate derivatives (theta_dot, phi_dot)
            const auto *cableAnglesDeriv = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(index++);
            stateLine << cableAnglesDeriv->values[0] << " " << cableAnglesDeriv->values[1] << " ";
        }

        // Log state to file
        // std::cout << stateLine.str() << std::endl;
        outFile << stateLine.str() << "\n";
        outFile.flush();
    });





    try
    {
        if (setup.solve(3))
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
