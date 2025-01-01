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

    // Payload bounds and initialization
    auto *compoundState = stateSpace->as<base::CompoundStateSpace>();
    if (!compoundState)
    {
        throw std::runtime_error("CompoundState is null. State space initialization failed.");
    }

    base::ScopedState<> start(stateSpace);

    // Set payload state
    auto *payloadStart = start->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    payloadStart->setX(125);
    payloadStart->setY(125);
    payloadStart->setZ(-150);
    payloadStart->rotation().setIdentity();

    for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    {
        unsigned int droneOrientationIndex = 1 + d * 2;
        unsigned int cableOrientationIndex = 2 + d * 2;
        unsigned int droneDerivativeIndex = 6 + d * 2;
        unsigned int cableDerivativeIndex = 7 + d * 2;

        // Drone orientation
        auto *droneOrientationStart = start->as<base::CompoundState>()
                                           ->as<base::SO3StateSpace::StateType>(droneOrientationIndex);
        if (!droneOrientationStart)
        {
            throw std::runtime_error("Drone orientation at index " + std::to_string(droneOrientationIndex) + " is null.");
        }
        droneOrientationStart->setIdentity();

        // Cable orientation
        auto *cableOrientationStart = start->as<base::CompoundState>()
                                           ->as<base::SO3StateSpace::StateType>(cableOrientationIndex);
        if (!cableOrientationStart)
        {
            throw std::runtime_error("Cable orientation at index " + std::to_string(cableOrientationIndex) + " is null.");
        }
        cableOrientationStart->setIdentity();

        // Drone derivatives
        auto *droneDerivativeStart = start->as<base::CompoundState>()
                                           ->as<base::RealVectorStateSpace::StateType>(droneDerivativeIndex);
        if (!droneDerivativeStart)
        {
            throw std::runtime_error("Drone derivatives at index " + std::to_string(droneDerivativeIndex) + " are null.");
        }
        for (unsigned int j = 0; j < 3; ++j)
            droneDerivativeStart->values[j] = 0.0;

        // Cable derivatives
        auto *cableDerivativeStart = start->as<base::CompoundState>()
                                           ->as<base::RealVectorStateSpace::StateType>(cableDerivativeIndex);
        if (!cableDerivativeStart)
        {
            throw std::runtime_error("Cable derivatives at index " + std::to_string(cableDerivativeIndex) + " are null.");
        }
        for (unsigned int j = 0; j < 3; ++j)
            cableDerivativeStart->values[j] = 0.0;
    }

    // Create the goal state
    base::ScopedState<> goal(stateSpace);

    // Set payload goal position and orientation
    auto *payloadGoal = goal->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    payloadGoal->setX(375);
    payloadGoal->setY(375);
    payloadGoal->setZ(-150);
    payloadGoal->rotation().setIdentity(); // Trivial quaternion

    // Set drone and cable orientations for goal state
    for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    {
        // Drone orientation (straight up)
        auto *droneOrientationGoal = goal->as<base::CompoundState>()
                                         ->as<base::SO3StateSpace::StateType>(1 + d * 2);
        droneOrientationGoal->setIdentity();

        // Cable orientation (straight up)
        auto *cableOrientationGoal = goal->as<base::CompoundState>()
                                         ->as<base::SO3StateSpace::StateType>(2 + d * 2);
        cableOrientationGoal->setIdentity();

        // Derivatives for drones and cables (all zero)
        auto *droneDerivativeGoal = goal->as<base::CompoundState>()
                                         ->as<base::RealVectorStateSpace::StateType>(6 + d * 2);
        for (unsigned int j = 0; j < 3; ++j)
        {
            droneDerivativeGoal->values[j] = 0.0;
        }

        auto *cableDerivativeGoal = goal->as<base::CompoundState>()
                                         ->as<base::RealVectorStateSpace::StateType>(7 + d * 2);
        for (unsigned int j = 0; j < 3; ++j)
        {
            cableDerivativeGoal->values[j] = 0.0;
        }
    }

    // Set start and goal states in the planner
    setup.setStartAndGoalStates(start, goal, 0.5);

    // Validity checker: Placeholder for user-defined validity checks
    setup.setStateValidityChecker([&setup](const ompl::base::State *state) -> bool {
        return true; // Modify as necessary for collision checks, etc.
    });

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

    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);


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
