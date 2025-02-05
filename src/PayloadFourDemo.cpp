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
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "PayloadFourDrones.h"

using namespace ompl;

void payloadSystemSetup(app::PayloadSystem &setup)
{
    base::StateSpacePtr stateSpace(setup.getStateSpace());
    base::ScopedState<base::CompoundStateSpace> start(stateSpace);
    start->as<base::SE3StateSpace::StateType>(0)->setXYZ(125.0, 125.0, -150.0);
    start->as<base::SE3StateSpace::StateType>(0)->rotation().setIdentity();
    for (unsigned int i = 0; i < 6; ++i)
    {
        start->as<base::RealVectorStateSpace::StateType>(1)->values[i] = 0.0;
    }
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        unsigned int baseIndex = 2 + i * 3;
        start->as<base::SO3StateSpace::StateType>(baseIndex)->setIdentity();
        for (unsigned int j = 0; j < 3; ++j)
        {
            start->as<base::RealVectorStateSpace::StateType>(baseIndex + 1)->values[j] = 0.0;
        }
        for (unsigned int j = 0; j < 4; ++j)
        {
            start->as<base::RealVectorStateSpace::StateType>(baseIndex + 2)->values[j] = 0.0;
        }
    }
    base::ScopedState<base::CompoundStateSpace> goal(stateSpace);
    goal->as<base::SE3StateSpace::StateType>(0)->setXYZ(150.0, 125.0, -150.0);
    goal->as<base::SE3StateSpace::StateType>(0)->rotation().setIdentity();
    for (unsigned int i = 0; i < 6; ++i)
    {
        goal->as<base::RealVectorStateSpace::StateType>(1)->values[i] = 0.0;
    }
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        unsigned int baseIndex = 2 + i * 3;
        goal->as<base::SO3StateSpace::StateType>(baseIndex)->setIdentity();
        for (unsigned int j = 0; j < 3; ++j)
        {
            goal->as<base::RealVectorStateSpace::StateType>(baseIndex + 1)->values[j] = 0.0;
        }
        for (unsigned int j = 0; j < 4; ++j)
        {
            goal->as<base::RealVectorStateSpace::StateType>(baseIndex + 2)->values[j] = 0.0;
        }
    }

    setup.setStartAndGoalStates(start, goal);

    // // Retrieve the state space
    // ompl::base::StateSpacePtr stateSpace = setup.getStateSpace();

    // // Define the payload goal position
    // Eigen::Vector3d goalPosition(150.0, 125.0, -150.0);

    // // Create a goal based only on payload position
    // auto goal = std::make_shared<PayloadPositionGoal>(setup.getSpaceInformation(), goalPosition);

    // // Set the goal in the planning setup
    // setup.setGoal(goal);

    // // Create the start state
    // ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(stateSpace);
    // start->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(125.0, 125.0, -150.0);
    // start->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    // for (unsigned int i = 0; i < 6; ++i)
    // {
    //     start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = 0.0;
    // }

    // for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    // {
    //     unsigned int baseIndex = 2 + i * 3;
    //     start->as<ompl::base::SO3StateSpace::StateType>(baseIndex)->setIdentity();
    //     for (unsigned int j = 0; j < 3; ++j)
    //     {
    //         start->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1)->values[j] = 0.0;
    //     }
    //     for (unsigned int j = 0; j < 4; ++j)
    //     {
    //         start->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2)->values[j] = 0.0;
    //     }
    // }

    // // Set start state
    // setup.setStartState(start);
}



void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // Set up the planner
    auto planner = std::make_shared<control::RRT>(setup.getSpaceInformation());
    planner->setGoalBias(0.05); // Example: Adjust goal bias
    setup.setPlanner(planner);

    // // Open file for writing and ensure it works
    // std::ofstream outFile("solution_path.txt", std::ios::out | std::ios::trunc);
    // if (!outFile.is_open())
    // {
    //     std::cerr << "Error: Unable to open file for writing solution path.\n";
    //     return;
    // }
    // std::cout << "Writing propagated states to solution_path.txt...\n";
    // // Wrap the state propagator to log every propagated state
    // auto originalPropagator = setup.getSpaceInformation()->getStatePropagator();
    // setup.getSpaceInformation()->setStatePropagator([&setup, originalPropagator, &outFile](
    //                                                     const ompl::base::State *from,
    //                                                     const ompl::control::Control *control,
    //                                                     double duration,
    //                                                     ompl::base::State *to) {
    //     // Perform state propagation
    //     originalPropagator->propagate(from, control, duration, to);

    //     // Log the propagated state
    //     const auto *compoundState = to->as<ompl::base::CompoundState>();
    //     if (!compoundState)
    //     {
    //         std::cerr << "Error: Propagated state is invalid.\n";
    //         return;
    //     }

    //     std::ostringstream stateLine;

    //     // Log payload state
    //     const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
    //     stateLine << payloadState->getX() << " " << payloadState->getY() << " " << payloadState->getZ() << " ";
    //     stateLine << payloadState->rotation().x << " " << payloadState->rotation().y << " "
    //           << payloadState->rotation().z << " " << payloadState->rotation().w << " ";

    //     // Log payload velocities
    //     const auto *payloadVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
    //     for (unsigned int i = 0; i < 6; ++i)
    //     {
    //         stateLine << payloadVelocity->values[i] << " ";
    //     }

    //     // Log drone states
    //     for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    //     {
    //         unsigned int baseIndex = 2 + d * 3;

    //         // Log drone orientation
    //         const auto *droneOrientation = compoundState->as<ompl::base::SO3StateSpace::StateType>(baseIndex);
    //         stateLine << droneOrientation->x << " " << droneOrientation->y << " "
    //               << droneOrientation->z << " " << droneOrientation->w << " ";

    //         // Log drone velocities
    //         const auto *droneVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
    //         for (unsigned int i = 0; i < 3; ++i)
    //         {
    //         stateLine << droneVelocity->values[i] << " ";
    //         }

    //         // Log cable angles and velocities
    //         const auto *cableState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
    //         for (unsigned int i = 0; i < 4; ++i)
    //         {
    //         stateLine << cableState->values[i] << " ";
    //         }
    //     }

    //     outFile << stateLine.str() << "\n";
    //     outFile.flush(); // Ensure the data is written immediately
    // });

    // Solve the planning problem
    // if (setup.solve(base::IterationTerminationCondition(2)))
    if (setup.solve(setup.getSolveTime()))
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
    std::ofstream outFile("solution_path.txt");
    control::PathControl &path(multiDrone.getSolutionPath());
    path.printAsMatrix(outFile); // Save the solution matrix to the file
    outFile.close();

    system("python3 /home/dolev/Desktop/Research/OMPL_drones/src/python/plot_trajectories_3D.py");
}
