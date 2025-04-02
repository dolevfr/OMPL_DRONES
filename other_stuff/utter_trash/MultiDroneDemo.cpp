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

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/goals/GoalRegion.h>
#include "MultiDronePlanning.h"
#include <omplapp/config.h>
#include <iostream>
#include <fstream>
#include <filesystem>

using namespace ompl;

void multiDroneSetup(app::MultiDronePlanning &setup)
{
    base::StateSpacePtr stateSpace = setup.getStateSpace();

    // Set bounds for SE3 subspaces
    base::RealVectorBounds bounds(3);
    bounds.setLow(-200);
    bounds.setHigh(500);
    for (unsigned int i = 0; i < setup.getRobotCount(); ++i)
    {
        stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(i * 2)->setBounds(bounds);
    }

    // Define explicit start and goal positions for all drones
    std::vector<std::tuple<double, double, double>> startPositions = {
        {100, 100, -100}, {150, 100, -100}, {150, 150, -100}, {100, 150, -100}};

    std::vector<std::tuple<double, double, double>> goalPositions = {
        {350, 350, -100}, {400, 350, -100}, {400, 400, -100}, {350, 400, -100}};

    // Define start and goal states for all drones
    base::ScopedState<> start(stateSpace);
    base::ScopedState<> goal(stateSpace);

    for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
    {
        // Start state
        auto *droneStart = start->as<base::CompoundState>()->components[d * 2]->as<ompl::base::SE3StateSpace::StateType>();
        droneStart->setX(std::get<0>(startPositions[d]));
        droneStart->setY(std::get<1>(startPositions[d]));
        droneStart->setZ(std::get<2>(startPositions[d]));
        droneStart->rotation().setIdentity();

        auto *velocityStart = start->as<base::CompoundState>()->components[d * 2 + 1]->as<base::RealVectorStateSpace::StateType>();
        for (unsigned int j = 0; j < 6; ++j)
        {
            velocityStart->values[j] = 0.0; // Initialize all velocity components to 0
        }

        // Goal state
        auto *droneGoal = goal->as<base::CompoundState>()->components[d * 2]->as<ompl::base::SE3StateSpace::StateType>();
        droneGoal->setX(std::get<0>(goalPositions[d]));
        droneGoal->setY(std::get<1>(goalPositions[d]));
        droneGoal->setZ(std::get<2>(goalPositions[d]));
        droneGoal->rotation().setIdentity();
    }

    // Convert SE3 start and goal to full states
    auto fullStart = setup.getFullStateFromGeometricComponent(start);
    auto fullGoal = setup.getFullStateFromGeometricComponent(goal);


    // Set the start and goal states
    setup.setStartAndGoalStates(fullStart, fullGoal, 0.5);
}


void multiDroneDemo(app::MultiDronePlanning &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // Set up the planner
    auto planner = std::make_shared<control::RRT>(setup.getSpaceInformation());
    planner->setGoalBias(0.05); // Example: Adjust goal bias
    setup.setPlanner(planner);

    // Adjust the goal region threshold for a farther distance
    auto goalRegion = std::dynamic_pointer_cast<ompl::base::GoalRegion>(setup.getProblemDefinition()->getGoal());
    if (goalRegion)
    {
        goalRegion->setThreshold(1000.0); // Set a large threshold
        std::cout << "Goal threshold set to " << goalRegion->getThreshold() << ".\n";
    }
    else
    {
        std::cerr << "Error: Goal is not a GoalRegion.\n";
        return;
    }

    // Open file for writing and ensure it works
    std::ofstream outFile("solution_path.txt", std::ios::out | std::ios::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Unable to open file for writing solution path.\n";
        return;
    }

    // Wrap the state propagator to collect and print propagated states
    auto originalPropagator = setup.getSpaceInformation()->getStatePropagator();
    setup.getSpaceInformation()->setStatePropagator([&setup, originalPropagator, &outFile](
                                                        const ompl::base::State *from,
                                                        const ompl::control::Control *control,
                                                        double duration,
                                                        ompl::base::State *to) {
        // Perform state propagation
        originalPropagator->propagate(from, control, duration, to);

        // Check if the propagated state is valid
        if (!setup.getSpaceInformation()->isValid(to))
        {
            // std::cerr << "Warning: Propagated state is invalid. Skipping.\n";
            return;
        }

        const auto *compoundState = to->as<ompl::base::CompoundState>();

        // Construct the line for the file
        std::ostringstream stateLine;
        for (int d = 0; d < setup.getRobotCount(); ++d)
        {
            const auto *droneState = compoundState->as<ompl::base::SE3StateSpace::StateType>(d * 2);

            stateLine << droneState->getX() << " " << droneState->getY() << " " << droneState->getZ() << " ";
            stateLine << droneState->rotation().x << " " << droneState->rotation().y << " "
                    << droneState->rotation().z << " " << droneState->rotation().w << " ";
        }

        std::string line = stateLine.str();
        // std::cout << line << "\n";  // Print to console
        outFile << line << "\n";    // Save to file
        outFile.flush();            // Ensure the line is written immediately
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
        std::cout << "Solution path saved to solution_path.txt.\n";
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
    app::MultiDronePlanning multiDrone;

    // Setup MultiDrone planning environment
    multiDroneSetup(multiDrone);

    // Run MultiDrone planning demo
    multiDroneDemo(multiDrone);

    system("cd .. && python3 plot_trajectories.py");

    // // Save solution path to file
    // saveSolutionPath(multiDrone.getSolutionPath(), multiDrone.getRobotCount());
}
