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

#include "PayloadFourDrones.h"

using namespace ompl;

void payloadSystemSetup(app::PayloadSystem &setup)
{
    // Set the start and goal states
    base::StateSpacePtr stateSpace(setup.getStateSpace());
    base::ScopedState<base::CompoundStateSpace> start(stateSpace);
    start->as<base::SE3StateSpace::StateType>(0)->setXYZ(-10.0, 0.0, 10.0);
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
    goal->as<base::SE3StateSpace::StateType>(0)->setXYZ(10.0, 0.0, 10.0);
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

}




void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // // Set up the planner
    // auto planner = std::make_shared<control::RRT>(setup.getSpaceInformation());
    // planner->setGoalBias(0.1); // Example: Adjust goal bias
    // setup.setPlanner(planner);

    auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(setup.getSpaceInformation());

    // Set the optimization objective in the problem definition
    setup.getProblemDefinition()->setOptimizationObjective(objective);

    auto planner = std::make_shared<ompl::control::SST>(setup.getSpaceInformation());
    planner->setGoalBias(0.05);
    planner->setSelectionRadius(0.05);  // Adjust for faster convergence
    planner->setPruningRadius(0.1);    // Helps control sparsity

    // Attach the problem definition with the optimization objective to the planner
    planner->setProblemDefinition(setup.getProblemDefinition());
    planner->setup();

    setup.setPlanner(planner);


    // Solve the planning problem
    // if (setup.solve(base::IterationTerminationCondition(2)))
    if (setup.solve(setup.getSolveTime()))
    {
        std::cout << "Planning completed successfully.\n";
        // control::PathControl &path(setup.getSolutionPath());
        // path.printAsMatrix(std::cout); // Optionally print the solution matrix

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

    system("python3 ../src/python/print_solution.py ../build/solution_path.txt");

    // system("python3 /home/dolev/Desktop/Research/OMPL_drones/src/python/plot_trajectories_3D.py");

    system("python3 ../src/python/extract_se3.py solution_path.txt solution_path_se3.txt");
    system("python3 ../src/python/ompl_app_multiple.py");
}
