/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Mark Moll */

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "QuadrotorPlanning.h"
#include <omplapp/config.h>

using namespace ompl;

void quadrotorSetup(app::QuadrotorPlanning &setup)
{
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^3 part of SE(3)
    base::RealVectorBounds bounds(3);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getGeometricComponentStateSpace());
    start->setX(-10.);
    start->setY(0.);
    start->setZ(10.);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(setup.getGeometricComponentStateSpace());
    goal->setX(10.);
    goal->setY(0.);
    goal->setZ(10.);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(setup.getFullStateFromGeometricComponent(start),
                                setup.getFullStateFromGeometricComponent(goal), .5);
}

void quadrotorDemo(app::QuadrotorPlanning &setup)
{

    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    setup.setPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    // auto planner = std::make_shared<ompl::control::SST>(setup.getSpaceInformation());
    // planner->setGoalBias(0.05);
    // planner->setSelectionRadius(0.3);  // Adjust for faster convergence
    // planner->setPruningRadius(0.2);    // Helps control sparsity

    // // Attach the problem definition with the optimization objective to the planner
    // planner->setProblemDefinition(setup.getProblemDefinition());
    // planner->setup();

    // setup.setPlanner(planner);

    // try to solve the problem
    if (setup.solve(30))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl &path(setup.getSolutionPath());
        // path.interpolate(); // uncomment if you want to plot the path
        path.printAsMatrix(std::cout);

        // Save solution path to file
        std::ofstream outFile("solution_path.txt");
        path.printAsMatrix(outFile); // Save the solution matrix to the file
        outFile.close();

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
}

void quadrotorBenchmark(app::QuadrotorPlanning &setup)
{
    tools::Benchmark::Request request(100., 10000., 10);  // runtime (s), memory (MB), run count

    setup.setup();

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<control::KPIECE1>(setup.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char ** /*unused*/)
{
    app::QuadrotorPlanning quadrotor;

    // Set the meshes for environment and robot
    quadrotor.setEnvironmentMesh("/home/dolev/Desktop/Research/OMPL_drones/src/meshes/blocked.dae");
    quadrotor.setRobotMesh("/home/dolev/Desktop/Research/OMPL_drones/src/meshes/drone.dae");

    // Set propagation step size and control duration range
    quadrotor.getSpaceInformation()->setPropagationStepSize(0.01);
    quadrotor.getSpaceInformation()->setMinMaxControlDuration(1, 50);

    // Regular setup (defines start/goal etc.)
    quadrotorSetup(quadrotor);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once and print the path.
    if (argc > 1)
        quadrotorBenchmark(quadrotor);
    else
        quadrotorDemo(quadrotor);
    return 0;
}
