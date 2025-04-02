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

#include "PayloadOneDrone.h"

using namespace ompl;

void payloadSystemSetup(app::PayloadSystem &setup)
{
    // Set the start and goal states
    base::StateSpacePtr stateSpace(setup.getStateSpace());
    base::ScopedState<base::CompoundStateSpace> start(stateSpace);
    const Eigen::Vector3d &startPos = setup.getStartPosition();
    start->as<base::SE3StateSpace::StateType>(0)->setXYZ(startPos.x(), startPos.y(), startPos.z());

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
    const Eigen::Vector3d &goalPos = setup.getGoalPosition();
    goal->as<base::SE3StateSpace::StateType>(0)->setXYZ(goalPos.x(), goalPos.y(), goalPos.z());

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

    // Ensure state space and si_ are initialized first
    auto validityChecker = std::make_shared<PayloadSystemValidityChecker>(setup.getSpaceInformation(), setup);
    setup.getSpaceInformation()->setStateValidityChecker(validityChecker);

}




void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    auto si = setup.getSpaceInformation();
    
    auto sampler = std::make_shared<PayloadSmoothDirectedControlSampler>(si.get(), &setup);
    si->setDirectedControlSamplerAllocator([sampler](const ompl::control::SpaceInformation *) {
        return sampler;
    });    

    std::cout << "Setting up the planner...\n";
    

    if (setup.getUseSST()) {
        auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);

        // Set the optimization objective in the problem definition
        setup.getProblemDefinition()->setOptimizationObjective(objective);

        auto planner = std::make_shared<ompl::control::SST>(si);
        planner->setGoalBias(0.05);
        planner->setSelectionRadius(3.0);  // Adjust for faster convergence
        planner->setPruningRadius(1.0);    // Helps control sparsity

        // Attach the problem definition with the optimization objective to the planner
        planner->setProblemDefinition(setup.getProblemDefinition());
        planner->setup();

        setup.setPlanner(planner);
    }
    else {
        // Set up the planner
        auto planner = std::make_shared<control::RRT>(si);
        planner->setGoalBias(0.05); // Example: Adjust goal bias
        setup.setPlanner(planner);
    }

    // // ----------------- Print the states to the file -----------------

    std::shared_ptr<std::ofstream> outFile;

    if (setup.getPrintAllStates())
    {
        // Open file for writing and ensure it works
        outFile = std::make_shared<std::ofstream>("solution_path.txt", std::ios::out | std::ios::trunc);
        if (!outFile->is_open())
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

            // auto endPropagate = std::chrono::high_resolution_clock::now();
            // auto durationPropagate = std::chrono::duration_cast<std::chrono::microseconds>(endPropagate - startPropagate).count();
            // std::cout << "Propagation computation time: " << durationPropagate << " µs\n";
            
            auto startStateLine = std::chrono::high_resolution_clock::now();

            // Check if the propagated state is valid
            if (!setup.getSpaceInformation()->isValid(to))
            {
                // std::cerr << "Warning: Propagated state is invalid. Skipping.\n";
                return;
            }
            // else
            // {
            //     std::cout << "Propagated state is valid.\n";
            // }

            const auto *compoundState = to->as<ompl::base::CompoundState>();


            // Construct the state line for logging the system state and inputs
            std::ostringstream stateLine;

            // Payload SE3 state (position XYZ, orientation quaternion XYZW)
            const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
            stateLine << payloadState->getX() << " " << payloadState->getY() << " " << payloadState->getZ() << " ";
            stateLine << payloadState->rotation().x << " " << payloadState->rotation().y << " "
                    << payloadState->rotation().z << " " << payloadState->rotation().w << " ";

            // Payload velocity (6 dimensions: linear XYZ, angular XYZ)
            const auto *payloadVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
            for (unsigned int i = 0; i < 6; ++i)
            {
                stateLine << payloadVelocity->values[i] << " ";
            }

            // Drone states (orientation, velocity, cable parameters)
            for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
            {
                unsigned int baseIndex = 2 + d * 3;

                // Drone orientation (quaternion XYZW)
                const auto *droneOrientation = compoundState->as<ompl::base::SO3StateSpace::StateType>(baseIndex);
                stateLine << droneOrientation->x << " " << droneOrientation->y << " "
                        << droneOrientation->z << " " << droneOrientation->w << " ";

                // Drone velocity (3 dimensions XYZ)
                const auto *droneVelocity = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
                for (unsigned int i = 0; i < 3; ++i)
                {
                    stateLine << droneVelocity->values[i] << " ";
                }

                // Cable parameters (angles and angular velocities)
                const auto *cableState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
                for (unsigned int i = 0; i < 4; ++i)
                {
                    stateLine << cableState->values[i] << " ";
                }
            }

            // Append control inputs (thrust and torques for each drone)
            const double *controlValues = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
            for (unsigned int d = 0; d < setup.getRobotCount(); ++d)
            {
                unsigned int controlBase = d * 4; // 4 control inputs per drone
                for (unsigned int i = 0; i < 4; ++i)
                {
                    stateLine << controlValues[controlBase + i] << " ";
                }
            }

            // Append the duration of the control
            stateLine << duration;

            std::string line = stateLine.str();
            // std::cout << line << "\n";  // Print to console
            *outFile << line << "\n";    // Save to file
            outFile->flush();            // Ensure the line is written immediately
        });
    }

    //     // ----------------- Print the states to the file -----------------
    // Solve the planning problem
    if (setup.solve(setup.getSolveTime()))
    {
        std::cout << "Planning completed successfully.\n";

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

    // Load Environment and Robot Meshes
    std::string meshDir = boost::filesystem::absolute("../src/meshes").string();
    multiDrone.setMeshPath({boost::filesystem::path(meshDir)});

    multiDrone.setEnvironmentMesh(meshDir + "/empty_env.dae");
    multiDrone.setRobotMesh(meshDir + "/box_collision.dae");
    
    // Setup MultiDrone planning environment
    payloadSystemSetup(multiDrone);

    // Run MultiDrone planning demo
    payloadSystemDemo(multiDrone);

    if (!multiDrone.getPrintAllStates())
    {
        // Save solution path to file
        std::ofstream outFile("solution_path.txt");
        control::PathControl &path(multiDrone.getSolutionPath());
        path.printAsMatrix(outFile); // Save the solution matrix to the file
        outFile.close();
    }

    // system("python3 ../src/python/print_solution.py ../build/solution_path.txt");

    system("python3 /home/dolev/Desktop/Research/OMPL_drones/src/python/plot_trajectories_one.py");

    // system("python3 ../src/python/extract_se3.py solution_path.txt solution_path_se3.txt");
    // system("python3 ../src/python/ompl_app_multiple.py");
}
