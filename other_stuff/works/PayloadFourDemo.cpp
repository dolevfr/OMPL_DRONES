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
#include "PayloadClasses.h"


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
}




void payloadSystemDemo(app::PayloadSystem &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    auto si = setup.getSpaceInformation();

    si->setDirectedControlSamplerAllocator(
        [&setup](const ompl::control::SpaceInformation *si) {
            return std::make_shared<PayloadSmoothDirectedControlSampler>(si, &setup);
        }
    );

    std::cout << "Setting up the planner...\n";
    

    if (setup.getUseSST()) {
        auto objective = std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(si);

        // Set the optimization objective in the problem definition
        setup.getProblemDefinition()->setOptimizationObjective(objective);

        // auto planner = std::make_shared<MySST>(si);
        auto planner = std::make_shared<MySST>(si, &setup);
        planner->setGoalBias(0.05);
        planner->setSelectionRadius(0.1);  // Adjust for faster convergence
        planner->setPruningRadius(0.05);    // Helps control sparsity

        // Attach the problem definition with the optimization objective to the planner
        planner->setProblemDefinition(setup.getProblemDefinition());
        planner->setup();

        setup.setPlanner(planner);
    }
    else {
        // Set up the planner
        auto planner = std::make_shared<MyRRT>(si);
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

            // Check if the propagated state is valid
            if (!setup.getSpaceInformation()->isValid(to))
            {
                // std::cerr << "Warning: Propagated state is invalid. Skipping.\n";
                return;
            }

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
        // std::cout << "Planning completed successfully.\n";

        // if (!setup.haveExactSolutionPath())
        // {
        //     std::cout << "Solution is approximate. Distance to actual goal is "
        //               << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        // }
        double dist = setup.haveExactSolutionPath() ? 0.0
                    : setup.getProblemDefinition()->getSolutionDifference();
        std::cout << "RESULT distance=" << dist << '\n';
    }
    else
    {
        std::cout << "No solution found within the time limit.\n";
    }
}



int main(int argc, char** argv)
{
    ompl::app::PayloadSystem sys;

    /* ---------- parse  --key value  and  --key=value  ---------- */
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg.rfind("--", 0) == 0)          // starts with “--”
        {
            std::string key = arg.substr(2);  // remove dashes
            std::string valstr;

            /* allow both  --k=v  and  --k v */
            if (auto eq = key.find('='); eq != std::string::npos)
            {
                valstr = key.substr(eq + 1);
                key.erase(eq);
            }
            else if (i + 1 < argc)
            {
                valstr = argv[++i];
            }
            else
            {
                std::cerr << "Missing value for " << arg << '\n';
                return 1;
            }

            try
            {
                double v = std::stod(valstr);
                if (!sys.setParam(key, v))
                    std::cerr << "Unknown parameter '" << key << "'\n";
            }
            catch (...)
            {
                std::cerr << "Bad value '" << valstr << "' for " << key << '\n';
                return 1;
            }
        }
        /* you could add single-dash or boolean flags here if needed */
    }

    sys.setDefaultBounds();          // re-compute limits once

    /* -------------- original demo setup continues -------------- */
    std::string meshDir = boost::filesystem::absolute("../src/meshes").string();
    sys.setMeshPath({boost::filesystem::path(meshDir)});
    sys.setEnvironmentMesh(meshDir + "/Apartment_env.dae");
    sys.setRobotMesh      (meshDir + "/box_collision.dae");

    payloadSystemSetup(sys);
    payloadSystemDemo (sys);

    /* ---------- save best path only if distance improves ---------------- */
    if (!sys.getPrintAllStates())
    {
        /* current run’s distance (0 if exact) --------------------------- */
        double currDist = sys.haveExactSolutionPath() ? 0.0
                    : sys.getProblemDefinition()->getSolutionDifference();

        /* read previous best (if any) ----------------------------------- */
        double bestDist = std::numeric_limits<double>::infinity();
        std::ifstream bestIn("best_distance.txt");
        if (bestIn) bestIn >> bestDist;

        if (currDist < bestDist)                       // better → overwrite
        {
            std::ofstream out("solution_path.txt");
            ompl::control::PathControl &path(sys.getSolutionPath());
            path.printAsMatrix(out);                   // save new path

            std::ofstream bestOut("best_distance.txt", std::ios::trunc);
            bestOut << currDist;                       // update marker
            std::cout << "🆕  Saved new BEST path (distance = "
                    << currDist << ")\n";
        }
        else
        {
            std::cout << "ℹ️  Path not saved (distance " << currDist
                    << " ≥ best " << bestDist << ")\n";
        }

        /* always print a machine-parsable line for the Python driver ---- */
        std::cout << "RESULT distance=" << currDist << std::endl;
    }


    // system("python3 ../src/python/print_solution.py ../build/solution_path.txt");

    // system("python3 ../src/python/plot_trajectories_four.py");

    // system("python3 ../src/python/extract_se3.py solution_path.txt solution_path_se3.txt");
    // system("python3 ../src/python/ompl_app_multiple.py");
}
