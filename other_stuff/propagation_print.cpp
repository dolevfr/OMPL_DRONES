// In the demo function:





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


    // In post propagate:



        // // Print control values
    // const double *controlValues = static_cast<const ompl::control::RealVectorControlSpace::ControlType *>(control)->values;
    // std::ostringstream stateStream;
    // stateStream << "Control: [";
    // for (unsigned int i = 0; i < droneCount_ * 4; ++i) // Assuming 4 control inputs per drone
    //     stateStream << controlValues[i] << " ";
    // stateStream << "]\n";

    // // Print payload position, quaternion, and velocity
    // stateStream << "Payload Position: ["
    //             << payloadSE3State.getX() << ", " << payloadSE3State.getY() << ", " << payloadSE3State.getZ() << "] ";
    // stateStream << "Quaternion: ["
    //             << payloadSE3State.rotation().x << ", " << payloadSE3State.rotation().y << ", "
    //             << payloadSE3State.rotation().z << ", " << payloadSE3State.rotation().w << "] ";

    // const auto *payloadVel = compoundState->components[1]->as<ompl::base::RealVectorStateSpace::StateType>();
    // stateStream << "Payload Velocity: ["
    //             << payloadVel->values[0] << ", " << payloadVel->values[1] << ", " << payloadVel->values[2] << "]\n";

    // // Print drone and cable states with velocities
    // unsigned int startIndex = 2; // Start index after payload's SE3 state and velocity
    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
    //     // Drone quaternion
    //     auto *droneQuat = compoundState->components[startIndex + 3 * i]
    //                         ->as<ompl::base::SO3StateSpace::StateType>();
    //     stateStream << "Drone " << i << " Quaternion: ["
    //                 << droneQuat->x << ", " << droneQuat->y << ", "
    //                 << droneQuat->z << ", " << droneQuat->w << "] ";

    //     // Drone velocity
    //     auto *droneVel = compoundState->components[startIndex + 3 * i + 1]
    //                         ->as<ompl::base::RealVectorStateSpace::StateType>();
    //     stateStream << "Drone " << i << " Velocity: ["
    //                 << droneVel->values[0] << ", " << droneVel->values[1] << ", " << droneVel->values[2] << "] ";

    //     // Cable angles
    //     auto *cableAngles = compoundState->components[startIndex + 3 * i + 2]
    //                             ->as<ompl::base::RealVectorStateSpace::StateType>();
    //     stateStream << "Cable " << i << " Spherical Coordinates (theta, phi): ["
    //                 << cableAngles->values[0] << ", " << cableAngles->values[1] << "]\n";

    //     // Cable velocities
    //     stateStream << "Cable " << i << " Velocities (theta_dot, phi_dot): ["
    //                 << cableAngles->values[2] << ", " << cableAngles->values[3] << "]\n";
    // }

    // std::cout << "Post-Propagation State:\n" << stateStream.str();
    // std::cout << stateStream.str();