#pragma once
#include <Eigen/Dense>

struct ExperimentConfig {
    double maxTorquePitchRoll;
    double maxTorqueYaw;
    double minThrust;
    double maxThrust;
    double maxDroneAngle;
    double maxDroneVel;
    double maxAnglePayload;
    double maxPayloadVel;
    double maxPayloadAngVel;
    double maxTheta;
    double maxThetaVel;

    double thrustStd;
    double torquePitchRollStd;
    double torqueYawStd;

    bool sameControls;
    bool useSST;
    bool printAllStates;

    Eigen::Vector3d startPosition;
    Eigen::Vector3d goalPosition;

    double solveTime;
    double goalBias;
};
