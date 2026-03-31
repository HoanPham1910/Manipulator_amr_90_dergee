#pragma once

namespace ArmConfig
{
    constexpr double PLANNING_TIME        = 10.0;
    constexpr int    NUM_PLANNING_ATTEMPTS = 1;
    constexpr double GOAL_POSITION_TOL    = 0.01;
    constexpr double GOAL_ORIENTATION_TOL = 0.05;
    constexpr double MAX_VELOCITY         = 1.0;
    constexpr double MAX_ACCELERATION     = 1.0;
}

inline void applyArmConfig(moveit::planning_interface::MoveGroupInterface & arm)
{
    arm.setPlanningTime(ArmConfig::PLANNING_TIME);
    arm.setNumPlanningAttempts(ArmConfig::NUM_PLANNING_ATTEMPTS);
    arm.setGoalPositionTolerance(ArmConfig::GOAL_POSITION_TOL);
    arm.setGoalOrientationTolerance(ArmConfig::GOAL_ORIENTATION_TOL);
    arm.setMaxVelocityScalingFactor(ArmConfig::MAX_VELOCITY);
    arm.setMaxAccelerationScalingFactor(ArmConfig::MAX_ACCELERATION);
}