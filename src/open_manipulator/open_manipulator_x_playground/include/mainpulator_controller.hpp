#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <cmath>
#include <stdexcept>

using json = nlohmann::json;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Pose      = geometry_msgs::msg::Pose;

// ── Utilities ────────────────────────────────────────────────

inline bool isXCrackZone(const Pose & p)
{
    return p.position.x >= 0.38 && p.position.x <= 0.46;
}

inline std::vector<Pose> loadPoses(const std::string & path)
{
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Cannot open JSON file: " + path);

    json data;
    file >> data;

    std::vector<Pose> poses;
    for (auto & p : data)
    {
        Pose pose;
        pose.position.x    = p["x"];
        pose.position.y    = p["y"];
        pose.position.z    = p["z"];
        pose.orientation.x = p.value("qx", 0.0);
        pose.orientation.y = p.value("qy", 0.0);
        pose.orientation.z = p.value("qz", 0.0);
        pose.orientation.w = p.value("qw", 1.0);
        poses.push_back(pose);
    }
    return poses;
}

inline bool needSafePose(const Pose & cur, const Pose & tgt, double z_th = 0.05, double qw_th = 0.9)
{
    double dot = cur.orientation.x * tgt.orientation.x
               + cur.orientation.y * tgt.orientation.y
               + cur.orientation.z * tgt.orientation.z
               + cur.orientation.w * tgt.orientation.w;
    return std::abs(tgt.position.z - cur.position.z) > z_th || std::abs(dot) < qw_th;
}

// ── Low-level ────────────────────────────────────────────────

inline bool executeJointStep(
    MoveGroup & arm,
    std::map<std::string, double> & working,
    const std::map<std::string, double> & target,
    const std::vector<std::string> & joints)
{
    bool changed = false;
    for (const auto & j : joints)
    {
        auto it = target.find(j);
        if (it == target.end()) continue;
        working[j] = it->second;
        changed = true;
    }
    if (!changed) return true;

    arm.stop();
    arm.clearPoseTargets();
    arm.clearPathConstraints();
    arm.setStartStateToCurrentState();
    for (const auto & j : joints) arm.setJointValueTarget(j, working[j]);

    MoveGroup::Plan plan;
    return static_cast<bool>(arm.plan(plan)) && static_cast<bool>(arm.execute(plan));
}

inline bool moveJointSequential(MoveGroup & arm, const std::map<std::string, double> & target, bool going_up)
{
    std::map<std::string, double> working;
    auto vals = arm.getCurrentJointValues();
    auto names = arm.getJointNames();
    for (size_t i = 0; i < names.size(); i++) working[names[i]] = vals[i];

    if (going_up)
        return executeJointStep(arm, working, target, {"joint1"})
            && executeJointStep(arm, working, target, {"joint2", "joint3", "joint4"});
    else
        return executeJointStep(arm, working, target, {"joint2", "joint3", "joint4"})
            && executeJointStep(arm, working, target, {"joint1"});
}

// ── Named poses ──────────────────────────────────────────────

inline bool moveToCentral(MoveGroup & arm)
{
    return moveJointSequential(arm, arm.getNamedTargetValues("central_point"), false);
}

inline bool moveNamedSequential(MoveGroup & arm, const std::string & name, bool up)
{
    auto target = arm.getNamedTargetValues(name);
    return !target.empty() && moveJointSequential(arm, target, up);
}

// ── Pose-level ───────────────────────────────────────────────

inline bool movePoseSequential(MoveGroup & arm, const Pose & pose)
{
    arm.stop();
    arm.clearPoseTargets();
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(pose);

    MoveGroup::Plan plan;
    if (!static_cast<bool>(arm.plan(plan))) return false;
    if (plan.trajectory_.joint_trajectory.points.empty()) return false;

    const auto & last  = plan.trajectory_.joint_trajectory.points.back();
    const auto & names = plan.trajectory_.joint_trajectory.joint_names;

    std::map<std::string, double> target;
    for (size_t i = 0; i < names.size(); i++) target[names[i]] = last.positions[i];

    return moveJointSequential(arm, target, true);
}

inline bool moveOrientationOnly(MoveGroup & arm, const Pose & target)
{
    if (!moveToCentral(arm)) return false;

    Pose safe = target;
    safe.position.x = 0.35;
    arm.stop();
    arm.clearPoseTargets();
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(safe);

    MoveGroup::Plan plan;
    if (!static_cast<bool>(arm.plan(plan))) return false;
    if (plan.trajectory_.joint_trajectory.points.empty()) return false;

    const auto & traj_names = plan.trajectory_.joint_trajectory.joint_names;
    const auto & last_pt    = plan.trajectory_.joint_trajectory.points.back();

    double j1 = 0.0;
    for (size_t k = 0; k < traj_names.size(); k++)
        if (traj_names[k] == "joint1") { j1 = last_pt.positions[k]; break; }

    auto central = arm.getNamedTargetValues("central_point");
    central["joint1"] = j1;

    std::map<std::string, double> working;
    auto vals  = arm.getCurrentJointValues();
    auto names = arm.getJointNames();
    for (size_t i = 0; i < names.size(); i++) working[names[i]] = vals[i];

    return executeJointStep(arm, working, central, {"joint1"});
}

// ── Tasks ────────────────────────────────────────────────────

inline bool pickup(MoveGroup & arm, MoveGroup & gripper, const std::vector<Pose> & poses)
{
    if (poses.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup(): poses list is empty.");
        return false;
    }

    const Pose & grasp = poses[0];

    // 1. Đến điểm gấp vật
    bool ok = isXCrackZone(grasp) ? moveOrientationOnly(arm, grasp) : movePoseSequential(arm, grasp);
    if (!ok) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed grasp point."); return false; }

    // 2. Đóng gripper
    gripper.setNamedTarget("close");
    if (!static_cast<bool>(gripper.move())) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed close gripper."); return false; }

    // 3. central → pick_up
    if (!moveToCentral(arm)) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed central (up)."); return false; }
    if (!moveNamedSequential(arm, "pick_up", true)) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed pick_up."); return false; }

    // 4. Mở gripper
    gripper.setNamedTarget("open");
    if (!static_cast<bool>(gripper.move())) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed open gripper."); return false; }

    // 5. central → break
    if (!moveToCentral(arm)) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed central (down)."); return false; }
    if (!moveNamedSequential(arm, "break", false)) { RCLCPP_ERROR(rclcpp::get_logger("manipulator_controller"), "pickup: failed break."); return false; }

    RCLCPP_INFO(rclcpp::get_logger("manipulator_controller"), "pickup: complete.");
    return true;
}