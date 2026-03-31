#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "config.hpp"
#include "mainpulator_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "manipulator_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface arm(node, "arm");
    applyArmConfig(arm);

    MoveGroupInterface gripper(node, "gripper");

    auto poses = loadPoses(
        "/home/hoan/colcon_ws/src/open_manipulator/"
        "open_manipulator_x_playground/src/points.json");

    pickup(arm, gripper, poses);

    rclcpp::shutdown();
    return 0;
}