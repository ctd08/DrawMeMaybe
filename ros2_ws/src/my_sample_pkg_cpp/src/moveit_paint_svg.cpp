#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <cstdio>
//necessary memory included
#include <memory>
//rclcpp - Ros Client Libirary for C++ package
#include <rclcpp/rclcpp.hpp>
//this is necessart to plan  and execute motion
#include <moveit/move_group_interface/move_group_interface.h>
//for converting Euler anglles to quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit_msgs/msg/display_robot_state.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");
//static const rclpp::Logger Logger = rclpp::get_logger("move_group");

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node =rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

	rclcpp:: executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread([&executor]() {executor.spin(); }) .detach();
	static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
	moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);

    // >>> globale Geschwindigkeit (20%) von max <<<
    move_group_arm.setMaxVelocityScalingFactor(0.2);
    move_group_arm.setMaxAccelerationScalingFactor(0.2);

	const moveit::core::JointModelGroup *joint_model_group_arm =
	move_group_arm. getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

	//Get Current State
	moveit::core::RobotStatePtr current_state_arm =
	move_group_arm.getCurrentState(10);
	std::vector<double> joint_group_positions_arm;
	current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
	move_group_arm.setStartStateToCurrentState();

	//Go Home
	RCLCPP_INFO(LOGGER, "Going Home");
	double deg2rad = M_PI / 180.0;
	joint_group_positions_arm[0] = -76.25 * deg2rad; //Shoulder Pan
	joint_group_positions_arm[1] = -81.76 * deg2rad; //Shoulder Lift
	joint_group_positions_arm[2] = 110.62 * deg2rad; //Elbow
	joint_group_positions_arm[3] = 238.40 * deg2rad; //Wrist 1
	joint_group_positions_arm[4] = -92.08 * deg2rad; //Wrist 2
	//joint_group_positions_arm[5] = 0.00* deg2rad; //Wrist 3
	move_group_arm.setJointValueTarget(joint_group_positions_arm);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
	bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
	move_group_arm.execute(my_plan_arm);

	//Pregrasp
	RCLCPP_INFO(LOGGER, "Pregrasp Position");

	moveit_msgs::msg::OrientationConstraint ori_constraint;
	geometry_msgs::msg::Pose target_pose1;

	target_pose1.orientation.x = 0.9996;
	target_pose1.orientation.y = -0.0177;
	target_pose1.orientation.z = -0.0020;
	target_pose1.orientation.w = -0.0212;
	target_pose1.position.x = 0.176;
	target_pose1.position.y = -0.447;
	target_pose1.position.z = 0.312;

	ori_constraint.header.frame_id = move_group_arm.getPlanningFrame();
	ori_constraint.link_name = move_group_arm.getEndEffectorLink();

	// gewünschte Orientierung (z. B. aus target_pose1)
	ori_constraint.orientation = target_pose1.orientation;

	// Toleranzen (rad)
	ori_constraint.absolute_x_axis_tolerance = 0.015;
	ori_constraint.absolute_y_axis_tolerance = 0.015;
	ori_constraint.absolute_z_axis_tolerance = 0.015;

	ori_constraint.weight = 1.0;

	moveit_msgs::msg::Constraints path_constraints;
	path_constraints.orientation_constraints.push_back(ori_constraint);

	move_group_arm.setPathConstraints(path_constraints);

	move_group_arm.setPoseTarget(target_pose1);
	success_arm = (move_group_arm.plan(my_plan_arm) ==
	moveit::core::MoveItErrorCode::SUCCESS);
	move_group_arm.execute(my_plan_arm);


	RCLCPP_INFO(LOGGER, "Datei ausgelesen");
	// JSON-Datei laden
    std::ifstream file("/home/drawmemaybe/DrawMeMaybe/ros2_ws/paths.json");
    if (!file.is_open()) {
        std::cerr << "Fehler beim Öffnen der Datei!" << std::endl;
        return -1;
    }

	// JSON-Daten aus der Datei einlesen
    json data;
    file >> data;

	RCLCPP_INFO(LOGGER, "Start");
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose retreat_start = move_group_arm.getCurrentPose().pose;
	RCLCPP_INFO(LOGGER, "Position: x=%.3f, y=%.3f, z=%.3f", retreat_start.position.x, 
		retreat_start.position.y, retreat_start.position.z);
	RCLCPP_INFO(LOGGER, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", retreat_start.orientation.x, 
		retreat_start.orientation.y, retreat_start.orientation.z, retreat_start.orientation.w);

    geometry_msgs::msg::Pose p = retreat_start;

    //Startpunkt A
	double z_position = 0.03;
	
	RCLCPP_INFO(LOGGER, "Startpunkt");
    p.position.x -= 0.13;
    p.position.y += 0;
	p.position.z -= 0.05;
    waypoints.push_back(p);

	//ab hier die Einlesefunkition aus der Datei

	// Variable zum Speichern der Pfade
    std::vector<std::vector<std::pair<double, double>>> paths;
	move_group_arm.setPathConstraints(path_constraints);
	geometry_msgs::msg::Pose start = move_group_arm.getCurrentPose().pose;

	RCLCPP_INFO(LOGGER, "Position: x=%.3f, y=%.3f, z=%.3f", start.position.x, 
		start.position.y, start.position.z);
	RCLCPP_INFO(LOGGER, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", start.orientation.x, 
		start.orientation.y, start.orientation.z, start.orientation.w);

	geometry_msgs::msg::Pose target;

	bool first = true;
	bool first_point = true;

	// JSON-Daten durchlaufen und die Koordinaten extrahieren
	RCLCPP_INFO(LOGGER, "Linein");
    for (const auto& path : data["paths"]) {
        std::vector<std::pair<double, double>> coordinates;
		first = true;
		
        for (const auto& point : path) {
			target.orientation.x = 0.9996;
			target.orientation.y = -0.0177;
			target.orientation.z = -0.0020;
			target.orientation.w = -0.0212;
			target.position.x = start.position.x - (double)point[0]; // Invertiere X-Achse und offset
			target.position.y = start.position.y - (double)point[1]; // Invertiere Y-Achse und offset
			target.position.z = z_position; // Setze Z-Achse auf festen Wert

			if(first_point){
				RCLCPP_INFO(LOGGER, "Position: x=%.3f, y=%.3f, z=%.3f", target.position.x, 
					target.position.y, target.position.z);
				RCLCPP_INFO(LOGGER, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", target.orientation.x, 
					target.orientation.y, target.orientation.z, target.orientation.w);
				first_point = false;
			} 
			
			waypoints.push_back(target);

			if (first){
				target.position.z -= 0.02;
				first = false;
				
				waypoints.push_back(target);
			}
        }
        //paths.push_back(coordinates);
		target.position.z += 0.02;
    	waypoints.push_back(target);
    }
	
    // B
    p.position.x += 0;
    p.position.y -= 0.16;
    waypoints.push_back(p);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;
	const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_retreat);
    move_group_arm.execute(trajectory_retreat);
	rclcpp::shutdown();
	return 0;
}