#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <fstream>
#include <nlohmann/json.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// TF
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("drawing_node");
static const std::string PLANNING_GROUP = "ur_manipulator";

// --------------------------------------------------
// Hilfsfunktion: Cartesian Path mit Geschwindigkeitskontrolle
// --------------------------------------------------
bool executeCartesianPath(
moveit::planning_interface::MoveGroupInterface &move_group,
const std::vector<geometry_msgs::msg::Pose> &waypoints,
double velocity_scaling = 0.2,
double acceleration_scaling = 0.2)
{
moveit_msgs::msg::RobotTrajectory trajectory;
const double eef_step = 0.005;
const double jump_threshold = 0.0;

double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

if (fraction < 0.99)
{
    RCLCPP_WARN(LOGGER,
        "Cartesian Path nur %.1f %% geplant", fraction * 100.0);
}

robot_trajectory::RobotTrajectory rt(
    move_group.getCurrentState()->getRobotModel(),
    PLANNING_GROUP);

rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

trajectory_processing::IterativeParabolicTimeParameterization iptp;
if (!iptp.computeTimeStamps(rt, velocity_scaling, acceleration_scaling))
{
    RCLCPP_ERROR(LOGGER, "Zeitparametrisierung fehlgeschlagen");
    return false;
}

rt.getRobotTrajectoryMsg(trajectory);
move_group.execute(trajectory);
return true;

}

// --------------------------------------------------
// MAIN
// --------------------------------------------------
int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);

rclcpp::NodeOptions node_options;
node_options.automatically_declare_parameters_from_overrides(true);

auto node = rclcpp::Node::make_shared("drawing_node", node_options);

rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread([&executor]() { executor.spin(); }).detach();

// --------------------------------------------------
// MoveGroup
// --------------------------------------------------
moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
move_group.setMaxVelocityScalingFactor(0.2);
move_group.setMaxAccelerationScalingFactor(0.2);

// --------------------------------------------------
// Temporärer Zeichen-Frame
// --------------------------------------------------
tf2_ros::StaticTransformBroadcaster tf_broadcaster(node);

geometry_msgs::msg::TransformStamped drawing_tf;
drawing_tf.header.stamp = node->now();
drawing_tf.header.frame_id = "base_link";
drawing_tf.child_frame_id = "drawing_frame";

drawing_tf.transform.translation.x = 0.25;
drawing_tf.transform.translation.y = -0.25;
drawing_tf.transform.translation.z = 0.30;

tf2::Quaternion q;
q.setRPY(M_PI, 0, 0);   // Tool senkrecht zur Fläche
drawing_tf.transform.rotation = tf2::toMsg(q);

tf_broadcaster.sendTransform(drawing_tf);
rclcpp::sleep_for(std::chrono::milliseconds(300));

move_group.setPoseReferenceFrame("drawing_frame");

// --------------------------------------------------
// HOME
// --------------------------------------------------
RCLCPP_INFO(LOGGER, "Going Home");

std::vector<double> home = {
    -76.25 * M_PI / 180.0,
    -81.76 * M_PI / 180.0,
    110.62 * M_PI / 180.0,
    238.40 * M_PI / 180.0,
    -92.08 * M_PI / 180.0,
    0.0
};
moveit_msgs::msg::OrientationConstraint ori_constraint;

ori_constraint.header.frame_id = move_group.getPlanningFrame();
ori_constraint.link_name = move_group.getEndEffectorLink();
geometry_msgs::msg::Pose target_pose1;

	target_pose1.orientation.x = 0.9996;
	target_pose1.orientation.y = -0.0177;
	target_pose1.orientation.z = -0.0020;
	target_pose1.orientation.w = -0.0212;
	target_pose1.position.x = 0.176;
	target_pose1.position.y = -0.447;
	target_pose1.position.z = 0.312;

// gewünschte Orientierung (z. B. aus target_pose1)
ori_constraint.orientation = target_pose1.orientation;

// Toleranzen (rad)
ori_constraint.absolute_x_axis_tolerance = 0.015;
ori_constraint.absolute_y_axis_tolerance = 0.015;
ori_constraint.absolute_z_axis_tolerance = 0.015;

ori_constraint.weight = 1.0;

moveit_msgs::msg::Constraints path_constraints;
path_constraints.orientation_constraints.push_back(ori_constraint);

move_group.setPathConstraints(path_constraints);

move_group.setJointValueTarget(home);
moveit::planning_interface::MoveGroupInterface::Plan plan;
move_group.plan(plan);
move_group.execute(plan);

// --------------------------------------------------
// PREGRASP (über Zeichenfläche)
// --------------------------------------------------
RCLCPP_INFO(LOGGER, "Pregrasp");

geometry_msgs::msg::Pose pregrasp;
pregrasp.orientation.w = 1.0;
pregrasp.position.x = 0.0;
pregrasp.position.y = 0.0;
pregrasp.position.z = 0.15;

move_group.setPoseTarget(pregrasp);
move_group.plan(plan);
move_group.execute(plan);

// --------------------------------------------------
// APPROACH (langsam nach unten)
// --------------------------------------------------
RCLCPP_INFO(LOGGER, "Approach");

std::vector<geometry_msgs::msg::Pose> approach;
geometry_msgs::msg::Pose p = pregrasp;

p.position.z -= 0.05;
approach.push_back(p);
RCLCPP_INFO(LOGGER, "Approach");
p.position.z -= 0.05;
approach.push_back(p);

executeCartesianPath(move_group, approach, 0.2, 0.2);

// JSON-Datei laden
std::ifstream file("/home/drawmemaybe/DrawMeMaybe/ros2_ws/paths.json");
if (!file.is_open()) {
    std::cerr << "Fehler beim Öffnen der Datei!" << std::endl;
    return -1;
}

// JSON-Daten aus der Datei einlesen
json data;
file >> data;

// --------------------------------------------------
// ZEICHNUNG (Quadrat)
// --------------------------------------------------
RCLCPP_INFO(LOGGER, "Drawing");
bool first = true;

std::vector<geometry_msgs::msg::Pose> drawing;
geometry_msgs::msg::Pose d = p;

for (const auto& path : data["paths"]) {
    std::vector<std::pair<double, double>> coordinates;
    first = true;
    
    for (const auto& point : path) {
        d.position.x =  -(double)point[0]; // Invertiere X-Achse und offset
        d.position.y = (double)point[1]; // Invertiere Y-Achse und offset
/*
        if(first_point){
            RCLCPP_INFO(LOGGER, "Position: x=%.3f, y=%.3f, z=%.3f", target.position.x, 
                target.position.y, target.position.z);
            RCLCPP_INFO(LOGGER, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", target.orientation.x, 
                target.orientation.y, target.orientation.z, target.orientation.w);
            first_point = false;
        } */

        drawing.push_back(d);

        if (first){
            d.position.z = 0;
            first = false;
            
            drawing.push_back(d);
        }
    }
    //paths.push_back(coordinates);
    d.position.z = 0.02;
    drawing.push_back(d);
}
/*
d.position.x = 0.00; d.position.y = 0.00;
drawing.push_back(d);

d.position.x = 0.10;
drawing.push_back(d);

d.position.y = 0.10;
drawing.push_back(d);

d.position.x = 0.00;
drawing.push_back(d);

d.position.y = 0.00;
drawing.push_back(d);

// B
RCLCPP_INFO(LOGGER, "Das");
    d.position.x = 0;
    d.position.y = 0.16;
    drawing.push_back(d);
RCLCPP_INFO(LOGGER, "Haus");
    // D
    d.position.x = -0.16;
    d.position.y = 0.16;
    drawing.push_back(d);
RCLCPP_INFO(LOGGER, "das");
    // A
    d.position.x = 0;
    d.position.y = 0;
    drawing.push_back(d);
RCLCPP_INFO(LOGGER, "Ni-");
    // E
    d.position.x = -0.16;
    d.position.y = 0;
    drawing.push_back(d);
RCLCPP_INFO(LOGGER, "ko");
    // B
    d.position.x = 0;
    d.position.y = 0.16;
    drawing.push_back(d);
RCLCPP_INFO(LOGGER, "laus");

    // D
    d.position.x = -0.08;
    d.position.y = 0.24;
    drawing.push_back(d);

    RCLCPP_INFO(LOGGER, "laus");

    // D
    d.position.x = -0.16;
    d.position.y = 0.16;
    drawing.push_back(d);

    RCLCPP_INFO(LOGGER, "laus");

    // D
    d.position.x = -0.16;
    d.position.y = 0;
    drawing.push_back(d);*/


executeCartesianPath(move_group, drawing, 0.2, 0.2);

rclcpp::shutdown();
return 0;
}