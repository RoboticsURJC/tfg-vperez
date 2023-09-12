
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex_move_A_to_B", node_options);
  node->declare_parameter("planning_plugin", "ompl_interface/OMPLPlanner");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
  // a RobotModel and a PlanningScene.
  //
  // We will start by instantiating a
  // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
  // object, which will look up the robot description on the ROS
  // parameter server and construct a
  // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
  // for us to use.
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  // Using the RobotModelLoader, we can construct a planning scene monitor that
  // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
  // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
  // startStateMonitor to fully initialize the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));

  /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                       the internal planning scene accordingly */
  psm->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                                world geometry, collision objects and optionally octomaps */
  psm->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision objects
                        and update the internal planning scene accordingly*/
  psm->startStateMonitor();

  /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // We can now setup the PlanningPipeline object, which will use the ROS parameter server
  // to determine the set of request adapters and the planning plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node, "", "planning_plugin", "request_adapters"));

 
  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.15;
  pose.pose.position.y = 0.15;
  pose.pose.position.z = 0.15;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper
  // function available from the
  // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
  // package.
  req.group_name = "arm";
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("end_effector_tip", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Now, call the pipeline and check whether planning was successful. */
  /* Check that the planning was successful */
  if (res.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    return 0;
  }

  RCLCPP_INFO(LOGGER, "Done");

  rclcpp::shutdown();
  return 0;
}