#include "arms_moveit_demos/plan_demo.hpp"

namespace arms_moveit_demos
{

PlanDemo::PlanDemo(const rclcpp::NodeOptions & options) :
  rclcpp::Node("plan_demo", options),
  node_(std::make_shared<rclcpp::Node>("plan_demo_node")),
  // executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  RCLCPP_INFO(this->get_logger(), "Constructed.");

  declare_parameters();
  move_group_init();

  timer_ = this->create_wall_timer(500ms, std::bind(&PlanDemo::move_group_control, this));
}

void PlanDemo::declare_parameters()
{
  parameters_.arm_move_group_name =
    node_->declare_parameter<std::string>("arm_move_group_name", "arm");
  parameters_.target_position =
    node_->declare_parameter<std::vector<double>>("target_position", {-0.282, -0.010, 0.5});
  parameters_.target_orientation =
    node_->declare_parameter<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});
  parameters_.max_vel_scaling_factor =
    node_->declare_parameter<double>("max_vel_scaling_factor", 0.5);
  parameters_.max_acc_scaling_factor =
    node_->declare_parameter<double>("max_acc_scaling_factor", 0.5);
}

void PlanDemo::move_group_init()
{
  move_group_ = std::make_unique<MoveGroupInterface>(
    node_, parameters_.arm_move_group_name);
  executor_->add_node(node_);
  std::thread([this]() { executor_->spin(); }).detach();

  move_group_->setPlanningPipelineId("ompl");
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setMaxVelocityScalingFactor(parameters_.max_vel_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(parameters_.max_acc_scaling_factor);
  move_group_->setPoseTarget(this->get_target_pose());

  move_group_debug();

  move_group_visual();
}

void PlanDemo::move_group_visual()
{
  moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
    node_, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_->getRobotModel());
  moveit_visual_tools_->deleteAllMarkers();
  moveit_visual_tools_->loadRemoteControl();
}

void PlanDemo::move_group_control()
{
  moveit::planning_interface::MoveGroupInterface::Plan group_plan;
  auto const ok = static_cast<bool>(move_group_->plan(group_plan));
  auto const [success, plan] = std::make_pair(ok, group_plan);

  auto const joint_model_group =
    moveit_visual_tools_->getRobotModel()->getJointModelGroup(parameters_.arm_move_group_name);
  moveit_visual_tools_->publishTrajectoryLine(plan.trajectory, joint_model_group);
  moveit_visual_tools_->trigger();

  moveit_visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to execute the plan");
  moveit_visual_tools_->trigger();

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Executing path.");
    move_group_->execute(plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
  }

  // Only move the robot once.
  end_node();
}

void PlanDemo::move_group_debug()
{
  RCLCPP_INFO(this->get_logger(), "End effector: %s", move_group_->getEndEffectorLink().c_str());
  RCLCPP_INFO(this->get_logger(), "Planner ID: %s", move_group_->getPlannerId().c_str());
  RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "Planning Pipeline: %s", move_group_->getPlanningPipelineId().c_str());
  RCLCPP_INFO(this->get_logger(), "Planning Time: %.2f", move_group_->getPlanningTime());
  RCLCPP_INFO(this->get_logger(), "Acceleration scaling factor: %.2f", move_group_->getMaxVelocityScalingFactor());
  RCLCPP_INFO(this->get_logger(), "Velocity scaling factor: %.2f", move_group_->getMaxAccelerationScalingFactor());
}

const geometry_msgs::msg::PoseStamped PlanDemo::get_target_pose()
{
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = this->now();

  tf2::Quaternion orientation;
  orientation.setRPY(
    parameters_.target_orientation[0],
    parameters_.target_orientation[1],
    parameters_.target_orientation[2]);
  target_pose.pose.orientation = tf2::toMsg(orientation);

  target_pose.pose.position.x = parameters_.target_position[0];
  target_pose.pose.position.y = parameters_.target_position[1];
  target_pose.pose.position.z = parameters_.target_position[2];

  RCLCPP_INFO(this->get_logger(), "target_pose [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f, %.2f]",
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
    target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z,
    target_pose.pose.orientation.w);

  return target_pose;
}

void PlanDemo::end_node()
{
  timer_->cancel();
  rclcpp::shutdown();
}

}  // namespace arms_moveit_demos


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);

  auto node = std::make_shared<arms_moveit_demos::PlanDemo>(node_options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}