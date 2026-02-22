#include "arms_moveit_demos/plan_demo.hpp"

namespace arms_moveit_demos
{

PlanDemo::PlanDemo(const std::string & node_name) : rclcpp::Node(node_name)
{
  RCLCPP_INFO(this->get_logger(), "Constructed.");

  plan_publisher_ =
    this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&PlanDemo::move_group_init, this));
}

void PlanDemo::declare_parameters()
{
  parameters_.target_position =
    this->declare_parameter<std::vector<double>>("target_position", {-0.282, -0.010, 0.5});
  parameters_.target_orientation =
    this->declare_parameter<std::vector<double>>("target_orientation", {-0.5, -0.5, 0.5, 0.5});
  parameters_.max_vel_scaling_factor =
    this->declare_parameter<double>("max_vel_scaling_factor", 0.5);
  parameters_.max_acc_scaling_factor =
    this->declare_parameter<double>("max_acc_scaling_factor", 0.5);

}

void PlanDemo::move_group_init()
{
  this->declare_parameters();

  move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");
  move_group_->setPlanningPipelineId("ompl");
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setMaxVelocityScalingFactor(parameters_.max_vel_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(parameters_.max_acc_scaling_factor);
  move_group_->setPoseTarget(this->get_target_pose());

  move_group_debug();

  moveit::planning_interface::MoveGroupInterface::Plan group_plan;
  auto const ok = static_cast<bool>(move_group_->plan(group_plan));
  auto const [success, plan] = std::make_pair(ok, group_plan);

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

  target_pose.pose.orientation.x = parameters_.target_orientation[0];
  target_pose.pose.orientation.y = parameters_.target_orientation[1];
  target_pose.pose.orientation.z = parameters_.target_orientation[2];
  target_pose.pose.orientation.w = parameters_.target_orientation[3];

  target_pose.pose.position.x = parameters_.target_position[0];
  target_pose.pose.position.y = parameters_.target_position[1];
  target_pose.pose.position.z = parameters_.target_position[2];

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
  auto node = std::make_shared<arms_moveit_demos::PlanDemo>("plan_demo_node");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}