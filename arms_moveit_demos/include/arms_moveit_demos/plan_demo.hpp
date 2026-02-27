#ifndef ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_
#define ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace arms_moveit_demos
{

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

struct Parameters
{
  std::string arm_move_group_name;
  std::vector<double> target_position;
  std::vector<double> target_orientation;
  double max_vel_scaling_factor;
  double max_acc_scaling_factor;
};

class PlanDemo : public rclcpp::Node
{
public:
  explicit PlanDemo(const rclcpp::NodeOptions & options);

  ~PlanDemo() = default;

private:
  void declare_parameters();

  void move_group_init();

  void move_group_visual();

  const geometry_msgs::msg::PoseStamped get_target_pose();

  void move_group_debug();

  void move_group_control();

  void end_node();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MoveGroupInterface> move_group_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

  Parameters parameters_;
};

}  // namespace arms_moveit_demos

#endif  // ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_