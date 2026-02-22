#ifndef ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_
#define ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arms_moveit_demos
{

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using namespace std::chrono_literals;

struct Parameters
{
  std::vector<double> target_position;
  std::vector<double> target_orientation;
  double max_vel_scaling_factor;
  double max_acc_scaling_factor;
};

class PlanDemo : public rclcpp::Node
{
public:
  explicit PlanDemo(const std::string & node_name);

  ~PlanDemo() = default;

private:
  void declare_parameters();

  void move_group_init();

  void move_group_debug();

  const geometry_msgs::msg::PoseStamped get_target_pose();

  void end_node();

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr plan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MoveGroupInterface> move_group_;
  PlanningSceneInterface planning_scene_interface_;

  Parameters parameters_;
};

}  // namespace arms_moveit_demos

#endif  // ARMS_MOVEIT_DEMOS__PLAN_DEMO_HPP_