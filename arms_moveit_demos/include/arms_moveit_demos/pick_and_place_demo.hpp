#ifndef ARMS_MOVEIT_DEMOS__PICK_AND_PLACE_DEMO_HPP_
#define ARMS_MOVEIT_DEMOS__PICK_AND_PLACE_DEMO_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gz/transport/Node.hh"
#include "gz/msgs/stringmsg.pb.h"
#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace arms_moveit_demos
{

struct Parameters
{
  std::string arm_move_group_name, gripper_move_group_name;
  std::string base_link;
  std::string init_pose;
  std::vector<double> target_position, target_orientation;
  std::string place_pose;
  double max_vel_scaling_factor, max_acc_scaling_factor;
  bool sim_attach;
  std::string last_arm_link, target_name, target_link;
};

class PickAndPlaceDemo : public rclcpp::Node
{
public:
  explicit PickAndPlaceDemo(const rclcpp::NodeOptions & options);

  ~PickAndPlaceDemo() = default;

private:
  void declare_parameters();

  void move_group_init();

  void get_robot_info();

  const geometry_msgs::msg::PoseStamped get_target_pose();

  void open_gripper();

  void close_gripper();

  void attach_gripper();

  void detach_gripper();

  void update_collision_scene();

  void pick_and_place();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher attacher_pub_;

  std::string robot_name_;
  std::string end_effector_link_;

  Parameters parameters_;
};

}  // namespace arms_moveit_demos

#endif  // ARMS_MOVEIT_DEMOS__PICK_AND_PLACE_DEMO_HPP_