#include "arms_moveit_demos/pick_and_place_demo.hpp"

namespace arms_moveit_demos
{

PickAndPlaceDemo::PickAndPlaceDemo(const rclcpp::NodeOptions & options) :
  rclcpp::Node("pick_and_place_demo", options),
  node_(std::make_shared<rclcpp::Node>("pick_and_place_demo_node")),
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  RCLCPP_INFO(this->get_logger(), "Constructed.");

  declare_parameters();
  move_group_init();

  attacher_pub_ = gz_node_.Advertise<gz::msgs::StringMsg>("/attach");

  pick_and_place();
}


void PickAndPlaceDemo::declare_parameters()
{
  parameters_.arm_move_group_name =
    node_->declare_parameter<std::string>("arm_move_group_name", "arm");
  parameters_.gripper_move_group_name =
    node_->declare_parameter<std::string>("gripper_move_group_name", "gripper");
  parameters_.init_pose =
    node_->declare_parameter<std::string>("init_pose", "zero");
  parameters_.target_position =
    node_->declare_parameter<std::vector<double>>("target_position", {-0.282, -0.010, 0.5});
  parameters_.target_orientation =
    node_->declare_parameter<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});
  parameters_.place_pose =
    node_->declare_parameter<std::string>("place_pose", "place");
  parameters_.max_vel_scaling_factor =
    node_->declare_parameter<double>("max_vel_scaling_factor", 0.5);
  parameters_.max_acc_scaling_factor =
    node_->declare_parameter<double>("max_acc_scaling_factor", 0.5);
  parameters_.sim_attach =
    node_->declare_parameter<bool>("sim_attach", true);
}

void PickAndPlaceDemo::move_group_init()
{
  // ARM MOVE GROUP INTERFACE
  arm_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    node_, parameters_.arm_move_group_name);

  arm_move_group_->setPlanningPipelineId("ompl");
  arm_move_group_->setPlannerId("RRTConnectkConfigDefault");
  arm_move_group_->setMaxVelocityScalingFactor(parameters_.max_vel_scaling_factor);
  arm_move_group_->setMaxAccelerationScalingFactor(parameters_.max_acc_scaling_factor);

  // GRIPPER MOVE GROUP INTERFACE
  gripper_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    node_, parameters_.gripper_move_group_name);

  // gripper_move_group_->setPlanningPipelineId("ompl");
  // gripper_move_group_->setPlannerId("RRTConnectkConfigDefault");
  // gripper_move_group_->setMaxVelocityScalingFactor(parameters_.max_vel_scaling_factor);
  // gripper_move_group_->setMaxAccelerationScalingFactor(parameters_.max_acc_scaling_factor);
  // gripper_move_group_->setPoseTarget(this->get_target_pose());

  executor_->add_node(node_);
  std::thread([this]() { executor_->spin(); }).detach();

  update_collision_scene();
}

const geometry_msgs::msg::PoseStamped PickAndPlaceDemo::get_target_pose()
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

void PickAndPlaceDemo::open_gripper()
{
  gripper_move_group_->setNamedTarget("open");
  gripper_move_group_->move();
}

void PickAndPlaceDemo::close_gripper()
{
  gripper_move_group_->setNamedTarget("half-close");
  gripper_move_group_->move();
}

void PickAndPlaceDemo::attach_gripper()
{
  std::vector<std::string> touch_links = {
    "right_finger_bottom_link",
    "right_finger_dist_link",
    "left_finger_bottom_link",
    "left_finger_dist_link",
    "<octomap>"};

  // gripper_move_group_->attachObject("apple", "tool_frame", touch_links);
  arm_move_group_->attachObject("apple", "end_effector_link", touch_links);
  if (parameters_.sim_attach) {
    gz::msgs::StringMsg msg;
    msg.set_data("[gen3_lite][end_effector_link][Apple_4][apple_link][attach]");
    attacher_pub_.Publish(msg);
  }
  close_gripper();
}

void PickAndPlaceDemo::detach_gripper()
{
  arm_move_group_->detachObject("apple");
  planning_scene_interface_.removeCollisionObjects({"apple"});
  if (parameters_.sim_attach) {
    gz::msgs::StringMsg msg;
    msg.set_data("[gen3_lite][end_effector_link][Apple_4][apple_link][detach]");
    attacher_pub_.Publish(msg);
  }
  open_gripper();
}

void PickAndPlaceDemo::update_collision_scene()
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Ground plane
  collision_objects[0].id = "ground_plane";
  collision_objects[0].header.frame_id = "base_link";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.0;
  collision_objects[0].primitives[0].dimensions[1] = 1.0;
  collision_objects[0].primitives[0].dimensions[2] = 0.001;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = -0.001;
  // Add ground plane to the scene
  collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

  // Apple
  collision_objects[1].id = "apple";
  collision_objects[1].header.frame_id = "base_link";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  collision_objects[1].primitives[0].dimensions.resize(1);
  collision_objects[1].primitives[0].dimensions[0] = 0.03;

  collision_objects[1].primitive_poses.resize(1);
  auto const target_pose = get_target_pose();
  collision_objects[1].primitive_poses[0].position = target_pose.pose.position;
  collision_objects[1].primitive_poses[0].orientation = target_pose.pose.orientation;

  // Add apple to the scene
  collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  RCLCPP_INFO(this->get_logger(), "Collision objects added to the scene.");
}

void PickAndPlaceDemo::pick_and_place()
{
  RCLCPP_INFO(this->get_logger(), "Initializing pick and place task.");
  // Open gripper
  open_gripper();

  // Moving the arm to zero pose
  arm_move_group_->setNamedTarget(parameters_.init_pose);
  {
    moveit::planning_interface::MoveGroupInterface::Plan group_plan;
    auto const ok = static_cast<bool>(arm_move_group_->plan(group_plan));
    auto const [success, plan] = std::make_pair(ok, group_plan);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Going to zero pose.");
      arm_move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning to zero pose failed!");
      return;
    }
  }

  // Moving the arm to the target pose
  arm_move_group_->setPoseTarget(get_target_pose());
  {
    moveit::planning_interface::MoveGroupInterface::Plan group_plan;
    auto const ok = static_cast<bool>(arm_move_group_->plan(group_plan));
    auto const [success, plan] = std::make_pair(ok, group_plan);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing path to target.");
      arm_move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning to target failed!");
      return;
    }
  }

  // Attaching the object
  RCLCPP_INFO(this->get_logger(), "Attaching the object.");
  attach_gripper();

  // // Moving the arm to place pose
  arm_move_group_->setNamedTarget(parameters_.place_pose);
  {
    moveit::planning_interface::MoveGroupInterface::Plan group_plan;
    auto const ok = static_cast<bool>(arm_move_group_->plan(group_plan));
    auto const [success, plan] = std::make_pair(ok, group_plan);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Going to zero pose.");
      arm_move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning to zero pose failed!");
      return;
    }
  }

  // Detaching the object
  RCLCPP_INFO(this->get_logger(), "Detaching the object.");
  detach_gripper();

  RCLCPP_INFO(this->get_logger(), "Pick and place completed.");
}

}  // namespace arms_moveit_demos


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);

  auto node = std::make_shared<arms_moveit_demos::PickAndPlaceDemo>(node_options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}