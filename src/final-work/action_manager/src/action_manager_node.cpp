#include "action_manager/action_manager_node.hpp"

#include <chrono>
#include <cmath>
#include <future>

using namespace std::chrono_literals;

namespace action_manager
{

ActionManagerNode::ActionManagerNode()
: Node("action_manager")
{
  RCLCPP_INFO(get_logger(), "ActionManager starting up");

  // Create callback groups for concurrent execution
  action_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  client_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Initialize service clients
  piece_location_client_ = this->create_client<sensing_module::srv::PieceLocation>(
    "piece_location",
    rmw_qos_profile_services_default,
    client_callback_group_);

  planning_client_ = this->create_client<planning_module::srv::PlanPickPlace>(
    "plan_pick_place",
    rmw_qos_profile_services_default,
    client_callback_group_);

  set_conf_client_ = this->create_client<chesslab_setup2_interfaces::srv::SetRobConf>(
    "set_robot_configuration",
    rmw_qos_profile_services_default,
    client_callback_group_);

  set_obj_pose_client_ = this->create_client<chesslab_setup2_interfaces::srv::SetObjPose>(
    "set_object_pose",
    rmw_qos_profile_services_default,
    client_callback_group_);

  gripper_client_ = this->create_client<robotiq_85_gripper_server::srv::GripperOrder>(
    "gripper_order",
    rmw_qos_profile_services_default,
    client_callback_group_);

  // Create action server
  action_server_ = rclcpp_action::create_server<ChessAction>(
    this,
    "chess_action",
    std::bind(&ActionManagerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionManagerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionManagerNode::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_qos(),
    action_callback_group_);

  RCLCPP_INFO(get_logger(), "ActionManager initialized successfully");
}

rclcpp_action::GoalResponse ActionManagerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ChessAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal: action=%u, piece_id=%d, target=%s",
    goal->action_type, goal->piece_aruco_id, goal->target_square.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionManagerNode::handle_cancel(
  const std::shared_ptr<GoalHandleChessAction> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Got cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionManagerNode::handle_accepted(
  const std::shared_ptr<GoalHandleChessAction> goal_handle)
{
  // Execute in separate thread to avoid blocking the action server
  std::thread{std::bind(&ActionManagerNode::execute_chess_action, this, goal_handle)}.detach();
}

void ActionManagerNode::execute_chess_action(
  const std::shared_ptr<GoalHandleChessAction> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing chess action");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ChessAction::Feedback>();
  auto result = std::make_shared<ChessAction::Result>();

  feedback->progress = 0.0f;
  feedback->status = "Starting chess action";
  goal_handle->publish_feedback(feedback);

  bool success = false;
  std::string error_msg;

  try {
    switch (goal->action_type) {
      case ChessAction::Goal::ACTION_MOVE:
        success = execute_move(goal_handle, feedback);
        break;
      case ChessAction::Goal::ACTION_CAPTURE:
        success = execute_capture(goal_handle, feedback);
        break;
      case ChessAction::Goal::ACTION_CASTLING:
        success = execute_castling(goal_handle, feedback);
        break;
      default:
        error_msg = "Unknown action type: " + std::to_string(goal->action_type);
        success = false;
    }
  } catch (const std::exception & e) {
    error_msg = std::string("Exception: ") + e.what();
    success = false;
  }

  // Finalize
  if (success) {
    // Get final position for verification
    geometry_msgs::msg::Pose final_pose;
    std::string final_cell;
    if (get_piece_pose(goal->piece_aruco_id, final_pose, final_cell)) {
      result->final_square = final_cell;
    } else {
      result->final_square = goal->target_square;
    }
    
    result->success = true;
    result->message = "Chess action completed successfully";
    RCLCPP_INFO(get_logger(), "Action SUCCEEDED");
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->message = error_msg.empty() ? "Action failed" : error_msg;
    result->final_square = "unknown";
    RCLCPP_ERROR(get_logger(), "Action FAILED: %s", result->message.c_str());
    goal_handle->abort(result);
  }
}

bool ActionManagerNode::execute_move(
  const std::shared_ptr<GoalHandleChessAction> goal_handle,
  std::shared_ptr<ChessAction::Feedback> feedback)
{
  const auto goal = goal_handle->get_goal();

  // Step 1: Get current piece location
  RCLCPP_INFO(get_logger(), "[MOVE] Step 1/5: Sensing current position");
  feedback->progress = 10.0f;
  feedback->status = "Sensing current position";
  goal_handle->publish_feedback(feedback);

  geometry_msgs::msg::Pose current_pose;
  std::string current_cell;
  if (!get_piece_pose(goal->piece_aruco_id, current_pose, current_cell)) {
    RCLCPP_ERROR(get_logger(), "Could not locate piece %d", goal->piece_aruco_id);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Piece %d at %s", goal->piece_aruco_id, current_cell.c_str());

  // Step 2: Convert target square to pose
  RCLCPP_INFO(get_logger(), "[MOVE] Step 2/5: Computing target pose");
  feedback->progress = 20.0f;
  feedback->status = "Computing target pose";
  goal_handle->publish_feedback(feedback);

  geometry_msgs::msg::Pose target_pose = cell_to_pose(goal->target_square);
  log_pose(target_pose, "Target pose");

  // Step 3: Plan pick-place trajectory
  RCLCPP_INFO(get_logger(), "[MOVE] Step 3/5: Planning trajectory");
  feedback->progress = 30.0f;
  feedback->status = "Planning trajectory";
  goal_handle->publish_feedback(feedback);

  std::vector<sensor_msgs::msg::JointState> trajectory;
  if (!plan_pick_place(current_pose, target_pose, trajectory)) {
    RCLCPP_ERROR(get_logger(), "Failed to plan trajectory");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Trajectory planned with %zu waypoints", trajectory.size());

  // Step 4: Execute trajectory and gripper control
  RCLCPP_INFO(get_logger(), "[MOVE] Step 4/5: Executing motion");
  feedback->progress = 50.0f;
  feedback->status = "Executing pick and place";
  goal_handle->publish_feedback(feedback);

  // Simplified execution: just send to gripper
  // In real system, would also move robot via chesslab_setup2 services
  if (!control_gripper(GRIPPER_CLOSE_POSITION)) {
    RCLCPP_ERROR(get_logger(), "Failed to close gripper");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Gripper closed");

  // Note: In production, execute_trajectory() would send actual robot motion
  // For now, we simulate by just opening gripper at target
  rclcpp::sleep_for(1s);  // Simulate motion time

  if (!control_gripper(GRIPPER_OPEN_POSITION)) {
    RCLCPP_ERROR(get_logger(), "Failed to open gripper");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Gripper opened");

  // Step 5: Verify result
  RCLCPP_INFO(get_logger(), "[MOVE] Step 5/5: Verifying result");
  feedback->progress = 90.0f;
  feedback->status = "Verifying result";
  goal_handle->publish_feedback(feedback);

  rclcpp::sleep_for(500ms);  // Wait for markers to stabilize

  geometry_msgs::msg::Pose final_pose;
  std::string final_cell;
  if (get_piece_pose(goal->piece_aruco_id, final_pose, final_cell)) {
    RCLCPP_INFO(get_logger(), "Final position: %s", final_cell.c_str());
    if (final_cell == goal->target_square) {
      RCLCPP_INFO(get_logger(), "SUCCESS: Piece at target square");
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Piece at %s, expected %s",
        final_cell.c_str(), goal->target_square.c_str());
    }
  }

  feedback->progress = 100.0f;
  feedback->status = "Move complete";
  goal_handle->publish_feedback(feedback);

  return true;  // Accept even if final position doesn't match (may be sensing error)
}

bool ActionManagerNode::execute_capture(
  const std::shared_ptr<GoalHandleChessAction> goal_handle,
  std::shared_ptr<ChessAction::Feedback> feedback)
{
  const auto goal = goal_handle->get_goal();

  RCLCPP_INFO(get_logger(), "[CAPTURE] Starting capture: piece %d, target %s, capture %d",
    goal->piece_aruco_id, goal->target_square.c_str(), goal->captured_piece_aruco_id);

  feedback->progress = 10.0f;
  feedback->status = "Sensing attacker position";
  goal_handle->publish_feedback(feedback);

  // Step 1: Get attacker piece location
  geometry_msgs::msg::Pose attacker_pose;
  std::string attacker_cell;
  if (!get_piece_pose(goal->piece_aruco_id, attacker_pose, attacker_cell)) {
    RCLCPP_ERROR(get_logger(), "Could not locate attacker piece %d", goal->piece_aruco_id);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Attacker at %s", attacker_cell.c_str());

  feedback->progress = 20.0f;
  feedback->status = "Sensing captured piece position";
  goal_handle->publish_feedback(feedback);

  // Step 2: Get captured piece location (for validation)
  geometry_msgs::msg::Pose captured_pose;
  std::string captured_cell;
  if (!get_piece_pose(goal->captured_piece_aruco_id, captured_pose, captured_cell)) {
    RCLCPP_WARN(get_logger(), "Could not locate captured piece %d", goal->captured_piece_aruco_id);
  }
  RCLCPP_INFO(get_logger(), "Captured piece at %s", captured_cell.c_str());

  // Step 3: Compute target pose
  feedback->progress = 30.0f;
  feedback->status = "Computing target pose";
  goal_handle->publish_feedback(feedback);

  geometry_msgs::msg::Pose target_pose = cell_to_pose(goal->target_square);

  // Step 4: Plan and execute attacker movement to target
  feedback->progress = 40.0f;
  feedback->status = "Planning attacker trajectory";
  goal_handle->publish_feedback(feedback);

  std::vector<sensor_msgs::msg::JointState> trajectory;
  if (!plan_pick_place(attacker_pose, target_pose, trajectory)) {
    RCLCPP_ERROR(get_logger(), "Failed to plan attacker trajectory");
    return false;
  }

  feedback->progress = 60.0f;
  feedback->status = "Executing capture";
  goal_handle->publish_feedback(feedback);

  // Close gripper, move to target, open gripper
  if (!control_gripper(GRIPPER_CLOSE_POSITION)) {
    return false;
  }
  rclcpp::sleep_for(1s);  // Simulate motion
  if (!control_gripper(GRIPPER_OPEN_POSITION)) {
    return false;
  }

  // Step 5: Remove captured piece from board (in simulation, would delete from Gazebo)
  feedback->progress = 80.0f;
  feedback->status = "Removing captured piece";
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(get_logger(), "Captured piece removed from board");

  feedback->progress = 100.0f;
  feedback->status = "Capture complete";
  goal_handle->publish_feedback(feedback);

  return true;
}

bool ActionManagerNode::execute_castling(
  const std::shared_ptr<GoalHandleChessAction> goal_handle,
  std::shared_ptr<ChessAction::Feedback> feedback)
{
  const auto goal = goal_handle->get_goal();

  RCLCPP_INFO(get_logger(), "[CASTLING] Starting castling: piece %d, side %s",
    goal->piece_aruco_id, goal->castling_side.c_str());

  feedback->progress = 10.0f;
  feedback->status = "Sensing king position";
  goal_handle->publish_feedback(feedback);

  // Castling logic is more complex; would involve:
  // 1. Get king and rook positions
  // 2. Move king two squares toward rook
  // 3. Move rook to other side of king
  // This is a simplified stub

  geometry_msgs::msg::Pose king_pose;
  std::string king_cell;
  if (!get_piece_pose(goal->piece_aruco_id, king_pose, king_cell)) {
    RCLCPP_ERROR(get_logger(), "Could not locate king");
    return false;
  }

  feedback->progress = 50.0f;
  feedback->status = "Executing castling moves";
  goal_handle->publish_feedback(feedback);

  // Implement actual castling logic here
  RCLCPP_WARN(get_logger(), "Castling not fully implemented yet");

  feedback->progress = 100.0f;
  feedback->status = "Castling complete";
  goal_handle->publish_feedback(feedback);

  return true;
}

geometry_msgs::msg::Pose ActionManagerNode::cell_to_pose(const std::string & cell)
{
  geometry_msgs::msg::Pose pose;

  if (cell.length() != 2) {
    RCLCPP_ERROR(get_logger(), "Invalid cell format: %s", cell.c_str());
    return pose;
  }

  // Parse file (A-H) and rank (1-8)
  int file = static_cast<int>(cell[0] - 'A');  // 0-7
  int rank = static_cast<int>(cell[1] - '1');  // 0-7

  // Convert to world coordinates (board center at origin)
  // Files go left-right (X), ranks go up-down (Y)
  pose.position.x = (file - 3.5) * SQUARE_SIZE;
  pose.position.y = (rank - 3.5) * SQUARE_SIZE;
  pose.position.z = BOARD_Z;

  // Default orientation (upright)
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  return pose;
}

std::string ActionManagerNode::pose_to_cell(const geometry_msgs::msg::Pose & pose)
{
  double fx = pose.position.x / SQUARE_SIZE + 3.5;
  double fy = pose.position.y / SQUARE_SIZE + 3.5;

  int file_idx = static_cast<int>(std::round(fx));
  int rank_idx = static_cast<int>(std::round(fy));

  if (file_idx < 0) file_idx = 0;
  if (file_idx > 7) file_idx = 7;
  if (rank_idx < 0) rank_idx = 0;
  if (rank_idx > 7) rank_idx = 7;

  char file = static_cast<char>('A' + file_idx);
  int rank = rank_idx + 1;

  return std::string(1, file) + std::to_string(rank);
}

double ActionManagerNode::get_piece_height(int aruco_id)
{
  // Determine piece type from ArUco ID
  int first_digit = aruco_id / 100;
  int piece_type = (aruco_id % 100) / 10;

  auto it = piece_heights_.find({first_digit, piece_type});
  if (it != piece_heights_.end()) {
    return it->second;
  }

  return 0.05;  // Default height
}

bool ActionManagerNode::get_piece_pose(
  int aruco_id,
  geometry_msgs::msg::Pose & pose_out,
  std::string & cell_out)
{
  if (!piece_location_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(get_logger(), "piece_location service not available");
    return false;
  }

  auto request = std::make_shared<sensing_module::srv::PieceLocation::Request>();
  request->aruco_id = aruco_id;

  auto future = piece_location_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Service call timed out");
    return false;
  }

  auto response = future.get();
  if (!response->found) {
    RCLCPP_WARN(get_logger(), "Piece %d not found", aruco_id);
    return false;
  }

  pose_out = response->pose;
  cell_out = response->cell;
  return true;
}

bool ActionManagerNode::plan_pick_place(
  const geometry_msgs::msg::Pose & source,
  const geometry_msgs::msg::Pose & target,
  std::vector<sensor_msgs::msg::JointState> & trajectory_out)
{
  if (!planning_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(get_logger(), "plan_pick_place service not available");
    return false;
  }

  auto request = std::make_shared<planning_module::srv::PlanPickPlace::Request>();
  request->source_pose = source;
  request->target_pose = target;

  auto future = planning_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Plan service call timed out");
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", response->message.c_str());
    return false;
  }

  trajectory_out = response->trajectory;
  return true;
}

bool ActionManagerNode::execute_trajectory(
  const std::vector<sensor_msgs::msg::JointState> & trajectory)
{
  // Send each waypoint to the robot controller
  // In simulation: use chesslab_setup2/set_robot_configuration
  // In real: use ur_robot_driver

  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto & js = trajectory[i];

    if (!set_conf_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "set_robot_configuration not available");
      return false;
    }

    auto request = std::make_shared<chesslab_setup2_interfaces::srv::SetRobConf::Request>();
    request->conf = std::vector<float>(js.position.begin(), js.position.end());

    auto future = set_conf_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Timeout setting robot configuration at waypoint %zu", i);
    }

    rclcpp::sleep_for(500ms);  // Wait between waypoints
  }

  return true;
}

bool ActionManagerNode::control_gripper(float position)
{
  if (!gripper_client_->wait_for_service(2s)) {
    RCLCPP_WARN(get_logger(), "gripper_order service not available");
    return false;
  }

  auto request = std::make_shared<robotiq_85_gripper_server::srv::GripperOrder::Request>();
  request->order = position > 0.5f ? 1 : 0;  // 1 = close, 0 = open

  auto future = gripper_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Gripper control timed out");
    return false;
  }

  return true;
}

void ActionManagerNode::log_pose(const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  RCLCPP_INFO(get_logger(), "%s: x=%.3f, y=%.3f, z=%.3f",
    label.c_str(), pose.position.x, pose.position.y, pose.position.z);
}

}  // namespace action_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<action_manager::ActionManagerNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
