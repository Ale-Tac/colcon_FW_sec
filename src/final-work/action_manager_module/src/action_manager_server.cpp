#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <future>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "action_manager_module_interfaces/action/execute_chess_action.hpp"
#include "sensing_module/srv/piece_location.hpp"
#include "planning_module/srv/plan_pick_place.hpp"

#include "robotiq_85_gripper_server/srv/gripper_open.hpp"
#include "robotiq_85_gripper_server/srv/gripper_close.hpp"

using namespace std::chrono_literals;

class ActionManagerServer : public rclcpp::Node
{
public:
  using ExecuteChessAction = action_manager_module_interfaces::action::ExecuteChessAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteChessAction>;

  using PieceLocation = sensing_module::srv::PieceLocation;
  using PlanPickPlace = planning_module::srv::PlanPickPlace;

  using GripperOpen  = robotiq_85_gripper_server::srv::GripperOpen;
  using GripperClose = robotiq_85_gripper_server::srv::GripperClose;

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  ActionManagerServer()
  : Node("action_manager_server")
  {
    // ---------- Parameters ----------
    this->declare_parameter<std::vector<int64_t>>(
      "known_aruco_ids",
      std::vector<int64_t>{201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,
                           301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316});

    this->declare_parameter<double>("square_size", 0.05);   // deve matchare sensing_node
    this->declare_parameter<double>("board_z", 0.04);       // idem
    this->declare_parameter<double>("waypoint_time", 2.0);  // sec per waypoint
    this->declare_parameter<double>("settle_time", 0.2);    // extra sleep dopo ogni waypoint

    // Calibration transform parameters (must match sensing_module)
    // Compensate for chessboard offset in Gazebo world
    this->declare_parameter<double>("calib_theta", 0.0);    // rotation angle (radians)
    this->declare_parameter<double>("calib_tx", 0.204);    // X translation (meters) - INVERTED SIGN
    this->declare_parameter<double>("calib_ty", -0.156);   // Y translation (meters) - INVERTED SIGN

    // “cestino catture” fuori scacchiera (in world)
    this->declare_parameter<double>("capture_bin_x", 0.40);
    this->declare_parameter<double>("capture_bin_y", 0.40);
    this->declare_parameter<double>("capture_bin_z", 0.04);

    this->declare_parameter<std::string>(
      "jt_topic", "/joint_trajectory_controller/joint_trajectory");
    
    this->declare_parameter<bool>("use_action_client", false);  // false for Gazebo, true for real robot

    // ---------- Action server ----------
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<ExecuteChessAction>(
      this,
      "execute_chess_action",
      std::bind(&ActionManagerServer::handle_goal, this, _1, _2),
      std::bind(&ActionManagerServer::handle_cancel, this, _1),
      std::bind(&ActionManagerServer::handle_accepted, this, _1));

    // ---------- Clients ----------
    piece_location_client_ = this->create_client<PieceLocation>("piece_location");
    plan_client_           = this->create_client<PlanPickPlace>("plan_pick_place");

    gripper_open_client_  = this->create_client<GripperOpen>("/GripperOpen");
    gripper_close_client_ = this->create_client<GripperClose>("/GripperClose");

    // ---------- Joint Trajectory action client ----------
    trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");
    
    // ---------- Joint State subscriber ----------
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ActionManagerServer::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Action server /execute_chess_action ready");
  }

private:
  rclcpp_action::Server<ExecuteChessAction>::SharedPtr server_;

  rclcpp::Client<PieceLocation>::SharedPtr piece_location_client_;
  rclcpp::Client<PlanPickPlace>::SharedPtr plan_client_;

  rclcpp::Client<GripperOpen>::SharedPtr  gripper_open_client_;
  rclcpp::Client<GripperClose>::SharedPtr gripper_close_client_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

  // UR3 joint order (sin gripper - el gripper tiene su propio controller)
  const std::vector<std::string> joint_names_ = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  // ------------ Action callbacks ------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteChessAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "Goal: piece=%s from=%s to=%s capture=%d castle=%d",
      goal->piece_aruco_id.c_str(),
      goal->from_cell.c_str(),
      goal->to_cell.c_str(),
      goal->is_capture,
      goal->is_castle);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
  }

  // ------------ Utilities ------------
  bool goal_still_active(const std::shared_ptr<GoalHandle> & gh) const
  {
    return rclcpp::ok() && gh && gh->is_active();
  }
  
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
  }

  static std::string normalize_cell(std::string c)
  {
    if (c.size() < 2) return c;
    c[0] = static_cast<char>(std::toupper(c[0]));
    return c;
  }

  // Inverse of sensing_node::pose_to_cell() logic
  // Calculates world-frame pose from cell notation
  // Transform pose from board frame to world frame (inverse of sensing transform)
  geometry_msgs::msg::Pose board_to_world_pose(const geometry_msgs::msg::Pose & pose_board) const
  {
    double calib_theta = this->get_parameter("calib_theta").as_double();
    double calib_tx = this->get_parameter("calib_tx").as_double();
    double calib_ty = this->get_parameter("calib_ty").as_double();

    double cos_theta = std::cos(calib_theta);
    double sin_theta = std::sin(calib_theta);

    geometry_msgs::msg::Pose pose_world = pose_board;
    
    // Sensing does: board = Rot(world) - offset
    // Inverse: world = Rot^-1(board + offset)
    // Since we rotated first then subtracted, we need to add first then inverse-rotate
    double bx = pose_board.position.x;
    double by = pose_board.position.y;
    
    pose_world.position.x = cos_theta * bx + sin_theta * by + calib_tx;
    pose_world.position.y = -sin_theta * bx + cos_theta * by + calib_ty;
    // Z remains the same
    
    return pose_world;
  }

  geometry_msgs::msg::Pose cell_to_pose(const std::string & cell_in) const
  {
    const auto cell = normalize_cell(cell_in);

    constexpr double SQUARE_SIZE = 0.05;  // Must match sensing_node
    constexpr double BOARD_Z = 0.055;     // Z height of pieces in Gazebo (from ArUco TF)

    int file_index = std::clamp<int>(cell[0] - 'A', 0, 7);
    int rank_index = 0;
    try {
      rank_index = std::clamp<int>(std::stoi(cell.substr(1)) - 1, 0, 7);
    } catch (...) {
      rank_index = 0;
    }

    // Calculate base position in idealized board coordinates
    double board_x = (static_cast<double>(file_index) - 3.5) * SQUARE_SIZE;
    double board_y = (static_cast<double>(rank_index) - 3.5) * SQUARE_SIZE;

    geometry_msgs::msg::Pose board_pose;
    board_pose.position.x = board_x;
    board_pose.position.y = board_y;
    board_pose.position.z = BOARD_Z;
    board_pose.orientation.w = 1.0;

    // Transform to world frame
    return board_to_world_pose(board_pose);
  }

  bool wait_for_service_or_abort(
    const rclcpp::ClientBase::SharedPtr & client,
    const std::string & name,
    const std::shared_ptr<GoalHandle> & gh,
    std::shared_ptr<ExecuteChessAction::Result> result,
    const std::shared_ptr<ExecuteChessAction::Feedback> & feedback)
  {
    (void)feedback;
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok() || (gh && gh->is_canceling())) {
        if (goal_still_active(gh)) {
          result->success = false;
          result->message = "Canceled while waiting for service " + name;
          gh->canceled(result);
        }
        return false;
      }
      RCLCPP_WARN(get_logger(), "Waiting for %s ...", name.c_str());
    }
    return true;
  }

  // ------------ Gripper ------------
  bool gripper_open()
  {
    if (!gripper_open_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "/GripperOpen not available");
      return false;
    }
    auto req = std::make_shared<GripperOpen::Request>();
    req->order = true;
    auto fut = gripper_open_client_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) return false;
    return fut.get()->status;
  }

  bool gripper_close()
  {
    if (!gripper_close_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "/GripperClose not available");
      return false;
    }
    auto req = std::make_shared<GripperClose::Request>();
    req->order = true;
    auto fut = gripper_close_client_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) return false;
    return fut.get()->status;
  }

  // ------------ Sensing ------------
  std::optional<PieceLocation::Response> piece_location(int aruco_id)
  {
    if (!piece_location_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "piece_location service not available");
      return std::nullopt;
    }
    auto req = std::make_shared<PieceLocation::Request>();
    req->aruco_id = aruco_id;
    auto fut = piece_location_client_->async_send_request(req);
    if (fut.wait_for(3s) != std::future_status::ready) return std::nullopt;

    auto resp = *fut.get();
    return resp;
  }

  std::optional<int> find_piece_at_cell(const std::string & cell_in, int exclude_id)
  {
    const auto cell = normalize_cell(cell_in);
    auto ids64 = this->get_parameter("known_aruco_ids").as_integer_array();

    for (auto id64 : ids64) {
      int id = static_cast<int>(id64);
      if (id == exclude_id) continue;

      auto resp_opt = piece_location(id);
      if (!resp_opt.has_value()) continue;

      if (resp_opt->found && normalize_cell(resp_opt->cell) == cell) {
        return id;
      }
    }
    return std::nullopt;
  }

  // ------------ Planning ------------
  std::optional<PlanPickPlace::Response> plan_pick_place(
    const geometry_msgs::msg::Pose & src,
    const geometry_msgs::msg::Pose & dst)
  {
    if (!plan_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "plan_pick_place service not available");
      return std::nullopt;
    }
    auto req = std::make_shared<PlanPickPlace::Request>();
    req->source_pose = src;
    req->target_pose = dst;

    auto fut = plan_client_->async_send_request(req);
    if (fut.wait_for(8s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting plan_pick_place");
      return std::nullopt;
    }
    auto resp = *fut.get();
    return resp;
  }

  // ------------ Execution (controller) ------------
  bool send_trajectory_goal(const std::vector<double> & q, double move_time_s)
  {
    std::vector<sensor_msgs::msg::JointState> single_waypoint;
    sensor_msgs::msg::JointState js;
    js.position = q;
    single_waypoint.push_back(js);
    return send_multi_waypoint_trajectory(single_waypoint, move_time_s);
  }
  
  bool send_multi_waypoint_trajectory(const std::vector<sensor_msgs::msg::JointState> & waypoints, double time_per_waypoint)
  {
    if (waypoints.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints provided");
      return false;
    }
    
    if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Trajectory action server not available");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Sending multi-waypoint trajectory with %zu waypoints", waypoints.size());
    
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.header.stamp = rclcpp::Time(0);
    goal_msg.trajectory.joint_names = joint_names_;
    
    // Tolerances
    goal_msg.goal_tolerance.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      goal_msg.goal_tolerance[i].name = joint_names_[i];
      goal_msg.goal_tolerance[i].position = 0.1;
      goal_msg.goal_tolerance[i].velocity = 0.5;
    }
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);

    // Build trajectory with all waypoints
    double cumulative_time = 0.0;
    for (size_t wp_idx = 0; wp_idx < waypoints.size(); ++wp_idx) {
      cumulative_time += time_per_waypoint;
      
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = waypoints[wp_idx].position;
      pt.velocities.resize(6, 0.0);
      pt.accelerations.resize(6, 0.0);
      pt.time_from_start = rclcpp::Duration::from_seconds(cumulative_time);
      goal_msg.trajectory.points.push_back(pt);
    }

    RCLCPP_INFO(get_logger(), "Trajectory has %zu points, total duration: %.1f sec", 
                goal_msg.trajectory.points.size(), cumulative_time);
    
    auto execution_promise = std::make_shared<std::promise<bool>>();
    auto execution_future = execution_promise->get_future();
    
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](GoalHandleTrajectory::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Trajectory goal accepted by server");
        }
      };
    
    send_goal_options.feedback_callback =
      [this](GoalHandleTrajectory::SharedPtr,
             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        (void)feedback;
      };
    
    send_goal_options.result_callback =
      [this, execution_promise](const GoalHandleTrajectory::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Trajectory execution SUCCEEDED");
          execution_promise->set_value(true);
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_ERROR(get_logger(), "Trajectory execution ABORTED");
          if (result.result && !result.result->error_string.empty()) {
            RCLCPP_ERROR(get_logger(), "Error: %s", result.result->error_string.c_str());
          }
          execution_promise->set_value(false);
        } else {
          RCLCPP_ERROR(get_logger(), "Trajectory execution ended with code: %d", (int)result.code);
          execution_promise->set_value(false);
        }
      };
    
    trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    // Wait for trajectory to complete with timeout
    double timeout_sec = cumulative_time + 3.0;  // Trajectory time + 3s margin
    RCLCPP_INFO(get_logger(), "Waiting up to %.1f seconds for trajectory completion...", timeout_sec);
    
    auto status = execution_future.wait_for(std::chrono::duration<double>(timeout_sec));
    
    if (status == std::future_status::ready) {
      bool success = execution_future.get();
      if (success) {
        RCLCPP_INFO(get_logger(), "Multi-waypoint trajectory completed successfully");
        return true;
      } else {
        RCLCPP_ERROR(get_logger(), "Multi-waypoint trajectory failed");
        return false;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Trajectory timeout (%.1fs) - continuing anyway (Gazebo controller issue)", timeout_sec);
      // Gazebo controller doesn't always send result, so we proceed
      // Sleep the expected time to ensure physical movement completes
      std::this_thread::sleep_for(std::chrono::duration<double>(cumulative_time + 0.5));
      return true;
    }
  }

  bool execute_waypoints_with_gripper(
    const std::vector<sensor_msgs::msg::JointState> & wps,
    const std::shared_ptr<GoalHandle> & gh,
    std::shared_ptr<ExecuteChessAction::Result> result,
    const std::shared_ptr<ExecuteChessAction::Feedback> & feedback,
    const std::function<void(const std::string&, float)> & publish_fb)
  {
    if (wps.size() < 6) {
      result->success = false;
      result->message = "Trajectory too short (expected 6 waypoints)";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    const double wp_time   = this->get_parameter("waypoint_time").as_double();

    auto check_cancel = [&]() -> bool {
      if (gh->is_canceling()) {
        result->success = false;
        result->message = "Goal canceled";
        if (goal_still_active(gh)) gh->canceled(result);
        return true;
      }
      return false;
    };

    // Ensure open at start
    publish_fb("opening_gripper", 0.05f);
    if (!gripper_open()) {
      result->success = false;
      result->message = "Failed to open gripper";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }
    if (check_cancel()) return false;

    // Phase 1: Approach and grasp (wp0 -> wp1) as multi-waypoint
    publish_fb("approach_grasp", 0.15f);
    std::vector<sensor_msgs::msg::JointState> approach_grasp_wps = {wps[0], wps[1]};
    if (!send_multi_waypoint_trajectory(approach_grasp_wps, wp_time)) {
      result->success = false;
      result->message = "Failed to move to grasp position";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    if (check_cancel()) return false;

    // Close gripper (pick)
    publish_fb("closing_gripper", 0.28f);
    if (!gripper_close()) {
      result->success = false;
      result->message = "Failed to close gripper";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }
    if (check_cancel()) return false;

    // Phase 2: Lift, transfer, and drop (wp2 -> wp3 -> wp4) as multi-waypoint
    publish_fb("lifting", 0.40f);
    std::vector<sensor_msgs::msg::JointState> lift_transfer_drop_wps = {wps[2], wps[3], wps[4]};
    if (!send_multi_waypoint_trajectory(lift_transfer_drop_wps, wp_time)) {
      result->success = false;
      result->message = "Failed to complete transfer";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    if (check_cancel()) return false;

    // Open gripper (place)
    publish_fb("opening_gripper", 0.80f);
    if (!gripper_open()) {
      result->success = false;
      result->message = "Failed to open gripper at drop";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    if (check_cancel()) return false;

    // Phase 3: Retreat (wp5)
    publish_fb("retreat", 0.90f);
    if (!send_trajectory_goal(wps[5].position, wp_time)) {
      result->success = false;
      result->message = "Failed to retreat from drop position";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    return true;
  }

  bool do_pick_place_by_id_to_cell(
    int piece_id,
    const std::string & to_cell,
    const std::shared_ptr<GoalHandle> & gh,
    std::shared_ptr<ExecuteChessAction::Result> result,
    const std::shared_ptr<ExecuteChessAction::Feedback> & feedback,
    const std::function<void(const std::string&, float)> & publish_fb)
  {
    auto src_opt = piece_location(piece_id);
    if (!src_opt.has_value() || !src_opt->found) {
      result->success = false;
      result->message = "Piece " + std::to_string(piece_id) + " not found (sensing)";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    // Transform source pose from board frame (sensing) to world frame (IK solver)
    geometry_msgs::msg::Pose src_pose = board_to_world_pose(src_opt->pose);
    geometry_msgs::msg::Pose dst_pose = cell_to_pose(to_cell);

    RCLCPP_INFO(get_logger(), "Source pose (piece %d): x=%.3f, y=%.3f, z=%.3f", 
                piece_id, src_pose.position.x, src_pose.position.y, src_pose.position.z);
    RCLCPP_INFO(get_logger(), "Target pose (cell %s): x=%.3f, y=%.3f, z=%.3f", 
                to_cell.c_str(), dst_pose.position.x, dst_pose.position.y, dst_pose.position.z);

    publish_fb("planning", 0.05f);
    auto plan_opt = plan_pick_place(src_pose, dst_pose);
    if (!plan_opt.has_value() || !plan_opt->success) {
      result->success = false;
      result->message = plan_opt ? plan_opt->message : "Planning service failed";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    publish_fb("executing", 0.10f);
    return execute_waypoints_with_gripper(plan_opt->trajectory, gh, result, feedback, publish_fb);
  }

  bool do_pick_place_pose_to_pose(
    const geometry_msgs::msg::Pose & src_pose,
    const geometry_msgs::msg::Pose & dst_pose,
    const std::shared_ptr<GoalHandle> & gh,
    std::shared_ptr<ExecuteChessAction::Result> result,
    const std::shared_ptr<ExecuteChessAction::Feedback> & feedback,
    const std::function<void(const std::string&, float)> & publish_fb)
  {
    publish_fb("planning", 0.05f);
    auto plan_opt = plan_pick_place(src_pose, dst_pose);
    if (!plan_opt.has_value() || !plan_opt->success) {
      result->success = false;
      result->message = plan_opt ? plan_opt->message : "Planning service failed";
      if (goal_still_active(gh)) gh->abort(result);
      return false;
    }

    publish_fb("executing", 0.10f);
    return execute_waypoints_with_gripper(plan_opt->trajectory, gh, result, feedback, publish_fb);
  }

  // ------------ Main execute() ------------
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<ExecuteChessAction::Feedback>();
    auto result   = std::make_shared<ExecuteChessAction::Result>();

    auto publish_fb = [&](const std::string & state, float progress) {
      feedback->state = state;
      feedback->progress = progress;
      if (goal_still_active(goal_handle)) {
        goal_handle->publish_feedback(feedback);
      }
    };

    // 1) parse moving piece id
    int moving_id = -1;
    try {
      moving_id = std::stoi(goal_handle->get_goal()->piece_aruco_id);
    } catch (...) {
      result->success = false;
      result->message = "Invalid piece_aruco_id (not an int): " + goal_handle->get_goal()->piece_aruco_id;
      if (goal_still_active(goal_handle)) goal_handle->abort(result);
      return;
    }

    const auto from_cell = normalize_cell(goal_handle->get_goal()->from_cell);
    const auto to_cell   = normalize_cell(goal_handle->get_goal()->to_cell);

    // 2) optional: check services up-front
    if (!wait_for_service_or_abort(piece_location_client_, "piece_location", goal_handle, result, feedback)) return;
    if (!wait_for_service_or_abort(plan_client_, "plan_pick_place", goal_handle, result, feedback)) return;

    // 3) CASTLING
    if (goal_handle->get_goal()->is_castle) {
      publish_fb("castle_start", 0.02f);

      // Determina celle torre in base alla destinazione del re
      // King side: e1->g1 => rook h1->f1 ; e8->g8 => rook h8->f8
      // Queen side: e1->c1 => rook a1->d1 ; e8->c8 => rook a8->d8
      std::string rook_from, rook_to;
      if (to_cell == "G1") { rook_from = "H1"; rook_to = "F1"; }
      else if (to_cell == "C1") { rook_from = "A1"; rook_to = "D1"; }
      else if (to_cell == "G8") { rook_from = "H8"; rook_to = "F8"; }
      else if (to_cell == "C8") { rook_from = "A8"; rook_to = "D8"; }
      else {
        result->success = false;
        result->message = "is_castle=true but to_cell is not a castling square: " + to_cell;
        if (goal_still_active(goal_handle)) goal_handle->abort(result);
        return;
      }

      // Trova l'ID della torre guardando quale pezzo è in rook_from
      auto rook_id_opt = find_piece_at_cell(rook_from, moving_id);
      if (!rook_id_opt.has_value()) {
        result->success = false;
        result->message = "Rook not found at " + rook_from + " for castling";
        if (goal_still_active(goal_handle)) goal_handle->abort(result);
        return;
      }

      // Muovi RE (moving_id) poi TORRE
      publish_fb("castle_move_king", 0.10f);
      {
        // usa posa reale del re (sensing) e posa target da cella
        auto king_src_opt = piece_location(moving_id);
        if (!king_src_opt.has_value() || !king_src_opt->found) {
          result->success = false;
          result->message = "King not found (sensing)";
          if (goal_still_active(goal_handle)) goal_handle->abort(result);
          return;
        }
        auto king_dst = cell_to_pose(to_cell);

        if (!do_pick_place_pose_to_pose(king_src_opt->pose, king_dst,
                                        goal_handle, result, feedback, publish_fb)) {
          return;
        }
      }

      publish_fb("castle_move_rook", 0.60f);
      if (!do_pick_place_by_id_to_cell(*rook_id_opt, rook_to,
                                       goal_handle, result, feedback, publish_fb)) {
        return;
      }

      result->success = true;
      result->message = "Castling executed";
      publish_fb("done", 1.0f);
      if (goal_still_active(goal_handle)) goal_handle->succeed(result);
      return;
    }

    // 4) CAPTURE (sposta prima il pezzo avversario in capture bin)
    if (goal_handle->get_goal()->is_capture) {
      publish_fb("capture_find_target", 0.05f);

      auto captured_id_opt = find_piece_at_cell(to_cell, moving_id);
      if (!captured_id_opt.has_value()) {
        result->success = false;
        result->message = "Capture requested but no piece found on " + to_cell;
        if (goal_still_active(goal_handle)) goal_handle->abort(result);
        return;
      }

      // Prepara la posa del "bin"
      geometry_msgs::msg::Pose bin_pose;
      bin_pose.position.x = this->get_parameter("capture_bin_x").as_double();
      bin_pose.position.y = this->get_parameter("capture_bin_y").as_double();
      bin_pose.position.z = this->get_parameter("capture_bin_z").as_double();
      bin_pose.orientation.w = 1.0;

      // source pose del catturato = cella to_cell (meglio dalla TF reale)
      auto cap_src_opt = piece_location(*captured_id_opt);
      if (!cap_src_opt.has_value() || !cap_src_opt->found) {
        result->success = false;
        result->message = "Captured piece " + std::to_string(*captured_id_opt) + " not found (sensing)";
        if (goal_still_active(goal_handle)) goal_handle->abort(result);
        return;
      }

      publish_fb("capture_remove_piece", 0.15f);
      if (!do_pick_place_pose_to_pose(cap_src_opt->pose, bin_pose,
                                      goal_handle, result, feedback, publish_fb)) {
        return;
      }
    }

    // 5) NORMAL MOVE: muovi moving_id da posa reale -> posa della to_cell
    publish_fb("move_start", 0.50f);

    auto src_opt = piece_location(moving_id);
    if (!src_opt.has_value() || !src_opt->found) {
      result->success = false;
      result->message = "Moving piece not found (sensing)";
      if (goal_still_active(goal_handle)) goal_handle->abort(result);
      return;
    }

    // If sensing already reports the piece is in the destination cell, skip the move.
    const auto sensed_cell = normalize_cell(src_opt->cell);
    if (sensed_cell == to_cell) {
      RCLCPP_WARN(get_logger(), "Requested to_cell=%s but sensing says piece is already at %s. Skipping move.",
        to_cell.c_str(), sensed_cell.c_str());
      result->success = true;
      result->message = "Piece already at destination: " + to_cell;
      publish_fb("done", 1.0f);
      if (goal_still_active(goal_handle)) goal_handle->succeed(result);
      return;
    }

    // (opzionale) check coerenza from_cell: warn but continue using sensed pose
    if (sensed_cell != from_cell) {
      RCLCPP_WARN(get_logger(),
        "Requested from_cell=%s but sensing says piece is at %s (continuo comunque).",
        from_cell.c_str(), sensed_cell.c_str());
    }

    geometry_msgs::msg::Pose dst_pose = cell_to_pose(to_cell);
    
    // Transform source pose from board frame to world frame
    geometry_msgs::msg::Pose src_pose_world = board_to_world_pose(src_opt->pose);
    
    RCLCPP_INFO(get_logger(), "Source pose (piece %d): x=%.3f, y=%.3f, z=%.3f", 
                moving_id, src_pose_world.position.x, src_pose_world.position.y, src_pose_world.position.z);
    RCLCPP_INFO(get_logger(), "Target pose (cell %s): x=%.3f, y=%.3f, z=%.3f", 
                to_cell.c_str(), dst_pose.position.x, dst_pose.position.y, dst_pose.position.z);
    
    if (!do_pick_place_pose_to_pose(src_pose_world, dst_pose,
                                    goal_handle, result, feedback, publish_fb)) {
      return;
    }

    result->success = true;
    result->message = "Move executed";
    publish_fb("done", 1.0f);
    if (goal_still_active(goal_handle)) goal_handle->succeed(result);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ActionManagerServer>();

  // MultiThreadedExecutor: importante perché facciamo wait su future nel thread execute()
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}