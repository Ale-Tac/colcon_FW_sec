#ifndef ACTION_MANAGER__ACTION_MANAGER_NODE_HPP_
#define ACTION_MANAGER__ACTION_MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "action_manager/action/chess_action.hpp"
#include "sensing_module/srv/piece_location.hpp"
#include "planning_module/srv/plan_pick_place.hpp"
#include "aruco_broadcaster/srv/get_marker_tf.hpp"
#include "chesslab_setup2_interfaces/srv/set_rob_conf.hpp"
#include "chesslab_setup2_interfaces/srv/set_obj_pose.hpp"
#include "robotiq_85_gripper_server/srv/gripper_order.hpp"

namespace action_manager
{

/**
 * @brief ActionManagerNode orchestrates chess piece movements
 * 
 * Handles:
 * - MOVE: move a piece to an empty square
 * - CAPTURE: remove an opponent piece and move attacking piece
 * - CASTLING: coordinated king + rook movement
 * 
 * Uses:
 * - sensing_module: get piece locations
 * - planning_module: compute trajectories
 * - chesslab_setup2: robot control (simulation)
 * - robotiq_85_gripper_server: gripper control
 */
class ActionManagerNode : public rclcpp::Node
{
public:
  using ChessAction = action_manager::action::ChessAction;
  using GoalHandleChessAction = rclcpp_action::ServerGoalHandle<ChessAction>;

  ActionManagerNode();

private:
  // Action server
  rclcpp_action::Server<ChessAction>::SharedPtr action_server_;

  // Service clients
  rclcpp::Client<sensing_module::srv::PieceLocation>::SharedPtr piece_location_client_;
  rclcpp::Client<planning_module::srv::PlanPickPlace>::SharedPtr planning_client_;
  rclcpp::Client<chesslab_setup2_interfaces::srv::SetRobConf>::SharedPtr set_conf_client_;
  rclcpp::Client<chesslab_setup2_interfaces::srv::SetObjPose>::SharedPtr set_obj_pose_client_;
  rclcpp::Client<robotiq_85_gripper_server::srv::GripperOrder>::SharedPtr gripper_client_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  // Board geometry
  static constexpr double SQUARE_SIZE = 0.05;  // 5 cm
  static constexpr double BOARD_CENTER_X = 0.0;
  static constexpr double BOARD_CENTER_Y = 0.0;
  static constexpr double BOARD_Z = 0.04;  // Height to place pieces on board

  // Gripper parameters
  static constexpr float GRIPPER_CLOSE_POSITION = 0.8f;
  static constexpr float GRIPPER_OPEN_POSITION = 0.0f;

  // Piece height mapping (mm)
  std::map<std::pair<int, int>, double> piece_heights_ = {
    {{2, 0}, 0.04},  // Black pawns (201-208)
    {{2, 1}, 0.06},  // Black rooks (209-210)
    {{2, 2}, 0.06},  // Black knights (211-212)
    {{2, 3}, 0.06},  // Black bishops (213-214)
    {{2, 4}, 0.08},  // Black queen (215)
    {{2, 5}, 0.08},  // Black king (216)
    {{3, 0}, 0.04},  // White pawns (301-308)
    {{3, 1}, 0.06},  // White rooks (309-310)
    {{3, 2}, 0.06},  // White knights (311-312)
    {{3, 3}, 0.06},  // White bishops (313-314)
    {{3, 4}, 0.08},  // White queen (315)
    {{3, 5}, 0.08},  // White king (316)
  };

  // Action handlers
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ChessAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleChessAction> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleChessAction> goal_handle);

  void execute_chess_action(
    const std::shared_ptr<GoalHandleChessAction> goal_handle);

  // Helper functions
  geometry_msgs::msg::Pose cell_to_pose(const std::string & cell);
  std::string pose_to_cell(const geometry_msgs::msg::Pose & pose);
  double get_piece_height(int aruco_id);

  // Service call wrappers
  bool get_piece_pose(
    int aruco_id,
    geometry_msgs::msg::Pose & pose_out,
    std::string & cell_out);

  bool plan_pick_place(
    const geometry_msgs::msg::Pose & source,
    const geometry_msgs::msg::Pose & target,
    std::vector<sensor_msgs::msg::JointState> & trajectory_out);

  bool execute_trajectory(
    const std::vector<sensor_msgs::msg::JointState> & trajectory);

  bool control_gripper(float position);

  // Action-specific implementations
  bool execute_move(
    const std::shared_ptr<GoalHandleChessAction> goal_handle,
    std::shared_ptr<ChessAction::Feedback> feedback);

  bool execute_capture(
    const std::shared_ptr<GoalHandleChessAction> goal_handle,
    std::shared_ptr<ChessAction::Feedback> feedback);

  bool execute_castling(
    const std::shared_ptr<GoalHandleChessAction> goal_handle,
    std::shared_ptr<ChessAction::Feedback> feedback);

  // Utility
  void log_pose(const geometry_msgs::msg::Pose & pose, const std::string & label);
};

}  // namespace action_manager

#endif  // ACTION_MANAGER__ACTION_MANAGER_NODE_HPP_
