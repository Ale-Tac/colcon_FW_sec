#ifndef PLANNING_MODULE__PLANNING_NODE_HPP_
#define PLANNING_MODULE__PLANNING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "planning_module/srv/plan_pick_place.hpp"
#include "kinenikros2/srv/inverse_kinematics.hpp"

namespace planning_module
{

class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode();

private:
  using PlanPickPlace = planning_module::srv::PlanPickPlace;
  using IKService     = kinenikros2::srv::InverseKinematics;

  rclcpp::Service<PlanPickPlace>::SharedPtr plan_service_;
  rclcpp::Client<IKService>::SharedPtr      ik_client_;
  rclcpp::CallbackGroup::SharedPtr          ik_callback_group_;

  // callback del servizio
  void handle_plan_request(
    const std::shared_ptr<PlanPickPlace::Request> request,
    std::shared_ptr<PlanPickPlace::Response> response);

  // chiama l'IK e riempie un JointState
  bool compute_ik_for_pose(
    const geometry_msgs::msg::Pose & pose,
    sensor_msgs::msg::JointState & joint_state_out);

  // crea la sequenza di pose (pre-grasp, grasp, lift, pre-drop, drop, post-drop)
  std::vector<geometry_msgs::msg::Pose> build_waypoints(
    const geometry_msgs::msg::Pose & source_pose,
    const geometry_msgs::msg::Pose & target_pose) const;

  // calcola la pose di grasp "tipo modulo-1" a partire dalla pose dell'oggetto
  geometry_msgs::msg::Pose compute_grasp_pose(
    const geometry_msgs::msg::Pose & object_pose,
    int grasp_direction) const;
};

}  // namespace planning_module

#endif  // PLANNING_MODULE__PLANNING_NODE_HPP_

