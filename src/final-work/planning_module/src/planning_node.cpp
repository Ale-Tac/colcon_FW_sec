#include "planning_module/planning_node.hpp"

#include <chrono>
#include <future>
#include <vector>

#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace planning_module
{

PlanningNode::PlanningNode()
: Node("planning_node")
{
  RCLCPP_INFO(get_logger(), "PlanningNode starting up");

  ik_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Service per l'action manager
  plan_service_ = this->create_service<PlanPickPlace>(
    "plan_pick_place",
    std::bind(
      &PlanningNode::handle_plan_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

geometry_msgs::msg::Pose PlanningNode::compute_grasp_pose(
  const geometry_msgs::msg::Pose & object_pose,
  int grasp_direction) const
{
  // Logica ispirata a grasps_tf2_broadcaster.cpp (modulo-1) ------------------
  const double d = 0.15;  // 15 cm di offset rispetto all'oggetto

  double dx = 0.0, dy = 0.0, dz = 0.0;
  switch (grasp_direction) {
    case 0: dx = d; break;  // grasp da +X
    case 1: dy = d; break;  // grasp da +Y
    default:
    case 2: dz = d; break;  // grasp dall'alto (+Z)
  }

  geometry_msgs::msg::Pose grasp_pose = object_pose;

  // Qui assumiamo che gli oggetti siano orientati con z verso l'alto,
  // quindi applichiamo l'offset nel frame "world".
  grasp_pose.position.x += dx;
  grasp_pose.position.y += dy;
  grasp_pose.position.z += dz;

  tf2::Quaternion q;
  switch (grasp_direction) {
    case 0:
      q.setRPY(0.0, -M_PI_2, 0.0);
      break;
    case 1:
      q.setRPY(M_PI_2, 0.0, 0.0);
      break;
    case 2:
    default:
      q.setRPY(M_PI, 0.0, 0.0);  // gripper "a testa in giù"
      break;
  }

  grasp_pose.orientation.x = q.x();
  grasp_pose.orientation.y = q.y();
  grasp_pose.orientation.z = q.z();
  grasp_pose.orientation.w = q.w();

  return grasp_pose;
}

std::vector<geometry_msgs::msg::Pose> PlanningNode::build_waypoints(
  const geometry_msgs::msg::Pose & source_pose,
  const geometry_msgs::msg::Pose & target_pose) const
{
  std::vector<geometry_msgs::msg::Pose> poses;

  const int grasp_direction = 2;         // per gli scacchi: grasp dall'alto
  const double pre_offset   = 0.10;      // 10 cm sopra il grasp
  const double lift_offset  = 0.10;      // altezza di sicurezza durante il trasporto

  // ---- SORGENTE ----
  geometry_msgs::msg::Pose grasp_src = compute_grasp_pose(source_pose, grasp_direction);
  geometry_msgs::msg::Pose pre_grasp_src = grasp_src;
  pre_grasp_src.position.z += pre_offset;

  geometry_msgs::msg::Pose lift_src = grasp_src;
  lift_src.position.z += lift_offset;

  // ---- DESTINAZIONE ----
  geometry_msgs::msg::Pose grasp_dst = compute_grasp_pose(target_pose, grasp_direction);
  geometry_msgs::msg::Pose pre_drop_dst = grasp_dst;
  pre_drop_dst.position.z += lift_offset;

  geometry_msgs::msg::Pose post_drop_dst = grasp_dst;
  post_drop_dst.position.z += pre_offset;

  // Sequenza WAYPOINTS:
  // 0: pre-grasp    (sopra sorgente)
  // 1: grasp        (scendi e afferra)
  // 2: lift         (solleva)
  // 3: pre-drop     (sopra destinazione, alto)
  // 4: drop         (abbassa e rilascia)
  // 5: post-drop    (leggera risalita)
  poses.push_back(pre_grasp_src);   // 0
  poses.push_back(grasp_src);       // 1
  poses.push_back(lift_src);        // 2
  poses.push_back(pre_drop_dst);    // 3
  poses.push_back(grasp_dst);       // 4
  poses.push_back(post_drop_dst);   // 5

  return poses;
}

bool PlanningNode::compute_ik_for_pose(
  const geometry_msgs::msg::Pose & pose,
  sensor_msgs::msg::JointState & joint_state_out)
{
  static const char * SERVICE_NAME = "/inverse_kinematics";

  if (!ik_client_) {
    RCLCPP_INFO(get_logger(), "Creating IK client to %s", SERVICE_NAME);
    ik_client_ = this->create_client<IKService>(
      SERVICE_NAME,
      rclcpp::ServicesQoS(),
      ik_callback_group_);
  }

  // Aspetta davvero che il servizio sia disponibile
  while (!ik_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(),
        "ROS shutdown while waiting for %s",
        SERVICE_NAME);
      return false;
    }
    RCLCPP_WARN(
      get_logger(),
      "Waiting for %s service...",
      SERVICE_NAME);
    rclcpp::sleep_for(1s);
  }

  auto request = std::make_shared<IKService::Request>();
  request->type = "UR3";
  request->pose = pose;

  RCLCPP_INFO(get_logger(), "Calling %s for IK", SERVICE_NAME);

  using SharedFuture = rclcpp::Client<IKService>::SharedFuture;
  auto promise = std::make_shared<std::promise<IKService::Response::SharedPtr>>();
  auto response_future = promise->get_future();

  // Fire-and-forget async; completion will run on an executor thread.
  ik_client_->async_send_request(
    request,
    [promise](SharedFuture result) {
      promise->set_value(result.get());
    });

  if (response_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout waiting for IK response");
    return false;
  }

  auto response = response_future.get();
  if (!response->status || response->ik_solution.empty()) {
    RCLCPP_ERROR(get_logger(), "IK solver failed or returned no solutions");
    return false;
  }

  joint_state_out.position = response->ik_solution[0].ik;
  joint_state_out.header.stamp = this->get_clock()->now();
  RCLCPP_INFO(get_logger(), "IK OK, first solution has %zu joints", response->ik_solution[0].ik.size());
  return true;
}


void PlanningNode::handle_plan_request(
  const std::shared_ptr<PlanPickPlace::Request> request,
  std::shared_ptr<PlanPickPlace::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received plan_pick_place request");

  // Costruisci le pose intermedie usando la logica del modulo-1
  auto waypoints = build_waypoints(request->source_pose, request->target_pose);

  std::vector<sensor_msgs::msg::JointState> trajectory;
  trajectory.reserve(waypoints.size());

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    sensor_msgs::msg::JointState js;
    if (!compute_ik_for_pose(waypoints[i], js)) {
      response->success = false;
      response->message = "IK failed for waypoint " + std::to_string(i);
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    trajectory.push_back(js);
  }

  response->success = true;
  response->message = "Trajectory planned successfully";
  response->trajectory = trajectory;

  RCLCPP_INFO(get_logger(), "Plan computed with %zu waypoints", trajectory.size());
}

}  // namespace planning_module

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<planning_module::PlanningNode>();

  // IMPORTANTE: esecutore multi-threaded con almeno due thread per permettere
  // che il callback del servizio e la risposta del client IK vengano gestiti
  // in parallelo, evitando deadlock quando si attende il future.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

