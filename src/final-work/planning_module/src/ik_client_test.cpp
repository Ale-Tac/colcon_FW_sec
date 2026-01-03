#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kinenikros2/srv/inverse_kinematics.hpp"

using namespace std::chrono_literals;
using InverseKinematics = kinenikros2::srv::InverseKinematics;

class IKClientNode : public rclcpp::Node
{
public:
  IKClientNode()
  : Node("ik_client_test")
  {
    RCLCPP_INFO(get_logger(), "IKClientNode starting");

    client_ = this->create_client<InverseKinematics>("/inverse_kinematics");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "ROS shutdown while waiting for /inverse_kinematics");
        return;
      }
      RCLCPP_WARN(get_logger(), "Waiting for /inverse_kinematics...");
    }
    RCLCPP_INFO(get_logger(), "/inverse_kinematics is available!");


    auto req = std::make_shared<InverseKinematics::Request>();
    req->type = "ik";

    // posa di prova, molto semplice
    req->pose.position.x = 0.3;
    req->pose.position.y = 0.0;
    req->pose.position.z = 0.3;
    req->pose.orientation.w = 1.0;

    auto future = client_->async_send_request(req);
    auto status = future.wait_for(5s);

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for IK response");
      return;
    }

    auto resp = future.get();
    if (!resp->status || resp->ik_solution.empty()) {
      RCLCPP_ERROR(get_logger(), "IK failed or returned no solutions");
      return;
    }

    RCLCPP_INFO(get_logger(), "IK OK, first solution has %zu joints",
                resp->ik_solution[0].ik.size());
  }

private:
  rclcpp::Client<InverseKinematics>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKClientNode>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}