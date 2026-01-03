#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "sensing_module/srv/piece_location.hpp"

using namespace std::chrono_literals;

class SensingNode : public rclcpp::Node
{
public:
  SensingNode()
  : Node("sensing_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::vector<int64_t>>(
      "aruco_ids", std::vector<int64_t>{316, 213, 201});
    this->declare_parameter<std::string>(
      "yaml_output_path", "/tmp/chess_configuration_sensed.yaml");

    auto ids_param = this->get_parameter("aruco_ids").as_integer_array();
    aruco_ids_.assign(ids_param.begin(), ids_param.end());
    yaml_output_path_ = this->get_parameter("yaml_output_path").as_string();

    timer_ = this->create_wall_timer(
      500ms, std::bind(&SensingNode::update_pieces, this));

    service_ = this->create_service<sensing_module::srv::PieceLocation>(
      "piece_location",
      std::bind(
        &SensingNode::handle_piece_location, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "sensing_node started");
  }

private:
  struct PieceInfo
  {
    geometry_msgs::msg::Pose pose;
    std::string cell;
    bool valid{false};
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<int> aruco_ids_;
  std::map<int, PieceInfo> pieces_;
  std::string yaml_output_path_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<sensing_module::srv::PieceLocation>::SharedPtr service_;

  static constexpr double SQUARE_SIZE = 0.05;
  static constexpr double BOARD_Z = 0.04;

  std::string pose_to_cell(const geometry_msgs::msg::Pose & pose)
  {
    double fx = pose.position.x / SQUARE_SIZE + 3.5;
    double fy = pose.position.y / SQUARE_SIZE + 3.5;

    int file_index = static_cast<int>(std::round(fx));
    int rank_index = static_cast<int>(std::round(fy));

    if (file_index < 0) file_index = 0;
    if (file_index > 7) file_index = 7;
    if (rank_index < 0) rank_index = 0;
    if (rank_index > 7) rank_index = 7;

    char file_char = static_cast<char>('A' + file_index);
    int rank = rank_index + 1;

    return std::string(1, file_char) + std::to_string(rank);
  }

  void update_pieces()
  {
    bool changed = false;

    for (int id : aruco_ids_) {
      const std::string frame_id = "aruco_" + std::to_string(id);

      try {
        geometry_msgs::msg::TransformStamped tf =
          tf_buffer_.lookupTransform("world", frame_id, tf2::TimePointZero);

        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;
        pose.orientation = tf.transform.rotation;

        std::string cell = pose_to_cell(pose);

        PieceInfo info{pose, cell, true};

        auto it = pieces_.find(id);
        if (it == pieces_.end() ||
            it->second.cell != cell ||
            it->second.pose.position.x != pose.position.x ||
            it->second.pose.position.y != pose.position.y) {
          changed = true;
        }

        pieces_[id] = info;

      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "No transform world -> %s yet: %s", frame_id.c_str(), ex.what());
      }
    }

    if (changed) {
      write_yaml();
    }
  }

  void write_yaml()
  {
    std::ofstream out(yaml_output_path_);
    if (!out) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot open yaml file '%s' for writing", yaml_output_path_.c_str());
      return;
    }

    out << "pieces:\n";
    for (const auto & kv : pieces_) {
      int id = kv.first;
      const PieceInfo & info = kv.second;
      if (!info.valid) {
        continue;
      }
      out << "  aruco_" << id << ": \"" << info.cell << "\"\n";
    }

    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Chess configuration written to %s", yaml_output_path_.c_str());
  }

  void handle_piece_location(
    const std::shared_ptr<sensing_module::srv::PieceLocation::Request> request,
    std::shared_ptr<sensing_module::srv::PieceLocation::Response> response)
  {
    auto it = pieces_.find(request->aruco_id);
    if (it == pieces_.end() || !it->second.valid) {
      response->found = false;
      response->cell = "not-found";
      response->pose = geometry_msgs::msg::Pose();

      RCLCPP_INFO(this->get_logger(),
                  "[piece_location] id %d -> not-found",
                  request->aruco_id);

      return;
    }

    response->found = true;
    response->pose = it->second.pose;
    response->cell = it->second.cell;

    RCLCPP_INFO(this->get_logger(),
                "[piece_location] id %d -> cell %s",
                request->aruco_id,
                it->second.cell.c_str());
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
