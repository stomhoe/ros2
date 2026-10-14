#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class RvizTopicDemo : public rclcpp::Node
{
public:
  RvizTopicDemo()
  : Node("rviz_topic_demo"), angle_(0.0)
  {
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/rviz_demo/markers", 10);
    timer_ = create_wall_timer(100ms, std::bind(&RvizTopicDemo::publish_markers, this));
    RCLCPP_INFO(
      get_logger(), "Publishing RViz markers on %s",
      marker_publisher_->get_topic_name());
  }

private:
  static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  visualization_msgs::msg::Marker make_marker(
    const rclcpp::Time & stamp, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "rviz_topic_demo";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void publish_markers()
  {
    const auto stamp = now();

    // Marker IDs are stable: RViz replaces each marker with the same namespace and ID.
    auto path = make_marker(stamp, 0, visualization_msgs::msg::Marker::LINE_STRIP);
    path.scale.x = 0.08;
    path.color.r = 0.1F;
    path.color.g = 0.45F;
    path.color.b = 1.0F;
    path.color.a = 1.0F;
    path.points = {
      make_point(-2.0, -1.0), make_point(0.0, 1.0), make_point(2.0, -1.0)};

    auto moving_sphere = make_marker(stamp, 1, visualization_msgs::msg::Marker::SPHERE);
    moving_sphere.pose.position = make_point(1.5 * std::cos(angle_), std::sin(angle_));
    moving_sphere.scale.x = 0.35;
    moving_sphere.scale.y = 0.35;
    moving_sphere.scale.z = 0.35;
    moving_sphere.color.r = 1.0F;
    moving_sphere.color.g = 0.35F;
    moving_sphere.color.b = 0.1F;
    moving_sphere.color.a = 1.0F;

    auto label = make_marker(stamp, 2, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position = make_point(0.0, 1.6, 0.0);
    label.scale.z = 0.3;
    label.color.r = 1.0F;
    label.color.g = 1.0F;
    label.color.b = 1.0F;
    label.color.a = 1.0F;
    label.text = "Published on /rviz_demo/markers";

    visualization_msgs::msg::MarkerArray message;
    message.markers = {path, moving_sphere, label};
    marker_publisher_->publish(message);

    angle_ = std::fmod(angle_ + 0.1, 2.0 * std::acos(-1.0));
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizTopicDemo>());
  rclcpp::shutdown();
  return 0;
}
