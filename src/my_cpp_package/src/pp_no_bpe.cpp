#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode()
  : Node("pure_pursuit")
  {
    points_ = {{
      make_point(
        declare_parameter<double>("point1.x", 0.0),
        declare_parameter<double>("point1.y", 0.0)),
      make_point(
        declare_parameter<double>("point2.x", 2.0),
        declare_parameter<double>("point2.y", 0.0)),
      make_point(
        declare_parameter<double>("point3.x", 2.0),
        declare_parameter<double>("point3.y", 2.0))}};

    validate_path();

    const auto initial_yaw = std::atan2(
      points_[1].y - points_[0].y, points_[1].x - points_[0].x);
    x_ = declare_parameter<double>("initial_pose.x", points_[0].x);
    y_ = declare_parameter<double>("initial_pose.y", points_[0].y);
    yaw_ = declare_parameter<double>("initial_pose.yaw", initial_yaw);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.4);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.8);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 3.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.1);
    update_rate_hz_ = declare_parameter<double>("update_rate", 20.0);
    debug_arc_duration_ = declare_parameter<double>("debug_arc_duration", 3.0);

    validate_parameters();
    progress_ = 0.0;
    finished_ = false;
    log_elapsed_ = 0.0;

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "turtle_visualization", 10);
    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PurePursuitNode::timer_callback, this));
  }

private:
  static constexpr double epsilon_ = 1e-9;

  static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double segment_length(std::size_t segment) const
  {
    return std::hypot(
      points_[segment + 1].x - points_[segment].x,
      points_[segment + 1].y - points_[segment].y);
  }

  void validate_path() const
  {
    for (const auto & point : points_) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("Path point coordinates must be finite");
      }
    }
    for (std::size_t i = 0; i < 2; ++i) {
      if (segment_length(i) <= epsilon_) {
        throw std::invalid_argument("Consecutive path points must be different");
      }
    }
  }

  void validate_parameters() const
  {
    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(yaw_)) {
      throw std::invalid_argument("Initial pose parameters must be finite");
    }
    if (!std::isfinite(lookahead_distance_) || lookahead_distance_ <= 0.0) {
      throw std::invalid_argument("lookahead_distance must be finite and positive");
    }
    if (!std::isfinite(linear_speed_) || linear_speed_ <= 0.0) {
      throw std::invalid_argument("linear_speed must be finite and positive");
    }
    if (!std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0) {
      throw std::invalid_argument("max_angular_speed must be finite and positive");
    }
    if (!std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0) {
      throw std::invalid_argument("goal_tolerance must be finite and positive");
    }
    if (!std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0) {
      throw std::invalid_argument("update_rate must be finite and positive");
    }
    if (update_rate_hz_ > 1000.0) {
      throw std::invalid_argument("update_rate must not exceed 1000 Hz");
    }
    if (!std::isfinite(debug_arc_duration_) || debug_arc_duration_ <= 0.0) {
      throw std::invalid_argument("debug_arc_duration must be finite and positive");
    }
  }

  double total_path_length() const
  {
    return segment_length(0) + segment_length(1);
  }

  geometry_msgs::msg::Point point_at_distance(double distance) const
  {
    distance = std::clamp(distance, 0.0, total_path_length());
    for (std::size_t i = 0; i < 2; ++i) {
      const auto length = segment_length(i);
      if (distance <= length || i == 1) {
        const auto ratio = std::clamp(distance / length, 0.0, 1.0);
        return make_point(
          points_[i].x + ratio * (points_[i + 1].x - points_[i].x),
          points_[i].y + ratio * (points_[i + 1].y - points_[i].y));
      }
      distance -= length;
    }
    return points_.back();
  }

  void update_progress()
  {
    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_progress = progress_;
    double segment_start_distance = 0.0;

    for (std::size_t i = 0; i < 2; ++i) {
      const auto dx = points_[i + 1].x - points_[i].x;
      const auto dy = points_[i + 1].y - points_[i].y;
      const auto length = std::hypot(dx, dy);
      const auto lower_ratio = std::clamp(
        (progress_ - segment_start_distance) / length, 0.0, 1.0);
      const auto projected_ratio = ((x_ - points_[i].x) * dx +
        (y_ - points_[i].y) * dy) / (length * length);
      const auto ratio = std::clamp(projected_ratio, lower_ratio, 1.0);
      const auto projected_x = points_[i].x + ratio * dx;
      const auto projected_y = points_[i].y + ratio * dy;
      const auto distance_squared = (x_ - projected_x) * (x_ - projected_x) +
        (y_ - projected_y) * (y_ - projected_y);
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_progress = segment_start_distance + ratio * length;
      }
      segment_start_distance += length;
    }

    progress_ = std::max(progress_, best_progress);
  }

  visualization_msgs::msg::Marker make_marker(
    const rclcpp::Time & stamp, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "pure_pursuit";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void timer_callback()
  {
    const auto goal = points_.back();
    update_progress();
    const auto goal_distance = std::hypot(goal.x - x_, goal.y - y_);
    if (!finished_ && goal_distance <= goal_tolerance_) {
      finished_ = true;
      progress_ = total_path_length();
    }

    const auto target_distance = std::min(
      progress_ + lookahead_distance_, total_path_length());
    const auto target = point_at_distance(target_distance);
    const auto dx = target.x - x_;
    const auto dy = target.y - y_;
    const auto local_y = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    const auto chord_squared = dx * dx + dy * dy;

    double curvature = 0.0;
    if (!finished_ && chord_squared > epsilon_) {
      curvature = 2.0 * local_y / chord_squared;
    }
    const auto angular_speed = std::clamp(
      linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
    const auto dt = 1.0 / update_rate_hz_;

    const auto stamp = now();
    visualization_msgs::msg::Marker turtle = make_marker(
      stamp, 0, visualization_msgs::msg::Marker::ARROW);
    turtle.pose.position = make_point(x_, y_);
    turtle.pose.orientation.z = std::sin(yaw_ / 2.0);
    turtle.pose.orientation.w = std::cos(yaw_ / 2.0);
    turtle.scale.x = 0.8;
    turtle.scale.y = 0.35;
    turtle.scale.z = 0.25;
    turtle.color.r = 0.1F;
    turtle.color.g = 0.8F;
    turtle.color.b = 0.2F;
    turtle.color.a = 1.0F;

    auto path = make_marker(stamp, 1, visualization_msgs::msg::Marker::LINE_STRIP);
    path.scale.x = 0.06;
    path.color.r = 0.1F;
    path.color.g = 0.4F;
    path.color.b = 1.0F;
    path.color.a = 1.0F;
    path.points.assign(points_.begin(), points_.end());

    auto waypoints = make_marker(
      stamp, 2, visualization_msgs::msg::Marker::SPHERE_LIST);
    waypoints.scale.x = 0.18;
    waypoints.scale.y = 0.18;
    waypoints.scale.z = 0.18;
    waypoints.color.r = 1.0F;
    waypoints.color.g = 0.65F;
    waypoints.color.a = 1.0F;
    waypoints.points.assign(points_.begin(), points_.end());

    auto lookahead = make_marker(stamp, 3, visualization_msgs::msg::Marker::SPHERE);
    lookahead.pose.position = target;
    lookahead.scale.x = 0.22;
    lookahead.scale.y = 0.22;
    lookahead.scale.z = 0.22;
    lookahead.color.r = 1.0F;
    lookahead.color.g = 0.1F;
    lookahead.color.b = 0.8F;
    lookahead.color.a = 1.0F;

    const auto closest_point = point_at_distance(progress_);
    auto projection = make_marker(stamp, 4, visualization_msgs::msg::Marker::SPHERE);
    projection.pose.position = closest_point;
    projection.scale.x = 0.16;
    projection.scale.y = 0.16;
    projection.scale.z = 0.16;
    projection.color.r = 1.0F;
    projection.color.g = 0.9F;
    projection.color.b = 0.0F;
    projection.color.a = 1.0F;

    auto pursuit_chord = make_marker(
      stamp, 5, visualization_msgs::msg::Marker::LINE_STRIP);
    pursuit_chord.scale.x = 0.025;
    pursuit_chord.color.r = 1.0F;
    pursuit_chord.color.g = 0.1F;
    pursuit_chord.color.b = 0.8F;
    pursuit_chord.color.a = 0.8F;
    pursuit_chord.points = {make_point(x_, y_, 0.03), make_point(target.x, target.y, 0.03)};

    auto curvature_arc = make_marker(
      stamp, 6, visualization_msgs::msg::Marker::LINE_STRIP);
    curvature_arc.scale.x = 0.025;
    curvature_arc.color.r = 0.1F;
    curvature_arc.color.g = 1.0F;
    curvature_arc.color.b = 1.0F;
    curvature_arc.color.a = 0.9F;
    const auto arc_linear_speed = finished_ ? 0.0 : linear_speed_;
    for (int i = 0; i <= 20; ++i) {
      const auto arc_time = debug_arc_duration_ * static_cast<double>(i) / 20.0;
      const auto arc_yaw = yaw_ + angular_speed * arc_time;
      double arc_x = x_;
      double arc_y = y_;
      if (std::abs(angular_speed) > epsilon_) {
        arc_x += arc_linear_speed / angular_speed *
          (std::sin(arc_yaw) - std::sin(yaw_));
        arc_y -= arc_linear_speed / angular_speed *
          (std::cos(arc_yaw) - std::cos(yaw_));
      } else {
        arc_x += arc_linear_speed * arc_time * std::cos(yaw_);
        arc_y += arc_linear_speed * arc_time * std::sin(yaw_);
      }
      curvature_arc.points.push_back(make_point(arc_x, arc_y, 0.02));
    }

    auto debug_text = make_marker(
      stamp, 7, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    debug_text.pose.position = make_point(x_, y_, 0.6);
    debug_text.scale.z = 0.25;
    debug_text.color.r = 1.0F;
    debug_text.color.g = 1.0F;
    debug_text.color.b = 1.0F;
    debug_text.color.a = 1.0F;
    std::ostringstream text;
    text.precision(2);
    text << std::fixed << "lookahead: " << std::sqrt(chord_squared)
         << " m\ncurvature: " << curvature << " 1/m"
         << "\nangular speed: " << angular_speed << " rad/s"
         << "\nprogress: " << progress_ << "/" << total_path_length() << " m"
         << (finished_ ? "\nGOAL REACHED" : "");
    debug_text.text = text.str();

    visualization_msgs::msg::MarkerArray markers;
    markers.markers = {
      turtle, path, waypoints, lookahead, projection, pursuit_chord, curvature_arc,
      debug_text};
    publisher_->publish(markers);

    log_elapsed_ += dt;
    if (log_elapsed_ >= 1.0) {
      RCLCPP_INFO(
        get_logger(),
        "position=(%.2f, %.2f) target=(%.2f, %.2f) curvature=%.3f 1/m%s",
        x_, y_, target.x, target.y, curvature, finished_ ? " goal reached" : "");
      log_elapsed_ = 0.0;
    }

    if (!finished_) {
      const auto mid_yaw = yaw_ + angular_speed * dt / 2.0;
      x_ += linear_speed_ * dt * std::cos(mid_yaw);
      y_ += linear_speed_ * dt * std::sin(mid_yaw);
      yaw_ = normalize_angle(yaw_ + angular_speed * dt);
    }
  }

  std::array<geometry_msgs::msg::Point, 3> points_;
  double x_;
  double y_;
  double yaw_;
  double progress_;
  double lookahead_distance_;
  double linear_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  double update_rate_hz_;
  double debug_arc_duration_;
  double log_elapsed_;
  bool finished_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PurePursuitNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pure_pursuit"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
