#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/clock.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"

namespace my_cpp_package
{

class PurePursuitBlock final : public bpe::IBlockPlugin, public bpe::ITimerBlock
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Marker = visualization_msgs::msg::Marker;
  using Point = geometry_msgs::msg::Point;

  bpe::BlockResult configure(const bpe::BlockConfig & config) override
  {
    try {
      blackboard_output_ = config["blackboard_output"].as<std::string>();
      frame_ = config["output_frame"].as<std::string>("world");
      marker_namespace_ = config["output_namespace"].as<std::string>("pure_pursuit");
      points_ = {{
        make_point(config["point1_x"].as<double>(0.0), config["point1_y"].as<double>(0.0)),
        make_point(config["point2_x"].as<double>(2.0), config["point2_y"].as<double>(0.0)),
        make_point(config["point3_x"].as<double>(2.0), config["point3_y"].as<double>(2.0))}};
      x_ = config["initial_pose_x"].as<double>(points_[0].x);
      y_ = config["initial_pose_y"].as<double>(points_[0].y);
      yaw_ = config["initial_pose_yaw"].as<double>(0.0);
      lookahead_distance_ = config["lookahead_distance"].as<double>(0.4);
      linear_speed_ = config["linear_speed"].as<double>(0.8);
      max_angular_speed_ = config["max_angular_speed"].as<double>(3.0);
      goal_tolerance_ = config["goal_tolerance"].as<double>(0.1);
      update_period_ = config["update_period"].as<double>(0.05);
      debug_arc_duration_ = config["debug_arc_duration"].as<double>(3.0);
      if (blackboard_output_.empty() || frame_.empty() || marker_namespace_.empty() ||
        !valid_parameters())
      {
        return bpe::BlockResult::FAILURE;
      }
      producer_.bind(blackboard_output_);
      return bpe::BlockResult::SUCCESS;
    } catch (const std::exception &) {
      return bpe::BlockResult::FAILURE;
    }
  }

  bpe::ExecutionResult execute() override
  {
    update_progress();
    if (std::hypot(points_.back().x - x_, points_.back().y - y_) <= goal_tolerance_) {
      finished_ = true;
      progress_ = total_path_length();
    }

    const auto target = point_at_distance(
      std::min(progress_ + lookahead_distance_, total_path_length()));
    const auto dx = target.x - x_;
    const auto dy = target.y - y_;
    const auto local_y = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    const auto chord_squared = dx * dx + dy * dy;
    const auto curvature = !finished_ && chord_squared > epsilon_ ?
      2.0 * local_y / chord_squared : 0.0;
    const auto angular_speed = std::clamp(
      linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
    const auto stamp = rclcpp::Clock().now();

    MarkerArray markers;
    auto turtle = make_marker(stamp, 0, Marker::ARROW);
    turtle.pose.position = make_point(x_, y_);
    turtle.pose.orientation.z = std::sin(yaw_ / 2.0);
    turtle.pose.orientation.w = std::cos(yaw_ / 2.0);
    turtle.scale.x = 0.8;
    turtle.scale.y = 0.35;
    turtle.scale.z = 0.25;
    set_color(turtle, 0.1F, 0.8F, 0.2F);
    markers.markers.push_back(turtle);

    auto path = make_marker(stamp, 1, Marker::LINE_STRIP);
    path.scale.x = 0.06;
    set_color(path, 0.1F, 0.4F, 1.0F);
    path.points.assign(points_.begin(), points_.end());
    markers.markers.push_back(path);

    auto waypoints = make_marker(stamp, 2, Marker::SPHERE_LIST);
    waypoints.scale.x = waypoints.scale.y = waypoints.scale.z = 0.18;
    set_color(waypoints, 1.0F, 0.65F, 0.0F);
    waypoints.points.assign(points_.begin(), points_.end());
    markers.markers.push_back(waypoints);

    auto lookahead = make_marker(stamp, 3, Marker::SPHERE);
    lookahead.pose.position = target;
    lookahead.scale.x = lookahead.scale.y = lookahead.scale.z = 0.22;
    set_color(lookahead, 1.0F, 0.1F, 0.8F);
    markers.markers.push_back(lookahead);

    auto projection = make_marker(stamp, 4, Marker::SPHERE);
    projection.pose.position = point_at_distance(progress_);
    projection.scale.x = projection.scale.y = projection.scale.z = 0.16;
    set_color(projection, 1.0F, 0.9F, 0.0F);
    markers.markers.push_back(projection);

    auto chord = make_marker(stamp, 5, Marker::LINE_STRIP);
    chord.scale.x = 0.025;
    set_color(chord, 1.0F, 0.1F, 0.8F, 0.8F);
    chord.points = {make_point(x_, y_, 0.03), make_point(target.x, target.y, 0.03)};
    markers.markers.push_back(chord);

    auto arc = make_marker(stamp, 6, Marker::LINE_STRIP);
    arc.scale.x = 0.025;
    set_color(arc, 0.1F, 1.0F, 1.0F, 0.9F);
    for (int i = 0; i <= 20; ++i) {
      const auto time = debug_arc_duration_ * static_cast<double>(i) / 20.0;
      const auto arc_yaw = yaw_ + (finished_ ? 0.0 : angular_speed) * time;
      double arc_x = x_;
      double arc_y = y_;
      if (!finished_ && std::abs(angular_speed) > epsilon_) {
        arc_x += linear_speed_ / angular_speed * (std::sin(arc_yaw) - std::sin(yaw_));
        arc_y -= linear_speed_ / angular_speed * (std::cos(arc_yaw) - std::cos(yaw_));
      } else {
        const auto arc_speed = finished_ ? 0.0 : linear_speed_;
        arc_x += arc_speed * time * std::cos(yaw_);
        arc_y += arc_speed * time * std::sin(yaw_);
      }
      arc.points.push_back(make_point(arc_x, arc_y, 0.02));
    }
    markers.markers.push_back(arc);

    auto text_marker = make_marker(stamp, 7, Marker::TEXT_VIEW_FACING);
    text_marker.pose.position = make_point(x_, y_, 0.6);
    text_marker.scale.z = 0.25;
    set_color(text_marker, 1.0F, 1.0F, 1.0F);
    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << "lookahead: " << std::sqrt(chord_squared)
         << " m\ncurvature: " << curvature << " 1/m"
         << "\nangular speed: " << angular_speed << " rad/s"
         << "\nprogress: " << progress_ << "/" << total_path_length() << " m"
         << (finished_ ? "\nGOAL REACHED" : "");
    text_marker.text = text.str();
    markers.markers.push_back(text_marker);

    auto output = producer_.write_access();
    output.value() = std::move(markers);

    if (!finished_) {
      const auto dt = update_period_;
      const auto mid_yaw = yaw_ + angular_speed * dt / 2.0;
      x_ += linear_speed_ * dt * std::cos(mid_yaw);
      y_ += linear_speed_ * dt * std::sin(mid_yaw);
      yaw_ = std::atan2(
        std::sin(yaw_ + angular_speed * dt), std::cos(yaw_ + angular_speed * dt));
    }
    return bpe::ExecutionResult::SUCCESS;
  }

private:
  static constexpr double epsilon_ = 1e-9;

  static Point make_point(double x, double y, double z = 0.0)
  {
    Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  static void set_color(
    Marker & marker, float red, float green, float blue, float alpha = 1.0F)
  {
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
  }

  Marker make_marker(const rclcpp::Time & stamp, int id, int type) const
  {
    Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace_;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  double segment_length(std::size_t index) const
  {
    return std::hypot(
      points_[index + 1].x - points_[index].x,
      points_[index + 1].y - points_[index].y);
  }

  double total_path_length() const
  {
    return segment_length(0) + segment_length(1);
  }

  bool valid_parameters() const
  {
    for (const auto & point : points_) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return false;
      }
    }
    return segment_length(0) > epsilon_ && segment_length(1) > epsilon_ &&
      std::isfinite(x_) && std::isfinite(y_) && std::isfinite(yaw_) &&
      std::isfinite(lookahead_distance_) && lookahead_distance_ > 0.0 &&
      std::isfinite(linear_speed_) && linear_speed_ > 0.0 &&
      std::isfinite(max_angular_speed_) && max_angular_speed_ > 0.0 &&
      std::isfinite(goal_tolerance_) && goal_tolerance_ > 0.0 &&
      std::isfinite(update_period_) && update_period_ > 0.0 &&
      std::isfinite(debug_arc_duration_) && debug_arc_duration_ > 0.0;
  }

  Point point_at_distance(double distance) const
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

  bpe::TypedProducer<MarkerArray> producer_;
  std::array<Point, 3> points_;
  std::string blackboard_output_;
  std::string frame_;
  std::string marker_namespace_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double progress_{0.0};
  double lookahead_distance_{0.4};
  double linear_speed_{0.8};
  double max_angular_speed_{3.0};
  double goal_tolerance_{0.1};
  double update_period_{0.05};
  double debug_arc_duration_{3.0};
  bool finished_{false};
};

}  // namespace my_cpp_package

PLUGINLIB_EXPORT_CLASS(my_cpp_package::PurePursuitBlock, bpe::IBlockPlugin)
