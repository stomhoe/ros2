#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"

namespace my_cpp_package
{

class TimedElasticBandBlock final : public bpe::IBlockPlugin, public bpe::ITimerBlock
{
public:
  using Point = geometry_msgs::msg::Point;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::TwistStamped;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  bpe::BlockResult configure(const bpe::BlockConfig & config) override
  {
    try {
      pose_input_ = config["pose_input"].as<std::string>();
      scan_input_ = config["scan_input"].as<std::string>();
      command_output_ = config["command_output"].as<std::string>();
      markers_output_ = config["markers_output"].as<std::string>();
      frame_ = config["frame"].as<std::string>("world");
      goal_ = make_point(config["goal_x"].as<double>(4.0), config["goal_y"].as<double>(0.0));
      max_linear_speed_ = config["max_linear_speed"].as<double>(0.35);
      max_angular_speed_ = config["max_angular_speed"].as<double>(1.0);
      obstacle_clearance_ = config["obstacle_clearance"].as<double>(0.55);
      goal_tolerance_ = config["goal_tolerance"].as<double>(0.15);
      pose_timeout_ = config["pose_timeout"].as<double>(0.5);
      scan_timeout_ = config["scan_timeout"].as<double>(0.5);
      node_count_ = config["node_count"].as<std::size_t>(14);
      optimization_steps_ = config["optimization_steps"].as<std::size_t>(35);
      time_step_ = config["time_step"].as<double>(0.25);

      if (pose_input_.empty() || scan_input_.empty() || command_output_.empty() ||
        markers_output_.empty() || frame_.empty() || !valid_parameters())
      {
        return bpe::BlockResult::FAILURE;
      }

      pose_consumer_.bind(pose_input_);
      scan_consumer_.bind(scan_input_);
      command_producer_.bind(command_output_);
      markers_producer_.bind(markers_output_);
      return bpe::BlockResult::SUCCESS;
    } catch (const std::exception &) {
      return bpe::BlockResult::FAILURE;
    }
  }

  bpe::ExecutionResult execute() override
  {
    const auto now = rclcpp::Clock().now();
    auto pose_input = pose_consumer_.read_access();
    if (pose_input.has_new_value()) {
      latest_pose_ = pose_input.value();
      pose_received_ns_ = now.nanoseconds();
      have_pose_ = true;
    }

    auto scan_input = scan_consumer_.read_access();
    if (scan_input.has_new_value()) {
      latest_scan_ = scan_input.value();
      scan_received_ns_ = now.nanoseconds();
      have_scan_ = true;
    }

    const bool pose_valid = have_pose_ &&
      age_seconds(now.nanoseconds(), pose_received_ns_) <= pose_timeout_ &&
      (latest_pose_.header.frame_id.empty() || latest_pose_.header.frame_id == frame_) &&
      std::isfinite(latest_pose_.pose.position.x) &&
      std::isfinite(latest_pose_.pose.position.y);
    const bool scan_valid = have_scan_ &&
      age_seconds(now.nanoseconds(), scan_received_ns_) <= scan_timeout_ &&
      valid_scan_data(latest_scan_);

    Point robot;
    double yaw = 0.0;
    if (pose_valid) {
      robot = latest_pose_.pose.position;
      yaw = get_yaw(latest_pose_.pose.orientation);
      if (!std::isfinite(yaw)) {
        return bpe::ExecutionResult::FAILURE;
      }
    }

    std::vector<Point> obstacles;
    if (pose_valid && scan_valid) {
      obstacles = scan_points(robot, yaw);
    }

    Twist command;
    command.header.stamp = now;
    command.header.frame_id = frame_;
    std::vector<Point> band;
    const auto goal_distance = pose_valid ?
      std::hypot(goal_.x - robot.x, goal_.y - robot.y) :
      std::numeric_limits<double>::infinity();
    const bool goal_reached = pose_valid && goal_distance <= goal_tolerance_;

    if (pose_valid && scan_valid && !goal_reached) {
      band = optimize_band(robot, obstacles);
      command = make_command(now, robot, yaw, band);
    }

    auto command_access = command_producer_.write_access();
    command_access.value() = command;
    auto marker_access = markers_producer_.write_access();
    marker_access.value() = make_markers(
      now, pose_valid, scan_valid, robot, yaw, goal_distance, band, obstacles, goal_reached);
    return bpe::ExecutionResult::SUCCESS;
  }

private:
  static constexpr std::size_t scan_stride_ = 3;
  static constexpr double obstacle_influence_distance_ = 1.1;
  static constexpr double max_time_step_ = 1.0;
  static constexpr double min_time_step_ = 0.08;

  static double get_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static Point make_point(double x, double y, double z = 0.0)
  {
    Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double age_seconds(int64_t now_ns, int64_t received_ns)
  {
    if (now_ns < received_ns) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - received_ns) / 1e9;
  }

  static bool valid_scan_data(const LaserScan & scan)
  {
    return !scan.ranges.empty() && std::isfinite(scan.angle_min) &&
      std::isfinite(scan.angle_increment) && scan.angle_increment > 0.0 &&
      std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
      scan.range_min >= 0.0 && scan.range_max > scan.range_min;
  }

  std::vector<Point> scan_points(const Point & robot, double yaw) const
  {
    std::vector<Point> points;
    points.reserve(latest_scan_.ranges.size() / scan_stride_);
    for (std::size_t i = 0; i < latest_scan_.ranges.size(); i += scan_stride_) {
      const auto range = static_cast<double>(latest_scan_.ranges[i]);
      if (!std::isfinite(range) || range < latest_scan_.range_min ||
        range > latest_scan_.range_max)
      {
        continue;
      }
      const auto angle = yaw + latest_scan_.angle_min +
        static_cast<double>(i) * latest_scan_.angle_increment;
      points.push_back(make_point(
        robot.x + range * std::cos(angle),
        robot.y + range * std::sin(angle),
        0.0));
    }
    return points;
  }

  std::vector<Point> optimize_band(
    const Point & robot, const std::vector<Point> & obstacles) const
  {
    const auto dx = goal_.x - robot.x;
    const auto dy = goal_.y - robot.y;
    const auto length = std::hypot(dx, dy);
    const auto ux = dx / length;
    const auto uy = dy / length;
    const auto detour_offset = obstacle_clearance_ + 0.4;
    double left_cost = 0.0;
    double right_cost = 0.0;
    bool needs_detour = false;
    for (const auto & obstacle : obstacles) {
      const auto rx = obstacle.x - robot.x;
      const auto ry = obstacle.y - robot.y;
      const auto progress = rx * ux + ry * uy;
      const auto lateral = ux * ry - uy * rx;
      if (progress <= 0.0 || progress >= length ||
        std::abs(lateral) >= obstacle_clearance_ + 0.3)
      {
        continue;
      }
      needs_detour = true;
      const auto left_distance = std::hypot(
        lateral - detour_offset, std::max(0.0, std::abs(progress - length / 2.0) - 0.3));
      const auto right_distance = std::hypot(
        lateral + detour_offset, std::max(0.0, std::abs(progress - length / 2.0) - 0.3));
      left_cost += std::pow(std::max(0.0, obstacle_clearance_ - left_distance), 2);
      right_cost += std::pow(std::max(0.0, obstacle_clearance_ - right_distance), 2);
    }
    const auto detour_side = left_cost <= right_cost ? 1.0 : -1.0;

    std::vector<Point> band(node_count_);
    for (std::size_t i = 0; i < node_count_; ++i) {
      const auto fraction = static_cast<double>(i) /
        static_cast<double>(node_count_ - 1);
      const auto offset = needs_detour ?
        detour_side * detour_offset * std::sin(3.14159265358979323846 * fraction) : 0.0;
      band[i] = make_point(
        robot.x + fraction * dx - uy * offset,
        robot.y + fraction * dy + ux * offset,
        0.0);
    }

    for (std::size_t step = 0; step < optimization_steps_; ++step) {
      auto next = band;
      for (std::size_t i = 1; i + 1 < band.size(); ++i) {
        const auto smooth_x = band[i - 1].x + band[i + 1].x - 2.0 * band[i].x;
        const auto smooth_y = band[i - 1].y + band[i + 1].y - 2.0 * band[i].y;
        double nearest_distance = obstacle_influence_distance_;
        double nearest_dx = 0.0;
        double nearest_dy = 0.0;
        for (const auto & obstacle : obstacles) {
          const auto dx = band[i].x - obstacle.x;
          const auto dy = band[i].y - obstacle.y;
          const auto distance = std::hypot(dx, dy);
          if (distance > 1e-6 && distance < nearest_distance) {
            nearest_distance = distance;
            nearest_dx = dx;
            nearest_dy = dy;
          }
        }
        const auto repel_strength = std::max(0.0, obstacle_clearance_ - nearest_distance);
        const auto repel_x = nearest_distance < obstacle_influence_distance_ ?
          nearest_dx / nearest_distance * repel_strength : 0.0;
        const auto repel_y = nearest_distance < obstacle_influence_distance_ ?
          nearest_dy / nearest_distance * repel_strength : 0.0;
        const auto displacement_x = 0.18 * smooth_x + 0.22 * repel_x;
        const auto displacement_y = 0.18 * smooth_y + 0.22 * repel_y;
        const auto displacement = std::hypot(displacement_x, displacement_y);
        const auto scale = displacement > 0.12 ? 0.12 / displacement : 1.0;
        next[i].x += displacement_x * scale;
        next[i].y += displacement_y * scale;
      }
      band = std::move(next);
    }
    return band;
  }

  Twist make_command(
    const rclcpp::Time & stamp, const Point & robot, double yaw,
    const std::vector<Point> & band) const
  {
    Twist command;
    command.header.stamp = stamp;
    command.header.frame_id = frame_;
    if (band.size() < 2) {
      return command;
    }

    const auto dx = band[1].x - robot.x;
    const auto dy = band[1].y - robot.y;
    const auto segment_length = std::hypot(dx, dy);
    if (segment_length < 1e-6) {
      return command;
    }
    const auto heading_error = normalize_angle(std::atan2(dy, dx) - yaw);
    const auto segment_time = std::clamp(
      std::max(time_step_, segment_length / max_linear_speed_),
      min_time_step_, max_time_step_);
    const auto requested_speed = std::min(max_linear_speed_, segment_length / segment_time);
    command.twist.linear.x = requested_speed *
      std::clamp(std::cos(heading_error), 0.0, 1.0);
    command.twist.angular.z = std::clamp(
      heading_error / segment_time, -max_angular_speed_, max_angular_speed_);
    return command;
  }

  bool valid_parameters() const
  {
    return std::isfinite(goal_.x) && std::isfinite(goal_.y) &&
      std::isfinite(max_linear_speed_) && max_linear_speed_ > 0.0 &&
      std::isfinite(max_angular_speed_) && max_angular_speed_ > 0.0 &&
      std::isfinite(obstacle_clearance_) && obstacle_clearance_ > 0.0 &&
      std::isfinite(goal_tolerance_) && goal_tolerance_ > 0.0 &&
      std::isfinite(pose_timeout_) && pose_timeout_ > 0.0 &&
      std::isfinite(scan_timeout_) && scan_timeout_ > 0.0 &&
      node_count_ >= 3 && node_count_ <= 100 &&
      optimization_steps_ > 0 && optimization_steps_ <= 500 &&
      std::isfinite(time_step_) && time_step_ > 0.0;
  }

  static void set_color(Marker & marker, float r, float g, float b, float a = 1.0F)
  {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
  }

  Marker make_marker(const rclcpp::Time & stamp, int id, int type) const
  {
    Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = stamp;
    marker.ns = "teb";
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  MarkerArray make_markers(
    const rclcpp::Time & stamp, bool pose_valid, bool scan_valid,
    const Point & robot, double yaw, double goal_distance,
    const std::vector<Point> & band, const std::vector<Point> & obstacles,
    bool goal_reached) const
  {
    MarkerArray markers;
    auto goal = make_marker(stamp, 0, Marker::SPHERE);
    goal.pose.position = goal_;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.24;
    set_color(goal, 0.1F, 1.0F, 0.2F);
    markers.markers.push_back(goal);

    auto robot_marker = make_marker(stamp, 1, Marker::ARROW);
    if (pose_valid) {
      robot_marker.pose.position = robot;
      robot_marker.pose.orientation.z = std::sin(yaw / 2.0);
      robot_marker.pose.orientation.w = std::cos(yaw / 2.0);
      robot_marker.scale.x = 0.42;
      robot_marker.scale.y = 0.17;
      robot_marker.scale.z = 0.14;
      set_color(robot_marker, 0.1F, 0.9F, 0.2F);
    } else {
      robot_marker.action = Marker::DELETE;
    }
    markers.markers.push_back(robot_marker);

    auto scan = make_marker(stamp, 2, Marker::POINTS);
    scan.scale.x = scan.scale.y = 0.055;
    set_color(scan, 1.0F, 0.25F, 0.1F);
    if (pose_valid && scan_valid) {
      scan.points = obstacles;
    }
    markers.markers.push_back(scan);

    auto trajectory = make_marker(stamp, 3, Marker::LINE_STRIP);
    trajectory.scale.x = 0.045;
    set_color(trajectory, 0.1F, 0.9F, 1.0F);
    trajectory.points = band;
    markers.markers.push_back(trajectory);

    auto status = make_marker(stamp, 4, Marker::TEXT_VIEW_FACING);
    if (pose_valid) {
      status.pose.position.x = robot.x;
      status.pose.position.y = robot.y;
      status.pose.position.z = 0.6;
      status.scale.z = 0.19;
      set_color(status, 1.0F, 1.0F, 1.0F);
      status.text = goal_reached ? "GOAL REACHED\nTEB" :
        "OPTIMIZING TIMED BAND\ndistance: " +
        std::to_string(goal_distance).substr(0, 4) + " m";
    } else {
      status.action = Marker::DELETE;
    }
    markers.markers.push_back(status);
    return markers;
  }

  bpe::TypedConsumer<Pose> pose_consumer_;
  bpe::TypedConsumer<LaserScan> scan_consumer_;
  bpe::TypedProducer<Twist> command_producer_;
  bpe::TypedProducer<MarkerArray> markers_producer_;
  Pose latest_pose_;
  LaserScan latest_scan_;
  Point goal_;
  std::string pose_input_;
  std::string scan_input_;
  std::string command_output_;
  std::string markers_output_;
  std::string frame_;
  double max_linear_speed_{0.35};
  double max_angular_speed_{1.0};
  double obstacle_clearance_{0.55};
  double goal_tolerance_{0.15};
  double pose_timeout_{0.5};
  double scan_timeout_{0.5};
  double time_step_{0.25};
  std::size_t node_count_{14};
  std::size_t optimization_steps_{35};
  int64_t pose_received_ns_{0};
  int64_t scan_received_ns_{0};
  bool have_pose_{false};
  bool have_scan_{false};
};

}  // namespace my_cpp_package

PLUGINLIB_EXPORT_CLASS(my_cpp_package::TimedElasticBandBlock, bpe::IBlockPlugin)
