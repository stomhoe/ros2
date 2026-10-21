#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <utility>

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

class Bug2ControllerBlock final : public bpe::IBlockPlugin, public bpe::ITimerBlock
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
      start_ = make_point(
        config["start_x"].as<double>(0.0), config["start_y"].as<double>(0.0));
      goal_ = make_point(
        config["goal_x"].as<double>(4.0), config["goal_y"].as<double>(0.0));
      linear_speed_ = config["linear_speed"].as<double>(0.35);
      boundary_speed_ = config["boundary_speed"].as<double>(0.22);
      max_angular_speed_ = config["max_angular_speed"].as<double>(1.0);
      obstacle_distance_ = config["obstacle_distance"].as<double>(0.7);
      wall_distance_ = config["wall_distance"].as<double>(0.48);
      goal_tolerance_ = config["goal_tolerance"].as<double>(0.15);
      pose_timeout_ = config["pose_timeout"].as<double>(0.5);
      scan_timeout_ = config["scan_timeout"].as<double>(0.5);

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

    const bool pose_fresh = have_pose_ && age_seconds(now.nanoseconds(), pose_received_ns_) <=
      pose_timeout_;
    const bool scan_fresh = have_scan_ && age_seconds(now.nanoseconds(), scan_received_ns_) <=
      scan_timeout_;

    Point robot = start_;
    double yaw = 0.0;
    if (pose_fresh && (latest_pose_.header.frame_id.empty() ||
      latest_pose_.header.frame_id == frame_))
    {
      robot.x = latest_pose_.pose.position.x;
      robot.y = latest_pose_.pose.position.y;
      yaw = get_yaw(latest_pose_.pose.orientation);
    }

    double front_range = std::numeric_limits<double>::infinity();
    double right_range = std::numeric_limits<double>::infinity();
    double left_range = std::numeric_limits<double>::infinity();
    const bool valid_scan = scan_fresh && valid_scan_data(latest_scan_);
    if (valid_scan) {
      front_range = sector_min(latest_scan_, -front_sector_, front_sector_);
      right_range = sector_min(latest_scan_, -right_sector_max_, -right_sector_min_);
      left_range = sector_min(latest_scan_, right_sector_min_, right_sector_max_);
    }

    const auto distance_to_goal = std::hypot(goal_.x - robot.x, goal_.y - robot.y);
    const auto mline = mline_coordinates(robot.x, robot.y);
    if (distance_to_goal <= goal_tolerance_) {
      state_ = State::GOAL_REACHED;
    }

    Twist twist;
    twist.header.stamp = now;
    twist.header.frame_id = frame_;

    if (pose_fresh && valid_scan && state_ != State::GOAL_REACHED) {
      if (state_ == State::GO_TO_GOAL) {
        const auto goal_heading = std::atan2(goal_.y - robot.y, goal_.x - robot.x);
        const auto heading_error = normalize_angle(goal_heading - yaw);
        if (std::abs(heading_error) < 0.6 && front_range < obstacle_distance_) {
          state_ = State::FOLLOW_BOUNDARY;
          hit_point_ = robot;
          hit_goal_distance_ = distance_to_goal;
          hit_mline_progress_ = mline.first;
          has_hit_point_ = true;
          has_left_mline_ = false;
          boundary_side_ = left_range < right_range ? -1.0 : 1.0;
        } else {
          twist.twist.linear.x = std::abs(heading_error) > 0.6 ?
            0.0 : linear_speed_;
          twist.twist.angular.z = std::clamp(
            1.8 * heading_error, -max_angular_speed_, max_angular_speed_);
        }
      }

      if (state_ == State::FOLLOW_BOUNDARY) {
        if (std::abs(mline.second) > mline_tolerance_) {
          has_left_mline_ = true;
        }
        const bool returned_to_mline = has_left_mline_ &&
          std::abs(mline.second) <= mline_tolerance_ &&
          mline.first > hit_mline_progress_ + leave_progress_margin_ &&
          distance_to_goal + leave_distance_margin_ < hit_goal_distance_;

        if (returned_to_mline) {
          state_ = State::GO_TO_GOAL;
          has_hit_point_ = false;
          const auto heading_error = normalize_angle(
            std::atan2(goal_.y - robot.y, goal_.x - robot.x) - yaw);
          twist.twist.linear.x = std::abs(heading_error) > 0.6 ? 0.0 : linear_speed_;
          twist.twist.angular.z = std::clamp(
            1.8 * heading_error, -max_angular_speed_, max_angular_speed_);
        } else {
          const auto goal_heading = std::atan2(goal_.y - robot.y, goal_.x - robot.x);
          const auto heading_error = normalize_angle(goal_heading - yaw);
          follow_boundary(front_range, right_range, left_range, heading_error, twist);
        }
      }
    }

    auto command = command_producer_.write_access();
    command.value() = twist;

    auto markers = markers_producer_.write_access();
    markers.value() = make_markers(
      now, pose_fresh, valid_scan, robot, yaw, distance_to_goal);
    return bpe::ExecutionResult::SUCCESS;
  }

private:
  enum class State { GO_TO_GOAL, FOLLOW_BOUNDARY, GOAL_REACHED };

  static constexpr double pi_ = 3.14159265358979323846;
  static constexpr double front_sector_ = 0.20;
  static constexpr double right_sector_min_ = 1.20;
  static constexpr double right_sector_max_ = 1.95;

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

  static double get_yaw(const geometry_msgs::msg::Quaternion & orientation)
  {
    return std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
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

  static double sector_min(const LaserScan & scan, double angle_min, double angle_max)
  {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const auto angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const auto range = static_cast<double>(scan.ranges[i]);
      if (angle >= angle_min && angle <= angle_max && std::isfinite(range) &&
        range >= scan.range_min && range <= scan.range_max)
      {
        nearest = std::min(nearest, range);
      }
    }
    return nearest;
  }

  std::pair<double, double> mline_coordinates(double x, double y) const
  {
    const auto dx = goal_.x - start_.x;
    const auto dy = goal_.y - start_.y;
    const auto length = std::hypot(dx, dy);
    const auto ux = dx / length;
    const auto uy = dy / length;
    const auto rx = x - start_.x;
    const auto ry = y - start_.y;
    return {rx * ux + ry * uy, ux * ry - uy * rx};
  }

  void follow_boundary(
    double front, double right, double left, double goal_heading_error,
    Twist & twist) const
  {
    const auto side = boundary_side_ > 0.0 ? right : left;
    if (front < obstacle_distance_ * 0.85) {
      twist.twist.angular.z = boundary_side_ * max_angular_speed_ * 0.75;
      return;
    }
    if (!std::isfinite(side)) {
      twist.twist.linear.x = boundary_speed_;
      twist.twist.angular.z = std::clamp(
        1.5 * goal_heading_error, -max_angular_speed_ * 0.6,
        max_angular_speed_ * 0.6);
      return;
    }

    const auto error = wall_distance_ - side;
    twist.twist.linear.x = boundary_speed_;
    twist.twist.angular.z = std::clamp(
      boundary_side_ * error * 1.8, -max_angular_speed_ * 0.65,
      max_angular_speed_ * 0.65);
  }

  bool valid_parameters() const
  {
    const auto goal_length = std::hypot(goal_.x - start_.x, goal_.y - start_.y);
    return std::isfinite(start_.x) && std::isfinite(start_.y) &&
      std::isfinite(goal_.x) && std::isfinite(goal_.y) && goal_length > 1e-6 &&
      std::isfinite(linear_speed_) && linear_speed_ > 0.0 &&
      std::isfinite(boundary_speed_) && boundary_speed_ > 0.0 &&
      std::isfinite(max_angular_speed_) && max_angular_speed_ > 0.0 &&
      std::isfinite(obstacle_distance_) && obstacle_distance_ > 0.0 &&
      std::isfinite(wall_distance_) && wall_distance_ > 0.0 &&
      std::isfinite(goal_tolerance_) && goal_tolerance_ > 0.0 &&
      std::isfinite(pose_timeout_) && pose_timeout_ > 0.0 &&
      std::isfinite(scan_timeout_) && scan_timeout_ > 0.0;
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
    marker.ns = "bug2";
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  MarkerArray make_markers(
    const rclcpp::Time & stamp, bool pose_fresh, bool scan_valid,
    const Point & robot, double yaw, double distance_to_goal) const
  {
    MarkerArray result;

    auto mline = make_marker(stamp, 0, Marker::LINE_STRIP);
    mline.scale.x = 0.045;
    set_color(mline, 0.1F, 0.45F, 1.0F);
    mline.points = {start_, goal_};
    result.markers.push_back(mline);

    auto goal = make_marker(stamp, 1, Marker::SPHERE);
    goal.pose.position = goal_;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.24;
    set_color(goal, 0.1F, 1.0F, 0.2F);
    result.markers.push_back(goal);

    auto robot_marker = make_marker(stamp, 2, Marker::ARROW);
    if (pose_fresh) {
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
    result.markers.push_back(robot_marker);

    auto scan_points = make_marker(stamp, 3, Marker::POINTS);
    scan_points.scale.x = scan_points.scale.y = 0.055;
    set_color(scan_points, 1.0F, 0.25F, 0.1F);
    if (pose_fresh && scan_valid) {
      for (std::size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
        const auto range = static_cast<double>(latest_scan_.ranges[i]);
        if (!std::isfinite(range) || range < latest_scan_.range_min ||
          range > latest_scan_.range_max)
        {
          continue;
        }
        const auto angle = yaw + latest_scan_.angle_min +
          static_cast<double>(i) * latest_scan_.angle_increment;
        scan_points.points.push_back(make_point(
          robot.x + range * std::cos(angle),
          robot.y + range * std::sin(angle), 0.02));
      }
    }
    result.markers.push_back(scan_points);

    auto hit = make_marker(stamp, 4, Marker::SPHERE);
    if (has_hit_point_) {
      hit.pose.position = hit_point_;
      hit.scale.x = hit.scale.y = hit.scale.z = 0.18;
      set_color(hit, 1.0F, 0.2F, 0.8F);
    } else {
      hit.action = Marker::DELETE;
    }
    result.markers.push_back(hit);

    auto status = make_marker(stamp, 5, Marker::TEXT_VIEW_FACING);
    if (pose_fresh) {
      status.pose.position = make_point(robot.x, robot.y, 0.6);
      status.scale.z = 0.19;
      set_color(status, 1.0F, 1.0F, 1.0F);
      const char * state_name = state_ == State::GO_TO_GOAL ?
        "GO TO GOAL" : (state_ == State::FOLLOW_BOUNDARY ?
        "FOLLOW BOUNDARY" : "GOAL REACHED");
      status.text = std::string(state_name) + "\ndistance: " +
        std::to_string(distance_to_goal).substr(0, 4) + " m";
    } else {
      status.action = Marker::DELETE;
    }
    result.markers.push_back(status);
    return result;
  }

  bpe::TypedConsumer<Pose> pose_consumer_;
  bpe::TypedConsumer<LaserScan> scan_consumer_;
  bpe::TypedProducer<Twist> command_producer_;
  bpe::TypedProducer<MarkerArray> markers_producer_;
  Pose latest_pose_;
  LaserScan latest_scan_;
  Point start_;
  Point goal_;
  Point hit_point_;
  std::string pose_input_;
  std::string scan_input_;
  std::string command_output_;
  std::string markers_output_;
  std::string frame_;
  double linear_speed_{0.35};
  double boundary_speed_{0.22};
  double max_angular_speed_{1.0};
  double obstacle_distance_{0.7};
  double wall_distance_{0.48};
  double goal_tolerance_{0.15};
  double pose_timeout_{0.5};
  double scan_timeout_{0.5};
  double hit_goal_distance_{0.0};
  double hit_mline_progress_{0.0};
  double boundary_side_{1.0};
  int64_t pose_received_ns_{0};
  int64_t scan_received_ns_{0};
  State state_{State::GO_TO_GOAL};
  bool have_pose_{false};
  bool have_scan_{false};
  bool has_hit_point_{false};
  bool has_left_mline_{false};
  double mline_tolerance_{0.10};
  double leave_progress_margin_{0.15};
  double leave_distance_margin_{0.15};
};

}  // namespace my_cpp_package

PLUGINLIB_EXPORT_CLASS(my_cpp_package::Bug2ControllerBlock, bpe::IBlockPlugin)
