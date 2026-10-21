# BPE Bug2 demo

This package contains a ROS 2 / Block Plugin Engine (BPE) example of the
Bug2 obstacle-avoidance algorithm. It follows an M-line from a start point
to a goal. When its laser scan detects an obstacle ahead, it follows the
obstacle boundary, then returns to goal-seeking after it reaches the M-line
closer to the goal than the obstacle hit point.

The default example uses a Python differential-drive simulator. It creates a
round obstacle across the M-line and several randomized wall segments. The
simulator publishes pose and laser data; BPE consumes those inputs, computes
velocity commands, and publishes visualization markers.

## Build and run

From the workspace root:

```bash
pixi run colcon build --packages-up-to my_cpp_package --symlink-install
pixi run bug2-bpe
```

This starts BPE, the example simulator, and RViz. Press Ctrl+C to stop.
The robot starts at `(0, 0)` and aims for `(4, 0)`. RViz displays the blue
M-line, green goal and robot, red laser returns, and purple obstacle hit
point. The status marker reports whether the controller is going to the goal,
following a boundary, or has reached the goal. A separate RViz display draws
the circular obstacle as a solid red cylinder and randomized walls as thick
yellow line segments, so the obstacle geometry is distinct from the red
laser-return points.

The simulator uses a fixed random seed by default so each run has the same
wall arrangement. Change the number of walls or choose another seed:

```bash
pixi run ros2 launch my_cpp_package bug2.launch.py wall_count:=10 wall_seed:=42
```

For the controller without RViz:

```bash
pixi run ros2 launch my_cpp_package bug2.launch.py rviz:=false
```

## BPE pipeline and topics

The pipeline configuration is `config/bug2.yaml`:

| Block | Input/output | Blackboard label |
| --- | --- | --- |
| ROS input boundary | `/bug2/pose` (`geometry_msgs/msg/PoseStamped`) | `robot_pose` |
| ROS input boundary | `/bug2/scan` (`sensor_msgs/msg/LaserScan`) | `obstacle_scan` |
| `Bug2ControllerBlock` | Consumes pose and scan; produces command and markers | `robot_pose`, `obstacle_scan`, `robot_command`, `bug2_markers` |
| ROS output boundary | `/bug2/cmd_vel` (`geometry_msgs/msg/TwistStamped`) | `robot_command` |
| ROS output boundary | `/bug2/markers` (`visualization_msgs/msg/MarkerArray`) | `bug2_markers` |

The example simulator also publishes explicit obstacle shapes to
`/bug2/obstacles` as a `visualization_msgs/msg/MarkerArray`. This display is
only generated for the built-in example; with a real robot, obstacle geometry
is usually visualized from the sensor data or a map.

Blackboard labels connect BPE blocks internally; they are not ROS topic
names. The ROS boundary blocks translate between topics and those labels.
Inspect the running topics with:

```bash
pixi run ros2 topic list
pixi run ros2 topic echo /bug2/cmd_vel
pixi run ros2 topic echo /bug2/markers
```

## Supplying your own sensor input

Disable the built-in simulator, but leave the controller running:

```bash
pixi run ros2 launch my_cpp_package bug2.launch.py example:=false
```

Then run your robot, simulator, or other ROS nodes so they continuously
publish:

* `geometry_msgs/msg/PoseStamped` on `/bug2/pose`. Set `header.frame_id` to
  `world` (or the configured controller frame), use world coordinates for
  `pose.position`, and provide the robot heading as a quaternion.
* `sensor_msgs/msg/LaserScan` on `/bug2/scan`. The scan angles are relative
  to the robot, angle zero points forward, and positive angles turn left.
  Provide a positive `angle_increment`, valid `range_min`/`range_max`, and
  one range per beam. Invalid or out-of-range measurements may be `inf`.

The controller needs both streams to keep arriving; if either is stale for
more than 0.5 seconds it publishes a zero velocity command. The scan is
expected to be a planar scan in the robot's frame, with the forward direction
near angle zero. Keep topic types and blackboard types unchanged, or update
the matching ROS boundary `ros_type` in `config/bug2.yaml` if you use other
message types.

For example, while a compatible scan publisher is running, send repeated
poses for a robot initially facing along +X:

```bash
pixi run ros2 topic pub -r 10 /bug2/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"
```

Read the generated velocity commands on `/bug2/cmd_vel` and apply them to
your robot only after checking that the controller's frame, units, limits,
and stop behavior are appropriate for your setup.

## Important limitations

This is a small educational Bug2 implementation, not a robust navigation
stack or safety-rated controller. The example's random walls are placed
outside the main test route so the included scenario remains repeatable;
the circular obstacle demonstrates the hit-boundary-leave behavior. Real
robots need correctly transformed sensor data, collision checking, actuator
limits, and independent emergency-stop protection.
