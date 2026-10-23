# BPE Bug2 demo

This package contains ROS 2 / Block Plugin Engine (BPE) navigation demos. The
standalone Bug2 controller uses direct goal-heading and wall-following control.
The combined version uses Pure Pursuit to turn a target
point into linear and angular velocity commands. Bug2 is the planner: it
chooses the goal as the target while the M-line is clear, and switches to a
boundary-following target after an obstacle is detected. It resumes
goal-seeking after returning to the start-to-goal M-line closer to the goal
than the obstacle hit point.

The `TimedElasticBandBlock` demo is a separate, self-contained TEB-style local
planner/controller, not the upstream `teb_local_planner` package. It optimizes
a short sequence of robot poses against laser obstacle returns and smoothness,
then time-parameterizes the first segment into a velocity command. It does not
use Bug2 or Pure Pursuit.

The default example uses a Python differential-drive simulator. It creates a
round obstacle across the M-line and several randomized wall segments close
to, but clear of, the example robot's route. The simulator publishes pose and
laser data; BPE consumes those inputs, computes velocity commands, and
publishes visualization markers.

## Build and run

From the workspace root:

```bash
pixi run colcon build --packages-up-to my_cpp_package --symlink-install
pixi run bug2-bpe
```

Use `pixi run bug2-bpe` for combined Pure Pursuit + Bug2, `pixi run
bug2-standalone` for standalone Bug2, or `pixi run teb-bpe` for the timed
elastic band demo. Each command starts BPE, the example simulator, and RViz;
run only one at a time. Press Ctrl+C to stop.
The robot starts at `(0, 0)` and aims for `(4, 0)`. The RViz left-side
**Displays** panel has three entries:

1. **Ground grid (world frame)** gives you a sense of position and scale.
2. **Bug2 path, robot and live laser returns** shows the blue M-line (the
   direct start-to-goal line), green goal and robot arrow, red dots returned
   by the simulated laser, and a purple sphere marking where Bug2 first met
   an obstacle. White text beside the robot shows its state and distance to
   the goal.
3. **Example obstacle shapes** draws the actual simulated obstacles: yellow
   wall segments and a solid red circle. These are separate from the red
   laser dots, which show what the robot currently senses.

The robot arrow points in its heading direction. In the combined demo, the
cyan sphere marks the current Pure Pursuit target. In the TEB demo, a cyan
line shows its optimized local trajectory. Watch the status text:
**GO TO GOAL** means Bug2
has selected the goal; **FOLLOW BOUNDARY** means Bug2 has selected a local
obstacle-following behavior; **GOAL REACHED** means the robot stopped within
the goal tolerance. The standalone demo turns toward the goal and follows the
nearby wall directly; the combined demo uses Pure Pursuit to steer toward
Bug2's selected target. In RViz, expand a display in the left
panel to see its topic, and untick its checkbox to hide it. The fixed frame
must remain `world`.

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

The combined, standalone, and TEB pipelines use `config/bug2.yaml`,
`config/bug2_standalone.yaml`, and `config/timed_elastic_band.yaml`,
respectively:

| Block | Input/output | Blackboard label |
| --- | --- | --- |
| ROS input boundary | `/bug2/pose` (`geometry_msgs/msg/PoseStamped`) | `robot_pose` |
| ROS input boundary | `/bug2/scan` (`sensor_msgs/msg/LaserScan`) | `obstacle_scan` |
| `PurePursuitBug2Block`, `Bug2ControllerBlock`, or `TimedElasticBandBlock` | Combined Bug2/Pure Pursuit, standalone Bug2, or local timed-band optimization | `robot_pose`, `obstacle_scan`, `robot_command`, `bug2_markers` or `teb_markers` |
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
