# ROS 2 RViz demo

Install the Pixi environment and build the package:

```sh
pixi install
pixi run build
```

Run the node and RViz in separate terminals from the workspace:

```sh
pixi run hello-cpp
pixi run rviz
```

The node reads three 2D waypoints from ROS parameters and uses a pure-pursuit
controller to steer a simulated turtle along the two resulting segments. It
stops at the final waypoint. RViz shows the path in blue, waypoints in orange,
the lookahead point in magenta, the path projection in yellow, the pursuit
chord in magenta, and the commanded curvature arc in cyan. A text marker shows
lookahead distance, curvature, angular speed, and progress. The default points
are `(0, 0)`, `(2, 0)`, and `(2, 2)`.

Set different points when launching the node, for example:

```sh
pixi run pursuit \
  -p point1.x:=0.0 -p point1.y:=0.0 \
  -p point2.x:=3.0 -p point2.y:=1.0 \
  -p point3.x:=4.0 -p point3.y:=4.0 \
  -p lookahead_distance:=0.4 \
  -p linear_speed:=0.8 \
  -p max_angular_speed:=3.0 \
  -p update_rate:=20.0 \
  -p goal_tolerance:=0.1 \
  -p debug_arc_duration:=3.0
```

`initial_pose.x`, `initial_pose.y`, and `initial_pose.yaw` can also be set; by
default the turtle starts at the first waypoint, facing along the first
segment. The two adjacent points in a segment must be different, and
`lookahead_distance`, `linear_speed`, `max_angular_speed`, `update_rate`,
`goal_tolerance`, and `debug_arc_duration` must be positive. A smaller
lookahead distance creates tighter turns; the default `0.4 m` makes cornering
more apparent on the example route. The cyan debug arc previews the commanded
turn for `debug_arc_duration` seconds.

## BPE Bug2 obstacle-avoidance demo

See [`src/my_cpp_package/README.md`](src/my_cpp_package/README.md) for full
run instructions, the BPE topic/blackboard wiring, and guidance for supplying
your own pose and laser-scan inputs.

The Bug2 BPE pipeline consumes `PoseStamped` and `LaserScan` inputs, publishes
`TwistStamped` velocity commands, and visualizes the M-line, obstacle scan,
robot, hit point, and goal in RViz. Its predefined route is from `(0, 0)` to
`(4, 0)`, with a simulated circular obstacle in the way. The controller
switches between moving toward the goal and following the obstacle boundary;
it leaves the boundary when it reaches the M-line closer to the goal than its
hit point.

Build and run the demo with its Python differential-drive and obstacle
simulator plus RViz:

```sh
pixi run colcon build --packages-up-to my_cpp_package --symlink-install
pixi run bug2-bpe
```

To connect your own pose and laser scan publishers, disable the example:

```sh
pixi run ros2 launch my_cpp_package bug2.launch.py example:=false
```

Publish `geometry_msgs/msg/PoseStamped` to `/bug2/pose` and
`sensor_msgs/msg/LaserScan` to `/bug2/scan`; pose and scan timestamps must
continue arriving or the controller stops its commands. Velocity commands
are published on `/bug2/cmd_vel` and RViz markers on `/bug2/markers`. This is
a learning simulation, not a safety-rated controller.
