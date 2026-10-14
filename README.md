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

## Learning how ROS topics connect to RViz

The separate `rviz_topic_demo` node is a small example of publishing
`visualization_msgs/msg/MarkerArray`. It publishes a fixed blue path, a moving
orange sphere, and a text label on `/rviz_demo/markers`. RViz subscribes to
that topic through its MarkerArray display; the display's Fixed Frame is
`world`, matching each marker's `header.frame_id`.

Build once, then start the node and its dedicated RViz configuration in
separate terminals:

```sh
pixi run build
pixi run rviz-demo-node
pixi run rviz-demo
```

Inspect the connection from another terminal:

```sh
pixi run ros2 topic list
pixi run ros2 topic info /rviz_demo/markers
pixi run ros2 topic echo /rviz_demo/markers --once
```

The node republishes markers every 100 ms. Each marker is identified by its
namespace and ID, so RViz updates the existing shape instead of adding another
one. The header frame must match RViz's Fixed Frame; the topic must match the
MarkerArray display's Topic setting.
