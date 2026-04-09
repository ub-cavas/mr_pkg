This is the ROS2 Mixed Reality Bridge

Step 1:

Clone this repo into `/autoware/src/universe/external`

Step 2:

Build this package `colcon build`

Step 3:

For the internal dual-EKF localization flow, start:

`ros2 launch mr_pkg localization.launch.py`

For Autoware localization on `/localization/kinematic_state`, start:

`ros2 launch mr_pkg autoware_localization.launch.py`

To convert `/localization/kinematic_state` into meter offsets on `/ub_mr/localization`:

`ros2 run mr_pkg kinematic_state_receiver`

This node treats `Odometry.pose.pose.position.x` and `.y` as MGRS offsets in
meters from the current 100 km grid square origin. The hardcoded map origin
latitude and longitude in `mr_pkg/kinematic_state_receiver.py` are converted
into that same MGRS square offset, and the node publishes the difference to
`/ub_mr/localization`.

Step 4:

Start the Unity Pakage: TODO


Notes:

If topics are not listing with `ros2 topic list` 

Try restarting the daemon

`ros2 daemon stop`

`ros2 daemon start`
