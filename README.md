This is the ROS2 Mixed Reality Bridge

Step 1:

Clone this repo into `/autoware/src/universe/external`

Step 2:

Build this package `colcon build`

Step 3:

Start the package `ros2 launch mr_pkg localization.py` 

Step 4:

Start the Unity Pakage: TODO


Notes:

If topics are not listing with `ros2 topic list` 

Try restarting the daemon

`ros2 daemon stop`

`ros2 daemon start`