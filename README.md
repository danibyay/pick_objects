# pick_objects

ROS node that publishes a navigation goal in terms of the position and orientation.

Possible arguments: x, y, z, w.

This node is programmed to send 2 navigation goals, but can be easily modified to send more and change the coordinates of the goals.

With the help of a wrapper project that uses amcl, like [home_service_robot](https://github.com/danibyay/home_service_robot), this node can be tested.


## Usage

`catkin_make`
`source devel/setup.bash`
`roslaunch pick_objects pick_objects.launch`


## Messages

This node uses a **MoveBaseGoal** message, that extends from **geometry_msgs/PoseStamped**

To see the message formatted when setting a nav goal in RViz, you can type

`rostopic echo move_bas_simple/goal`
