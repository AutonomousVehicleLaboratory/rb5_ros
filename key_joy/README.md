### An Example ROS node that publishes the Joy message. 

You can use this keyboard to control the robot.
'w','s': straight motion
'a','d': slide motion
'q','e': rotate motion
'esc', 'ctrl+c':quit

Use the following command to make the python node executable
```
chmod +x src/key_joy_node.py
```

Remember to run roscore in another terminal
```
source /opt/ros/melodic/setup.bash
roscore
```

Then in a new terminal, go to the workspace you have this repository in,
source the workspace.
```
source devel/setup.bash
```

then you can run the node use "rospack find key_joy" if you cannot find this node
```
rosrun key_joy key_joy_node.py
```

For HW1, you can replace the key_joy_node.py with your waypoint following controller. But still publish the Joy message.

