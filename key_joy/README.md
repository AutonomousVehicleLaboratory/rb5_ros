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
- - - 
Copyright 2023 UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

