# ORB_SLAM3_RB5

This is the ROS examples from ORB_SLAM3. We modify the Monocular version and tested it on RB5.

Compile:
1. Make sure you have the ORB_SLAM3 compiled, the orignal version from the official repository might have errors on RB5. We recommend use the version hosted on our github repository.
2. Once that is compiled, Download this repository to a ros workspace. Modify the CMakeLists.txt file, set the ORB_SLAM3_SOURCE_DIR to where the ORB_SLAM3 source code is.
3. Compile the package and run it. An example is given here

```
rosrun ORB_SLAM3_RB5 Mono path/to/ORB_SLAM3_RB5/Vocabulary/ORBvoc.txt path/to/ORB_SLAM3_RB5/Examples_old/Monocular/EuRoC.yaml
···