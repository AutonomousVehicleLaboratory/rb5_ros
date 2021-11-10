#ifndef RB5_CAMERA
#define RB5_CAMERA
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <gst/gst.h>
#include <glib.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"

class RbCamera{
  public:
    RbCamera();

    void processData();

    ros::NodeHandle n;
    ros::Publisher cam_pub;

};

#endif
