#ifndef RB5_CAMERA_OCV
#define RB5_CAMERA_OCV
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <gst/gst.h>
#include <glib.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include <string>

class RbCamera{
  public:
    RbCamera();
    ~RbCamera();

    // data
    typedef struct _CustomData
    {
      GstElement *pipeline;
      GstElement *source;
      GstElement *convert;
      GstElement *capsfiltersrc;
      GstElement *capsfilterapp;
      GstElement *appsink;
    } CustomData;

    void init();
    //static GstFlowReturn processData(GstElement * sink, CustomData * data);

    //ros::NodeHandle n;
    //ros::Publisher cam_pub;

    // Gstream
    //GMainLoop *loop;
    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;
    
    uint camera_id;
    uint width;
    uint height;
    uint frame_rate;

    std::string input_format;
    std::string output_format;


    CustomData data;
};

#endif
