#ifndef RB5_CAMERA
#define RB5_CAMERA
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <gst/gst.h>
#include <glib.h>
#include <fastcv.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"

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
      GstElement *filter;
      GstElement *tee;
      GstElement *appsink;
      GstElement *videosink;
    } CustomData;

    void init();
    //static GstFlowReturn processData(GstElement * sink, CustomData * data);

    //ros::NodeHandle n;
    //ros::Publisher cam_pub;

    // Gstream
    //GMainLoop *loop;
    //GstElement *pipeline, *source, *convert, *frame_filter, *tee, *sink, *video_sink;
    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;
    gint8 camera_id;


    CustomData data;
};

#endif
