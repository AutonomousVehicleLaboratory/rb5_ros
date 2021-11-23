#ifndef RB5_CAMERA
#define RB5_CAMERA

#include <stdio.h>
#include <sstream>
#include <gst/gst.h>
#include <glib.h>
#include <fastcv.h>
#include "opencv2/opencv.hpp"
#include <queue>


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

      gboolean data_ready;
      std::queue<cv::Mat> image_queue;
      
    } CustomData;

    void init();

    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;
    gint8 camera_id;

    CustomData data;
};

#endif
