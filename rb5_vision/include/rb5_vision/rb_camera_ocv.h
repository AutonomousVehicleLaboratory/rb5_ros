/*

Copyright 2023, UC San Diego, Contextual Robotics Institute

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

*/

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
    RbCamera(
      int camera_id_,
      int width_,
      int height_,
      int frame_rate_,
      std::string input_format_,
      std::string output_format_
    );
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

    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;
    
    // caps parameters for the camera
    int camera_id;
    int width;
    int height;
    int frame_rate;

    std::string input_format;
    std::string output_format;


    CustomData data;
};

#endif
