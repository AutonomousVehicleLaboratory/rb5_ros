#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>

using namespace cv;

std::deque<Mat> frameQueue;

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


/* The appsink has received a buffer */
static GstFlowReturn
new_sample (GstElement * sink, CustomData * data)
{
  GstSample *sample;
  GstBuffer *app_buffer, *buffer;

  /* Retrieve the buffer */
  g_signal_emit_by_name (sink, "pull-sample", &sample);
  // sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
  GstMapInfo map_info;

  if (sample) {
    /* The only thing we do in this example is print a * to indicate a received buffer */
    g_print ("#");
    
    // sample = gst_app_sink_pull_sample (GST_APP_SINK (sink));
    buffer = gst_sample_get_buffer (sample);
    GstCaps *caps = gst_sample_get_caps(sample);

    if (!gst_buffer_map ((buffer), &map_info, GST_MAP_READ)) {
      gst_buffer_unmap ((buffer), &map_info);
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    // gst_opencv_parse_cv_mat_params_from_caps()
    // g_print ("%s\n", gst_caps_to_string(caps));
    //  frame = Mat::zeros(1080, 1920, CV_8UC3);
    Mat frame_rgb = Mat::zeros(480, 640, CV_8UC3);
    Mat frame(Size(640, 480), CV_8UC4, (char*)map_info.data, Mat::AUTO_STEP);

    cvtColor(frame, frame_rgb, cv::COLOR_RGBA2BGR);
    // frame = cv::Mat(480, 640, CV_8UC3, (char *)map_info.data,
    //         cv::Mat::AUTO_STEP);
    //memcpy(frame.data,map_info.data,map_info.size);
    // frameQueue.push_back(frame);
    // frameQueue.pop()

    imshow("Display window", frame_rgb);

    int k = waitKey(20); // Wait for a keystroke in the window

    /* make a copy */
    app_buffer = gst_buffer_copy (buffer);

    // GstStructure *sample_info;
    // sample_info = gst_sample_get_info (sample);
    // if (sample_info) {
    //   GST_LOG ("structure is %" GST_PTR_FORMAT, sample_info);
    // }

    gst_sample_unref (sample);
    return GST_FLOW_OK;
  }

  return GST_FLOW_ERROR;
}

int
main (int argc, char *argv[])
{
  CustomData data;
  GstBus *bus;
  GstMessage *msg;
  GstStateChangeReturn ret;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  /* Create the elements */
  data.source = gst_element_factory_make ("autovideosrc", "source");
  data.convert = gst_element_factory_make ("videoconvert", "convert");
  data.appsink = gst_element_factory_make ("appsink", "sink");
  data.videosink = gst_element_factory_make ("autovideosink", "sink");
  data.filter = gst_element_factory_make ("vertigotv", "filter");
  data.tee = gst_element_factory_make ("tee", "tee");

  /* Create the empty pipeline */
  data.pipeline = gst_pipeline_new ("test-pipeline");

  if (!data.pipeline || !data.convert || !data.source || !data.appsink || 
      !data.filter || !data.videosink) {
    g_printerr ("Not all elements could be created.\n");
    return -1;
  }

  /* Build the pipeline */
  gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.convert, data.filter, data.appsink, NULL);
  if (gst_element_link (data.source, data.convert) != TRUE ||
      gst_element_link (data.convert, data.filter) != TRUE ||
      gst_element_link (data.filter, data.appsink) != TRUE) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }

  /* Modify the source's properties */
  // g_object_set (data.source, "pattern", 0, NULL);
  // g_object_set(G_OBJECT (data.source));

  g_object_set (data.appsink, "emit-signals", TRUE, NULL);
  g_signal_connect (data.appsink, "new-sample", G_CALLBACK (new_sample),
    &data);

  /* Start playing */
  ret = gst_element_set_state (data.pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }

  /* Wait until error or EOS */
  bus = gst_element_get_bus (data.pipeline);
  msg =
      gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  /* Parse message */
  if (msg != NULL) {
    GError *err;
    gchar *debug_info;

    switch (GST_MESSAGE_TYPE (msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error (msg, &err, &debug_info);
        g_printerr ("Error received from element %s: %s\n",
            GST_OBJECT_NAME (msg->src), err->message);
        g_printerr ("Debugging information: %s\n",
            debug_info ? debug_info : "none");
        g_clear_error (&err);
        g_free (debug_info);
        break;
      case GST_MESSAGE_EOS:
        g_print ("End-Of-Stream reached.\n");
        break;
      default:
        /* We should not reach here because we only asked for ERRORs and EOS */
        g_printerr ("Unexpected message received.\n");
        break;
    }
    gst_message_unref (msg);
  }

  /* Free resources */
  gst_object_unref (bus);
  gst_element_set_state (data.pipeline, GST_STATE_NULL);
  gst_object_unref (data.pipeline);
  return 0;
}
