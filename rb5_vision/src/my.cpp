#include "rb_camera.h"


ros::Publisher cam_pub;

static GstFlowReturn processData(GstElement * sink, RbCamera::CustomData * data){
  sensor_msgs::Image cam_msg;
  cv_bridge::CvImage bridge;

  GstSample *sample;
  GstBuffer *buffer;
  GstMapInfo map_info;

  // Retrieve buffer
  g_signal_emit_by_name (sink, "pull-sample", &sample);

  if(sample){
    buffer = gst_sample_get_buffer (sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *caps_structure = gst_caps_get_structure(caps, 0);

    gboolean res;
    int width, height;
    const gchar *format;
    res = gst_structure_get_int (caps_structure, "width", &width);
    res |= gst_structure_get_int (caps_structure, "height", &height);
    format = gst_structure_get_string (caps_structure, "format");
    if (!res) {
        g_print("no dimensions");
    }

    g_print("%s\n", gst_structure_to_string(caps_structure));

    if (!gst_buffer_map ((buffer), &map_info, GST_MAP_READ)) {
      gst_buffer_unmap ((buffer), &map_info);
      gst_sample_unref(sample);

      return GST_FLOW_ERROR;
    }


    // cv::Mat frame_rgb = cv::Mat::zeros(width, height, CV_8UC3);
    cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (char*)map_info.data, cv::Mat::AUTO_STEP);
    // cv::cvtColor(frame, frame_rgb, cv::COLOR_RGB2);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_rgb);
    bridge.toImageMsg(cam_msg);
    


    // Publish camera frame and free resources
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    cam_pub.publish(cam_msg);

    return GST_FLOW_OK;
  }

  //fcvOperationMode mode = FASTCV_OP_PERFORMANCE;
  //fcvSetOperationMode(mode);
  return GST_FLOW_ERROR;


}

RbCamera::RbCamera(){

  gst_init(0, nullptr);
  camera_id = 0;
  //loop = g_main_loop_new(NULL, FALSE);

  data.source        = gst_element_factory_make("qtiqmmfsrc", "source");
  data.capsfiltersrc = gst_element_factory_make("capsfilter", "capsfiltersrc");
  data.convert        = gst_element_factory_make ("videoconvert", "convert");
  data.capsfilterapp = gst_element_factory_make("capsfilter", "capsfilterapp");
  data.appsink       = gst_element_factory_make ("appsink", "sink");
  data.pipeline      = gst_pipeline_new("rb5-camera");

  if (!data.pipeline || !data.convert || !data.source || !data.appsink ||
      !data.capsfiltersrc || !data.capsfilterapp ) {
    g_printerr ("Not all elements could be created.\n");
    return;
  }


  // Build pipeline
  gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.convert, data.appsink, data.capsfiltersrc, data.capsfilterapp, nullptr);
  if(gst_element_link(data.source, data.capsfiltersrc) != TRUE)
    std::cout << "Could not link source & capsfiltersrc" << std::endl;
  if(gst_element_link(data.capsfiltersrc, data.convert) != TRUE)
    std::cout << "Could not link capsfiltersrc & convert" << std::endl;
  if(gst_element_link(data.convert, data.capsfilterapp) != TRUE)
    std::cout << "Could not link convert and capsfilterapp" << std::endl;
  if(gst_element_link(data.capsfilterapp, data.appsink) != TRUE)
    std::cout << "Could not link capsfilterapp and appsink" << std::endl;

  //if (gst_element_link_many (data.source, data.capsfiltersrc, data.convert, data.capsfilterapp, data.appsink, nullptr) != TRUE) {
  //  g_printerr ("Elements could not be linked.\n");
  //  gst_object_unref (data.pipeline);
  //  return;
  //}


  g_object_set(G_OBJECT(data.capsfilterapp), "caps",
		  gst_caps_from_string("video/x-raw,format=RGB"), nullptr);

  if (camera_id == 0) {
    g_object_set(G_OBJECT(data.capsfiltersrc), "caps",
                 gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"), nullptr);
  }
  else{
    g_object_set(G_OBJECT(data.capsfiltersrc), "caps",
                 gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1280,height=720"), nullptr);
  }

  g_object_set (G_OBJECT(data.source), "camera", camera_id, nullptr);
  //g_object_set (G_OBJECT(sink), "x", 0, "y", 200, "width", 640, "height", 360, nullptr);
  //bus = gst_pipeline_get_bus(GST_PIPELINE(data.pipeline));
  //

}

RbCamera::~RbCamera(){
  gst_object_unref(bus);
  gst_element_set_state(data.pipeline, GST_STATE_NULL);
  gst_object_unref(data.pipeline);
}

void RbCamera::init(){
  g_object_set(data.appsink, "emit-signals", TRUE, nullptr);
  g_object_set(G_OBJECT(data.source), "camera", camera_id, NULL);
  g_signal_connect(data.appsink, "new-sample", G_CALLBACK(processData), &data);

  // play
  ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
  if(ret == GST_STATE_CHANGE_FAILURE){
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (data.pipeline);
    return;
  }
  bus = gst_element_get_bus (data.pipeline);

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
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "rb_camera");
  ros::NodeHandle n;
  cam_pub = n.advertise<sensor_msgs::Image>("camera", 10);
  RbCamera cam;
  cam.init();

  //ros::spin();
  //ros::Rate rate(10.0);
  //while(ros::ok()){
  //  cam.processData();
  //  rate.sleep();
  //}
  //return 0;
}

