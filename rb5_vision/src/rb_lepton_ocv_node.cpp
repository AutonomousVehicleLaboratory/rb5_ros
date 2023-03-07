/* Modification to rb_camera_ocv_node to work with 
FLIR Lepton 3.5 thermal camera with PureThermal2 board */

#include "rb_camera_ocv.h"
#include <sys/time.h>

static int _camera_id;
static int _width;
static int _height;
static int _frame_rate;
static bool _image_compress;

std::string _input_format;
std::string _output_format;
std::string _topic_name;
std::string _device_path;

ros::Publisher cam_pub, cam_compress_pub;


/* Callback for appsink to parse the video stream and publish images. */
static GstFlowReturn processData(GstElement * sink, RbCamera::CustomData * data){

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

    // g_print("%s\n", gst_structure_to_string(caps_structure));

    if (!gst_buffer_map ((buffer), &map_info, GST_MAP_READ)) {
      gst_buffer_unmap ((buffer), &map_info);
      gst_sample_unref(sample);

      return GST_FLOW_ERROR;
    }

    timeval current_time;
    gettimeofday(&current_time, 0);
    std::cout << current_time.tv_sec << "." << current_time.tv_usec << std::endl;

    // Parse data from buffer, depending on the format, conversion might be needed.
    // cv::Mat frame_rgb = cv::Mat::zeros(width, height, CV_8UC3);
    cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (char*)map_info.data, cv::Mat::AUTO_STEP);
    // cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2);

    // Prepare ROS message.
    cv_bridge::CvImage bridge;

    // Publish camera image
    sensor_msgs::Image cam_msg;
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_rgb);
    bridge.toImageMsg(cam_msg);
    cam_pub.publish(cam_msg);
    
    // Publish camera frame 
    if (_image_compress) {
      sensor_msgs::CompressedImage cam_compress_msg;
      cam_compress_msg.header.stamp = header.stamp;
      std::vector<int> p;
      p.push_back(cv::IMWRITE_JPEG_QUALITY);
      p.push_back(90);
      
      cv::Mat frame_bgr = cv::Mat::zeros(width, height, CV_8UC3);
      cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
      cv::imencode(".jpg", frame_bgr, cam_compress_msg.data, p);

      cam_compress_pub.publish(cam_compress_msg);
    }

    // free resources
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }


  return GST_FLOW_ERROR;
}

RbCamera::RbCamera(
  int camera_id_,
  int width_,
  int height_,
  int frame_rate_,
  std::string input_format_,
  std::string output_format_
){

  gst_init(0, nullptr);

  camera_id = camera_id_;
  width = width_;
  height = height_;
  frame_rate = frame_rate_;
  input_format = input_format_;
  output_format = output_format_;

  _device_path = "/dev/video" + std::to_string(camera_id);
  std::string input_caps = "video/x-raw,format=" + input_format + 
                           ",framerate=" + std::to_string(frame_rate) + "/1" + 
                           ",width=" + std::to_string(width) + 
                           ",height=" + std::to_string(height);
  
  std::string output_caps = "video/x-raw,format=" + output_format;

  std::cout << "device path: " << _device_path << std::endl;
  std::cout << "lepton input caps: " << input_caps << std::endl;
  std::cout << "output caps: " << output_caps << std::endl;

  data.source        = gst_element_factory_make("v4l2src", "source");
  data.capsfiltersrc = gst_element_factory_make("capsfilter", "capsfiltersrc");
  data.convert        = gst_element_factory_make ("videoconvert", "convert");
  data.capsfilterapp = gst_element_factory_make("capsfilter", "capsfilterapp");
  data.appsink       = gst_element_factory_make ("appsink", "sink");
  data.pipeline      = gst_pipeline_new("rb5-lepton");

  if (!data.pipeline || !data.convert || !data.source || !data.appsink ||
     !data.capsfiltersrc || !data.capsfilterapp ) {
    g_printerr ("Not all elements could be created.\n");
    if (!data.pipeline) {g_printerr ("pipeline could not be created!\n");}
    if (!data.capsfiltersrc) {g_printerr ("capsfiltersrc could not be created!\n");}
    if (!data.convert) {g_printerr ("convert could not be created!\n");}
    if (!data.capsfilterapp) {g_printerr ("capsfilterapp could not be created!\n");}
    if (!data.appsink) {g_printerr ("appsink could not be created!\n");}
    if (!data.source) {g_printerr ("source could not be created!\n");}
    return;
  }

  // Build pipeline
  gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.convert, data.appsink, data.capsfiltersrc, data.capsfilterapp, nullptr);
  if (gst_element_link_many (data.source, data.capsfiltersrc, data.convert, data.capsfilterapp, data.appsink, nullptr) != TRUE) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return;
  }

  g_object_set(G_OBJECT(data.capsfiltersrc), "caps",
               gst_caps_from_string(input_caps.c_str()), nullptr);
  g_object_set(G_OBJECT(data.capsfilterapp), "caps",
		           gst_caps_from_string(output_caps.c_str()), nullptr);


}

RbCamera::~RbCamera(){
  gst_object_unref(bus);
  gst_element_set_state(data.pipeline, GST_STATE_NULL);
  gst_object_unref(data.pipeline);
}

void RbCamera::init(){
  g_object_set(data.appsink, "emit-signals", TRUE, nullptr);
  g_object_set(G_OBJECT(data.source), "device", _device_path.c_str(), NULL);
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
  ros::NodeHandle private_nh("~");

  private_nh.param("camera_id", _camera_id, 0);
  ROS_INFO("camera_id: %d", _camera_id);
  private_nh.param("frame_rate", _frame_rate, 30);
  ROS_INFO("frame_rate: %d", _frame_rate);
  private_nh.param("width", _width, 1920);
  ROS_INFO("width: %d", _width);
  private_nh.param("height", _height, 1080);
  ROS_INFO("height: %d", _height);
  private_nh.param<std::string>("input_format", _input_format, "NV12");
  ROS_INFO("input_format: %s", _input_format.c_str());
  private_nh.param<std::string>("output_format", _output_format, "RGB");
  ROS_INFO("output_format: %s", _output_format.c_str());
  private_nh.param<std::string>("topic_name", _topic_name, std::string("camera")+std::to_string(_camera_id));
  ROS_INFO("topic_name: %s", _topic_name.c_str());
  private_nh.param("image_compress", _image_compress, true);
  ROS_INFO("image_compress: %d", _image_compress);

  cam_pub = n.advertise<sensor_msgs::Image>(_topic_name.c_str(), 10);
  if (_image_compress) {
    cam_compress_pub = n.advertise<sensor_msgs::CompressedImage>((_topic_name+std::string("/compressed")).c_str(), 10);
  }

  RbCamera cam(
    _camera_id,
    _width,
    _height,
    _frame_rate,
    _input_format,
    _output_format
  );
  cam.init();
}

