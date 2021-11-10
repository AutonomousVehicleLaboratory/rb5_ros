#include "rb_camera.h"


RbCamera::RbCamera(){
  cam_pub = n.advertise<sensor_msgs::Image>("camera", 10);

  gst_init(0, nullptr);
  loop = g_main_loop_new(NULL, FALSE);

  pipeline    = gst_pipeline_new("video-display");
  source      = gst_element_factory_make("qtiqmmfsrc", "qmmf-source");
  framefilter = gst_element_factory_make("capsfilter", "frame-filter");
  sink        = gst_element_factory_make("waylandsink","display");
  if (!pipeline || !source || !framefilter || !sink) {
    g_printerr ("Create element failed.\n");
  }

  g_object_set (G_OBJECT(source), "camera", 0, nullptr);

  if (camera_id == 0) {
    g_object_set(G_OBJECT(framefilter), "caps",
                 gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"), nullptr);
  }
  g_object_set (G_OBJECT(sink), "x", 0, "y", 200, "width", 640, "height", 360, nullptr);
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));

}


void RbCamera::processData(){
  sensor_msgs::Image cam_msg;
  //fcvOperationMode mode = FASTCV_OP_PERFORMANCE;
  //fcvSetOperationMode(mode);

  cam_pub.publish(cam_msg);

}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "rb_camera");
  RbCamera cam;

  //ros::spin();
  ros::Rate rate(10.0);
  while(ros::ok()){
    cam.processData();
    rate.sleep();
  }
  return 0;
}

