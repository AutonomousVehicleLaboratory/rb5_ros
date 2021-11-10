#include "rb_camera.h"


RbCamera::RbCamera(){
  cam_pub = n.advertise<sensor_msgs::Image>("camera", 10);
}

void RbCamera::processData(){
  sensor_msgs::Image cam_msg;
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

