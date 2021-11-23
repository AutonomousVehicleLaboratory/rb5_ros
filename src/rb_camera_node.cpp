#include "rb_camera_node.h"
#include "rb_camera.h"


ros::Publisher cam_pub;

void publish_images(std::queue<cv::Mat> image_queue){
  sensor_msgs::Image cam_msg;
  cv_bridge::CvImage bridge;

  std_msgs::Header header;
  header.seq = 0;
  header.stamp = ros::Time::now();

  cv::Mat frame_rgb = image_queue.front();
  bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_rgb);
  bridge.toImageMsg(cam_msg);
  image_queue.pop();

  cam_pub.publish(cam_msg);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "rb_camera");
  ros::NodeHandle n;
  cam_pub = n.advertise<sensor_msgs::Image>("camera", 10);
  RbCamera cam;
  cam.init();

  ros::spin();
  ros::Rate rate(1000.0);
  while(ros::ok()){
    if (cam.data.data_ready){
      publish_images(cam.data.image_queue);
      cam.data.data_ready = false;
      rate.sleep();
    };
  }
  
  //return 0;
}

