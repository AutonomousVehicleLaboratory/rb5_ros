#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "april_detection.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pose_pub;
ros::Subscriber image_sub;
AprilDetection det;

double distortion_coeff[] = {0.022327, 
                             -0.019742, 
			     -0.000961, 
			     0.000625, 
			     0.000000};
double intrinsics[] = {691.01615,    0.     ,  954.51,
                       0.     ,  690.10114,  540.77467,
                       0.     ,    0.     ,    1.};

cv::Mat d(cv::Size(1, 5), CV_64FC1, distortion_coeff);
cv::Mat K(cv::Size(3, 3), CV_64FC1, intrinsics);

cv::Mat rectify(const cv::Mat image){
  cv::Mat image_rect = image.clone();
  //cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, d, image.size(), 1.0); 
  cv::undistort(image, image_rect, K, d, K); 

  return image_rect;
}
void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("image received");
  cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg);

  // rectify
    

  // run detection
  det.processImage(rectify(img_cv->image));

  geometry_msgs::PoseStamped pose_msg;
  pose_pub.publish(pose_msg);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "april_detection");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/april_poses", 10); 
  image_sub = n.subscribe("/camera_0", 1, imageCallback);
  
  ros::spin();
  return 0;
  
}
