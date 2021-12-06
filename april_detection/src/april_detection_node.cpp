#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include "april_detection.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

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

const cv::Mat d(cv::Size(1, 5), CV_64FC1, distortion_coeff);
const cv::Mat K(cv::Size(3, 3), CV_64FC1, intrinsics);

cv::Mat rectify(const cv::Mat image){
  cv::Mat image_rect = image.clone();
  const cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, d, image.size(), 1.0); 
  cv::undistort(image, image_rect, K, d, new_K); 

  return image_rect;
}

tf::Transform retrieveTransform(vector<apriltag_pose_t> poses){
  tf::Quaternion q;
  tf::Matrix3x3 so3_mat;
  tf::Transform tf;
  static tf::TransformBroadcaster br;

  for (int i=0; i<poses.size(); i++){

    // translation
    tf.setOrigin(tf::Vector3(poses[i].t->data[0],
                             poses[i].t->data[1],
                             poses[i].t->data[2]));
    // orientation - SO(3)
    so3_mat.setValue(poses[i].R->data[0], poses[i].R->data[1], poses[i].R->data[2],
                     poses[i].R->data[3], poses[i].R->data[4], poses[i].R->data[5], 
                     poses[i].R->data[6], poses[i].R->data[7], poses[i].R->data[8]);

    double roll, pitch, yaw; 

    // orientation - q
    so3_mat.getRPY(roll, pitch, yaw); // so3 to RPY
    cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << endl;
    cout << "t: " << poses[i].t->data[0] << " " << poses[i].t->data[1] << " " << poses[i].t->data[2] << endl;
    q.setRPY(roll, pitch, yaw);

    tf.setRotation(q);
    //br.sendTransform();
    
  }
  

  return tf;
}


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("image received");
  cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg);

  // rectify and run detection (pair<vector<apriltag_pose_t>, cv::Mat>)
  auto april_obj =  det.processImage(rectify(img_cv->image));

  retrieveTransform(april_obj.first);
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
