/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

static int cameraWidth, cameraHeight;

class ImageGrabber
{
public:
    int count = 0;
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}
    
    void PublishPose(
        const Sophus::SE3f& T, 
        const ros::Time& stamp,
        const string& frame_id, 
        const string& child_frame_id,
        tf::TransformBroadcaster& tf_br
    );
    
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    ORB_SLAM3::System *mpSLAM;

    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ROS_INFO("Started SLAM system setup");
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    ROS_INFO("Finished SLAM system setup");

    // Load camera parameters from settings file
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    cameraWidth = fSettings["Camera.width"].operator int();
    cameraHeight = fSettings["Camera.height"].operator int();

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera_0", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("A message is received");
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ROS_INFO("Resize Image");
    cv::Mat img;
    
    if (cv_ptr->image.cols != cameraWidth or 
        cv_ptr->image.rows != cameraHeight) {
        ROS_INFO("Resize image from (%d, %d) to (%d, %d).", cv_ptr->image.cols, cv_ptr->image.rows, cameraWidth, cameraHeight);
        cv::resize(cv_ptr->image, img, cv::Size(cameraWidth, cameraHeight), cv::INTER_LINEAR);
    }
    else{
        img = cv_ptr->image;
    }

    // ROS_INFO("track using this image");
    Sophus::SE3f se3_tf = mpSLAM->TrackMonocular(img, cv_ptr->header.stamp.toSec());
    // ROS_INFO("image tracked");

    cout << se3_tf.rotationMatrix() << endl;
    cout << se3_tf.translation() << endl;

    string frame_id("map");
    string child_frame_id("camera_");
    child_frame_id = child_frame_id + to_string(count);
    count += 1;
    ros::Time stamp = msg->header.stamp;
    static tf::TransformBroadcaster br;
    PublishPose(se3_tf, stamp, frame_id, child_frame_id, br);
}

// code reference
// https://github.com/uzh-rpg/rpg_vikit/blob/10871da6d84c8324212053c40f468c6ab4862ee0/vikit_ros/src/output_helper.cpp#L15 
void ImageGrabber::PublishPose(
    const Sophus::SE3f& T, 
    const ros::Time& stamp,
    const string& frame_id, 
    const string& child_frame_id,
    tf::TransformBroadcaster& tf_br)
{
    tf::Transform transform_msg;
    Eigen::Quaternionf q(T.rotationMatrix());
    transform_msg.setOrigin(tf::Vector3(T.translation().x(), T.translation().y(), T.translation().z()));
    tf::Quaternion tf_q; tf_q.setX(q.x()); tf_q.setY(q.y()); tf_q.setZ(q.z()); tf_q.setW(q.w());
    transform_msg.setRotation(tf_q);
    tf_br.sendTransform(tf::StampedTransform(transform_msg, stamp, frame_id, child_frame_id));
}
