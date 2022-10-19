#ifndef APRIL_DETECTION_H
#define APRIL_DETECTION_H
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/getopt.h>
#include <bits/stdc++.h>

using namespace std;
class AprilDetection{
  public:
    AprilDetection();
    ~AprilDetection();

    // process input image
    tuple<vector<apriltag_pose_t>, vector<int>, cv::Mat> processImage(cv::Mat image);
    void setInfo(double tagSize, double fx, double fy, double cx, double cy);
    
    // variables
    apriltag_detection_info_t info;

  private:
    apriltag_detector_t *a_detector;
    apriltag_family_t *tf;


};

#endif // APRIL_DETECTION_H
