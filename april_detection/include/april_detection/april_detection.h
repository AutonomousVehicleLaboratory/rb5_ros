/*
Copyright 2023, UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
