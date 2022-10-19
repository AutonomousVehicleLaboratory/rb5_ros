#include "april_detection.h"


AprilDetection::AprilDetection(){

  a_detector = apriltag_detector_create();

  // tag36h11
  tf = tag36h11_create();

  //getopt_t *getopt = getopt_create();
  //const char *famname = getopt_get_string(getopt, "family");

  // configure detector
  //apriltag_detector_add_family_bits(a_detector, tf, getopt_get_int(getopt, "hamming"));

  //a_detector->quad_decimate = getopt_get_double(getopt, "decimate");
  //a_detector->quad_sigma = getopt_get_double(getopt, "blur");
  //a_detector->nthreads = getopt_get_int(getopt, "threads");
  //a_detector->debug = getopt_get_bool(getopt, "debug");
  //a_detector->refine_edges = getopt_get_bool(getopt, "refine-edges");

  apriltag_detector_add_family(a_detector, tf);
  

  return;
}

AprilDetection::~AprilDetection(){
  return;
}

tuple<vector<apriltag_pose_t>, vector<int>, cv::Mat> AprilDetection::processImage(cv::Mat image){

  cv::Mat image_gray; 
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  image_u8_t im = { .width  = image_gray.cols,
                    .height = image_gray.rows,
                    .stride = image_gray.cols, 
                    .buf    = image_gray.data 
  };

  zarray_t * detections = apriltag_detector_detect(a_detector, &im);

  apriltag_detection_t *det;
  vector<apriltag_pose_t> poses;
  vector<int> ids;

  for (int i=0; i<zarray_size(detections); i++){

    zarray_get(detections, i, &det);
    info.det = det;
    apriltag_pose_t pose;

    // estimate SE(3) pose 
    estimate_tag_pose(&info, &pose);
    poses.push_back(pose);
    ids.push_back(det->id);

  }
  
  return make_tuple(poses, ids, image);

}

void AprilDetection::setInfo(double tagSize, double fx, double fy, double cx, double cy){
  info.tagsize = tagSize;
  info.fx = fx;
  info.fy = fy;
  info.cx = cx;
  info.cy = cy;
}
