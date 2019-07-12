#include"opencv2/opencv.hpp"
//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/ahmedshehata/Libraries/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "/home/ahmedshehata/Libraries/librealsense/examples/example.hpp"
#include "/home/ahmedshehata/Libraries/librealsense/third-party/imgui/imgui.h"
#include "/home/ahmedshehata/Libraries/librealsense/third-party/imgui/imgui_impl_glfw.h"
//#include "opencv2/tracking.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
//#include <opencv2/video/background_segm.hpp>
#include "/home/ahmedshehata/Libraries/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <signal.h>
// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include "ros/ros.h"
#include "track3d/ball_trajectory.h"
#include <unistd.h>
#include"tf2/LinearMath/Transform.h"
//#include"cv_bridge/cv_bridge.h"
//#include "image_transport/image_transport.h"
#include "track3d/img_stream.h"
//#include "opencv2/core/cuda.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudafilters.hpp"
//#include <opencv2/cudaimgproc.hpp>
//#include "opencv2/core/cuda.inl.hpp"
using namespace std;
using namespace std::chrono;
#include "opencv2/gpu/gpu.hpp"
using namespace cv;
//using namespace cv::cuda;

tf2::Transform cam_to_base;
using pixel = std::pair<int, int>;

int src_depth_fps = 30, src_rgb_fps = 30;
int width_rgb = 1280, height_rgb = 720;
int width_depth = 1280, height_depth = 720;
const char* selection_window = "selection";
const char* source_window = "Source image";
const char* corners_window = "Corners detected";

//---------------------------------------------------------------

//create msg holder
track3d::ball_trajectory ball_trajectory_msg;
const int points_num = 10;
double cam_points[4][points_num];
int current_point = 0, mouse_col= -1, mouse_row = -1, thresh = 200, max_thresh = 255;
pixel cp1;
Rect2d r;

void MouseCallbackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        mouse_col =  x;
        mouse_row = y;
        cout<<"dosnaaaaaaaaaaa"<<endl;
    }
}

int low_r =0 , low_G = 0, low_b = 112;
int high_r = 194, high_G = 55, high_b = 255;
int max_value = 255;
const String window_detection_name = "Thresholding";

//Trackbar defs--------------------------------------------
static void on_low_H_thresh_trackbar(int, void *)
{
    low_r = min(high_r-1, low_r);
    setTrackbarPos("Low R", window_detection_name, low_r);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    setTrackbarPos("High R", window_detection_name, high_r);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_G = min(high_G-1, low_G);
    setTrackbarPos("Low G", window_detection_name, low_G);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_G = max(high_G, low_G+1);
    setTrackbarPos("High G", window_detection_name, high_G);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_b = min(high_b-1, low_b);
    setTrackbarPos("Low B", window_detection_name, low_b);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    setTrackbarPos("High B", window_detection_name, high_b);
}
//---------------------------------------------------------------


//ball position and velocity per frame variables:
//------------------------------------------------
class SubscribeAndPublish
{
public:
  SubscribeAndPublish(int height, int width, double ppx, double ppy, double fx, double fy):
    x_dist_origin_(height,width,CV_64F),
    y_dist_origin_(height,width,CV_64F)
  {

    //Topic you want to subscribe
    img_stream_sub_ = nh_.subscribe("/img_stream", 10, &SubscribeAndPublish::img_stream_callback, this);

    for(int row=0;row<height;row++)
    {
      for(int col=0;col<width;col++)
      {
        x_dist_origin_.at<double>(row,col) = (double(col)- ppx)/fx;
        y_dist_origin_.at<double>(row,col) = (double(row)-ppy)/fy;
      }
    }

  }

  void img_stream_callback(const track3d::img_stream& msg)
  {
    rs2_intrinsics intr;
    intr.width = msg.width;
    intr.height = msg.height;
    intr.ppx = msg.ppx;
    intr.ppy = msg.ppy;
    intr.fx = msg.fx;
    intr.fy = msg.fy;
    intr.model = rs2_distortion(msg.model);
//    cout<<intr.height<<", "<<intr.width<<endl;
//    cout<<"ppx = "<<intr.ppx;
//    cout<<", fx = "<<intr.fx;
//    cout<<", ppy = "<<intr.ppy;
//    cout<<", fy = "<<intr.fy<<endl;
//    cout<<"here_1"<<endl;
    for(int i=0;i<5;i++){intr.coeffs[i] = msg.coeffs[i];}
    Mat depth_img(msg.height,msg.width,CV_16UC1,(void*)msg.depth.data());
//    cout<<"here_1_1"<<endl;
    if(msg.ir.data() == NULL)cout<<"Nothing"<<endl;
    Mat img(msg.height,msg.width,CV_8UC1,(void*)msg.ir.data());
//    cout<<"here_2"<<endl;
    Mat depth;
    depth_img.convertTo(depth, CV_64F);
    depth = depth * msg.depth_scale;
    Mat x_dist = x_dist_origin_.mul(depth), y_dist = y_dist_origin_.mul(depth);
//    cout<<"here_3"<<endl;
    if(waitKey(10) == 'a')
    {
      // Select ROI
      r = selectROI(img);
      // Crop image
      Mat imgCrop = img(r), frame_threshold;
      inRange(imgCrop, Scalar(low_r, low_G, low_b), Scalar(high_r, high_G, high_b), frame_threshold);
      threshold(frame_threshold,frame_threshold,0,1,CV_THRESH_BINARY);
      Moments m = moments(frame_threshold);
      cp1.first = m.m10/m.m00;
      cp1.second = m.m01/m.m00;
      circle(img, Point(cp1.first, cp1.second), 3, Scalar(0,0,0), -1);
      imshow(source_window, img);
      waitKey(0);
    }
//    imshow(source_window, img);
//    waitKey(1);
    cam_points[0][current_point] = x_dist.at<double>(cp1.second, cp1.first);
    cam_points[1][current_point] = y_dist.at<double>(cp1.second, cp1.first);
    cam_points[2][current_point] = depth.at<double>(cp1.second, cp1.first);
    cam_points[3][current_point] = 1.0;
    cout<<"["<<cam_points[0][current_point]<<","<<cam_points[1][current_point]<<","<<cam_points[2][current_point]<<"]"<<endl;
    current_point++;
    mouse_col = -1;
    mouse_row = -1;
    if(current_point == points_num)
    {
      cout<<"[";
      for(int row =0;row<4;row++)
      {
        cout<<"[";
        for(int col =0;col<points_num;col++)
        {
          cout<<cam_points[row][col];
          if(col != points_num-1)cout<<",";
        }
        cout<<"]"<<endl;
      }
      cout<<"]"<<endl;
      ros::shutdown();
    }
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber img_stream_sub_;
  rs2_intrinsics intr_;
  Mat x_dist_origin_;
  Mat y_dist_origin_;

  //    image_transport::ImageTransport it_(nh_);

};//End of class SubscribeAndPublish


int main(int argc, char *argv[])
{
    namedWindow(source_window);
    namedWindow(window_detection_name);
    createTrackbar("Low R", window_detection_name, &low_r, max_value, on_low_H_thresh_trackbar);
    createTrackbar("High R", window_detection_name, &high_r,max_value, on_high_H_thresh_trackbar);
    createTrackbar("Low G", window_detection_name, &low_G, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High G", window_detection_name, &high_G, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low B", window_detection_name, &low_b, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High B", window_detection_name, &high_b, max_value, on_high_V_thresh_trackbar);
    //ROS specifics:
    //----------------
    //Initializing ROS node with a name of demo_topic_publisher
    ros::init(argc, argv,"ball_detection", ros::init_options::NoSigintHandler);
    //Create an object of class SubscribeAndPublish that will take care of everything

    double ppx = 637.87, fx = 650.882, ppy = 361.211, fy = 650.882;
    SubscribeAndPublish SAPObject(height_depth,width_depth, ppx, ppy, fx, fy);
    //set the callback function for any mouse event
//    setMouseCallback(source_window, MouseCallbackFunc, NULL);
//    SubscribeAndPublish SAPObject(160,284, ppx, ppy, fx, fy);

    ros::spin();

    return EXIT_SUCCESS;
}


