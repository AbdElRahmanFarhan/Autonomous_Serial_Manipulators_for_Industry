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
using namespace cv::cuda;

tf2::Transform cam_to_base;
using pixel = std::pair<int, int>;

pixel p1;
const int search_step = 1, search_area_length = 50;
pixel search_area[search_area_length][search_area_length];
float x_search_dist[search_area_length*search_area_length];
float y_search_dist[search_area_length*search_area_length];
float z_search_dist[search_area_length*search_area_length];

double x_shift=0.195,y_shift=-0.009,z_shift=0.067;
int src_depth_fps = 90, src_rgb_fps = 60;
int width_rgb = 848, height_rgb = 480;
int width_depth = 848, height_depth = 480;
const char* image_window = "Source Image";
const char* depth_window = "depth_window";
double cb_t1=0,cb_t2=0;
high_resolution_clock::time_point t_frame1;
high_resolution_clock::time_point search_t_1;
high_resolution_clock::time_point search_t_2;
high_resolution_clock::time_point t_frame2;
Vec3f dist_3d(Mat frame, pixel u, rs2_intrinsics);
void write_to_file(std::string, std::vector<Vec3f>, std::vector<double>);
void write_to_file_pix(std::string, std::vector<Vec2i>, std::vector<double>);
Moments motion_detect(Mat,float);

const char* source_window = "Source image";


int lower_x=115,upper_x=400,lower_y=120,upper_y=290,max_x_dist=400,max_y_dist=400;

//Trackbar defs--------------------------------------------
static void on_low_x_thresh_trackbar(int, void *)
{
    lower_x = min(upper_x-1, lower_x);
    setTrackbarPos("Low X", source_window, lower_x);
}
static void on_high_x_thresh_trackbar(int, void *)
{
    upper_x = max(upper_x, lower_x+1);
    setTrackbarPos("High X", source_window, upper_x);
}
static void on_low_y_thresh_trackbar(int, void *)
{
    lower_y = min(upper_y-1, lower_y);
    setTrackbarPos("Low Y", source_window, lower_y);
}
static void on_high_y_thresh_trackbar(int, void *)
{
    upper_y = max(upper_y, lower_y+1);
    setTrackbarPos("High Y", source_window, upper_y);
}
//---------------------------------------------------------------

//create msg holder
track3d::ball_trajectory ball_trajectory_msg;

int calibrate_flag = 0, publish_flag = 0, write_flag = 0; // Note:: 1:if u want to publish 0:if u don't

//refrence frame variables:
//-------------------------
int ref_assign_flag  = 1; // Note:: 0: if u want to detect reference frame 1: if u don't
int points_selected  = 1; // Note:: 1: if u want to detect reference frame 0: if u don't

//ball position and velocity per frame variables:
//------------------------------------------------
int frame_count = 0, vel_flag = 1, point_num = 0, publish_count = 0, start_flag = 0;
int detect_flag=0;
vector<Vec3f> ball_trajectory_data_ref;
vector<Vec2i> ball_trajectory_pixels;
vector<double> time_stamp;
vector<Mat> images_to_write, depth_to_write;
double t_1 = 0, t_2 = 0, delta_t, t_f = 0, depth_t_1=0, depth_t_2=0;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish(int height, int width, double ppx, double ppy, double fx, double fy):
    x_dist_origin_(height,width,CV_64F),
    y_dist_origin_(height,width,CV_64F)
  {
    //Topic you want to publish
    bal_traj_pub_ = nh_.advertise<track3d::ball_trajectory>("/ball_trajectory_topic", 1);

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
    cb_t1 = msg.time_stamp;
    cout<<"fpsssss = "<<1000.0/fabs(cb_t1-cb_t2)<<endl;
    cb_t2 = cb_t1;
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

    for(int i=0;i<5;i++){intr.coeffs[i] = msg.coeffs[i];}
    Mat depth_img(msg.height,msg.width,CV_16UC1,(void*)msg.depth.data());
//    Mat img(height_depth,width_depth,CV_8UC1,(void*)msg.ir.data());

    Mat depth, depth_written, binarized_x, binarized_y;
    depth_img.convertTo(depth, CV_64F);
    depth = depth * msg.depth_scale;
    depth_img.convertTo(depth_written,CV_8UC1);


//    depth_img.convertTo(depth_written, CV_8UC1);
//    imshow("source_window", img);
//    imshow("depth_window", detection_frame);
    waitKey(1);
    if (!start_flag)
    {
  //                    imshow(image_window,ir_img);
        if(waitKey(8)=='a')
        {
            start_flag=1;
        }
        return;
    }
    search_t_1= high_resolution_clock::now();
    Mat x_dist = x_dist_origin_.mul(depth), y_dist = y_dist_origin_.mul(depth);
    search_t_2= high_resolution_clock::now();
    auto search_duration = duration_cast<milliseconds>(search_t_2-search_t_1).count();
    cout<<"x_y_calc duration = "<<search_duration<<endl;
//    cout<<"ppx = "<<intr.ppx<<" ppy = "<<intr.ppy<<", fx = "<<intr.fx<<", fy = "<<intr.fy<<endl;

    t_frame1= high_resolution_clock::now();
    if(calibrate_flag)inRange(x_dist, (lower_x*0.01)-2,(upper_x*0.01)-2,binarized_x);
    else inRange(x_dist, (lower_x*0.01)-2,(upper_x*0.01)-2,binarized_x);
    cv::threshold( binarized_x, binarized_x, 0, 1, THRESH_BINARY);
    if(calibrate_flag)inRange(y_dist, (lower_y*0.01)-2,(upper_y*0.01)-2,binarized_y);
    else inRange(y_dist, (lower_y*0.01)-2,(upper_y*0.01)-2,binarized_y);
    cv::threshold( binarized_y, binarized_y, 0, 1, THRESH_BINARY);
    depth_written = depth_written.mul(binarized_x.mul(binarized_y));
//    img = img.mul(binarized_x.mul(binarized_y));

    imshow(source_window, depth_written);
//    imshow(depth_window, img);

    t_frame2= high_resolution_clock::now();
    auto duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
    cout<<"thresholding time="<<duration2<<endl;
    t_frame1= high_resolution_clock::now();
    cout<<"start_calc_time================"<<endl;
    search_t_1= high_resolution_clock::now();
    Moments m = motion_detect(depth_written,-1);
    t_frame2= high_resolution_clock::now();
    duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
    cout<<"back_sub time="<<duration2<<endl;
    t_frame1= high_resolution_clock::now();
    float dist_ball_from_cam[3]={0};

    if(m.m00>0/* && m.m00<=1000*/)
    {
        cout<<"m00 = "<<m.m00<<endl;
        p1.first=m.m10/m.m00;
        p1.second=m.m01/m.m00;
        int search_counter=0;

        //Search Area//-------------------------------
        for(int row=0;row<search_area_length;row++)
        {
            for(int col=0;col<search_area_length;col++)
            {
                search_area[row][col].first=p1.first-(search_step*int(search_area_length/2))+col*search_step;
                search_area[row][col].second=p1.second-(search_step*int(search_area_length/2))+row*search_step;
                if(search_area[row][col].first<=width_rgb && search_area[row][col].second<=height_rgb)
                {
                    Point s1( search_area[row][col].first, search_area[row][col].second);
                    if(row==0 || row==search_area_length-1 || col==0 || col==search_area_length-1)
                    {
//                        circle(img, s1, search_step, Scalar(0,0,128), -1);
                        circle(depth_written, s1, search_step, Scalar(255,255,255), -1);
                    }
                    Vec3f distanceVec = dist_3d(depth,search_area[row][col],intr);
                    if(distanceVec.val[2] <=0.1)distanceVec.val[2] = 100.0;
                    x_search_dist[search_counter]=distanceVec.val[0];
                    y_search_dist[search_counter]=distanceVec.val[1];
                    z_search_dist[search_counter]=distanceVec.val[2];
                    search_counter++;
                }
            }
        }
        Mat x_mat(search_area_length,search_area_length,CV_32FC1,x_search_dist);
        Mat y_mat(search_area_length,search_area_length,CV_32FC1,y_search_dist);
        Mat z_mat(search_area_length,search_area_length,CV_32FC1,z_search_dist);

        double min_val, max_val;
        Point min_idx;
        cv::minMaxLoc(z_mat, &min_val,&max_val,&min_idx);

        dist_ball_from_cam[0] = x_mat.at<float_t>(min_idx.y,min_idx.x);
        dist_ball_from_cam[1] = y_mat.at<float_t>(min_idx.y,min_idx.x);
        dist_ball_from_cam[2] = z_mat.at<float_t>(min_idx.y,min_idx.x);

        search_t_2= high_resolution_clock::now();
        search_duration = duration_cast<milliseconds>(search_t_2-search_t_1).count();
        t_frame2= high_resolution_clock::now();
        auto duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
        cout<<"search time="<<duration2<<endl;
        cout<<"if:total duration = "<<search_duration<<endl;
        //--------------------------------------------
//        Point p( p1.first, p1.second);
//        circle(depth_written, p, 5, Scalar(0,0,0), -1);
//        circle(img, p, 5, Scalar(0,0,0), -1);

//        imshow(depth_window, img);

        if(write_flag==1)
        {
            depth_to_write.push_back(depth_written);
            if(!publish_flag)
            {
              ball_trajectory_pixels.push_back({min_idx.x,min_idx.y});

              ball_trajectory_data_ref.push_back({ dist_ball_from_cam[0],
                                                   dist_ball_from_cam[1],
                                                   dist_ball_from_cam[2]});
            }
//            images_to_write.push_back(img.clone());
        }
    }
    else
    {
        cout<<"no ball detected"<<endl;
        search_t_2= high_resolution_clock::now();
        auto search_duration = duration_cast<milliseconds>(search_t_2-search_t_1).count();
        cout<<"else:total duration = "<<search_duration<<endl;
        return;

    }
    //=========================================================//


    tf2::Vector3 dist_ball_from_base = cam_to_base.operator ()(tf2::Vector3(dist_ball_from_cam[0],dist_ball_from_cam[1], dist_ball_from_cam[2]));
    cout<<"BALL DETECTED| Showing pixel and point coordinates"<<p1.first<<" "<<p1.second<<endl;
    cout<<"Ball Distance_cam: "<< dist_ball_from_cam[0]<<" "<< dist_ball_from_cam[1]<<" "<< dist_ball_from_cam[2]<<endl;
    cout<<"Ball Distance_base: "<<dist_ball_from_base.getX()<<" "<<dist_ball_from_base.getY()<<" "<<dist_ball_from_base.getZ()<<endl;

    if(publish_flag == 1)
    {
      ROS_INFO("Ball Found Time %d",point_num);
      ball_trajectory_pixels.push_back({p1.first,p1.second});

      ball_trajectory_data_ref.push_back({ dist_ball_from_cam[0],
                                           dist_ball_from_cam[1],
                                           dist_ball_from_cam[2]});

      time_stamp.push_back(cb_t1);

      ball_trajectory_msg.x.push_back(dist_ball_from_cam[0]);
      ball_trajectory_msg.y.push_back(dist_ball_from_cam[1]);
      ball_trajectory_msg.z.push_back(dist_ball_from_cam[2]);
      ball_trajectory_msg.t.push_back(cb_t1);

      point_num++;
      if(point_num == 12)
      {
        ROS_INFO("Trajectory start sending Time");
        bal_traj_pub_.publish(ball_trajectory_msg);
        write_to_file("/home/ahmedshehata/Loggings/ball_trajectory_ref" , ball_trajectory_data_ref, time_stamp);
        write_to_file_pix("/home/ahmedshehata/Loggings/ball_trajectory_pixels" , ball_trajectory_pixels, time_stamp);
//        for(int i=0;i<images_to_write.size();i++)
//        {
//            imwrite("/home/ahmedshehata/Downloads/Ball_traj_recNew/"+to_string(i)+".jpg",images_to_write[i]);
//        }
//        for(int i=0;i<depth_to_write.size();i++)
//        {
//            imwrite("/home/ahmedshehata/Downloads/Ball_traj_recNew/depth/"+to_string(i)+".jpg",depth_to_write[i]);
//        }
//        ros::Duration(1).sleep();
        point_num = 0;
        ball_trajectory_msg.x.clear();
        ball_trajectory_msg.y.clear();
        ball_trajectory_msg.z.clear();
        ball_trajectory_msg.t.clear();
        ball_trajectory_data_ref.clear();
        ball_trajectory_pixels.clear();
        time_stamp.clear();
        depth_to_write.clear();
        waitKey(0);
      }
    }

  }


private:
  ros::NodeHandle nh_;
  ros::Publisher bal_traj_pub_;
  ros::Subscriber img_stream_sub_;
  rs2_intrinsics intr_;
  Mat x_dist_origin_;
  Mat y_dist_origin_;
  //    image_transport::ImageTransport it_(nh_);

};//End of class SubscribeAndPublish

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  if(write_flag)
  {
//    write_to_file("/home/ahmedshehata/Loggings/ball_trajectory_ref" , ball_trajectory_data_ref, time_stamp);
//    write_to_file_pix("/home/ahmedshehata/Loggings/ball_trajectory_pixels" , ball_trajectory_pixels, time_stamp);
//    for(int i=0;i<images_to_write.size();i++)
//    {
//      imwrite("/home/ahmedshehata/Downloads/Ball_traj_recNew/"+to_string(i)+".jpg",images_to_write[i]);
//    }
//    for(int i=0;i<depth_to_write.size();i++)
//    {
//      imwrite("/home/ahmedshehata/Downloads/Ball_traj_recNew/depth/"+to_string(i)+".jpg",depth_to_write[i]);
//    }
  }
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char *argv[])
{
    namedWindow(source_window);
    createTrackbar("Low X", source_window, &lower_x, max_x_dist, on_low_x_thresh_trackbar);
    createTrackbar("High X", source_window, &upper_x, max_x_dist, on_high_x_thresh_trackbar);
    createTrackbar("Low Y", source_window, &lower_y, max_y_dist, on_low_y_thresh_trackbar);
    createTrackbar("High Y", source_window, &upper_y, max_y_dist, on_high_y_thresh_trackbar);
    //ROS specifics:
    //----------------
    //Initializing ROS node with a name of demo_topic_publisher
    ros::init(argc, argv,"ball_detection", ros::init_options::NoSigintHandler);
    //Create an object of class SubscribeAndPublish that will take care of everything
    if(calibrate_flag)
    {
      publish_flag = 0;
      write_flag = 0;
    }
    else
    {
      publish_flag = 1;
      write_flag = 1;
    }
    double ppx = 211.015, fx = 215.523, ppy = 120.4, fy = 215.523;
//    SubscribeAndPublish SAPObject(480,848, ppx, ppy, fx, fy);
    SubscribeAndPublish SAPObject(240, 424, ppx, ppy, fx, fy);

    signal(SIGINT, mySigintHandler);

    ros::spin();

    return EXIT_SUCCESS;
}
Vec3f dist_3d(Mat frame, pixel u, rs2_intrinsics intr)
{
    float upixel[2]; // From pixel
    float upoint[3]; // From point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;

    auto udist = frame.at<double>(upixel[1], upixel[0]);

    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
    Vec3f upoint_vec;
    upoint_vec.val[0] = upoint[0];
    upoint_vec.val[1] = upoint[1];
    upoint_vec.val[2] = upoint[2];

    return upoint_vec;
}


void write_to_file(std::string file_name , std::vector<Vec3f> data, std::vector<double> time)
{
    std::ofstream myfile;
    int status;

    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_x.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_y.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_z.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_t.txt");

    myfile.open(file_name+"_x.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<data.size() ;i++)
        {
            myfile<<std::to_string(data[i].val[0])<<std::endl;
        }
        myfile.close();
    }
    myfile.open(file_name+"_y.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<data.size();i++)
        {
            myfile<<std::to_string(data[i].val[1])<<std::endl;
        }
        myfile.close();
    }
    myfile.open(file_name+"_z.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<data.size() ;i++)
        {
            myfile<<std::to_string(data[i].val[2])<<std::endl;
        }
        myfile.close();
    }

    myfile.open(file_name+"_t.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<time.size() ;i++)
        {
            myfile<<std::to_string(time[i])<<std::endl;
        }
        myfile.close();
    }
}
void write_to_file_pix(std::string file_name , std::vector<Vec2i> data, std::vector<double> time)
{
    std::ofstream myfile;
    int status;

    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_pixels_x.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_pixels_y.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_pixels_z.txt");
    status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_pixels_t.txt");

    myfile.open(file_name+"_x.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<data.size() ;i++)
        {
            myfile<<std::to_string(data[i].val[0])<<std::endl;
        }
        myfile.close();
    }
    myfile.open(file_name+"_y.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<data.size();i++)
        {
            myfile<<std::to_string(data[i].val[1])<<std::endl;
        }
        myfile.close();
    }

    myfile.open(file_name+"_t.txt",std::ios::binary|std::ios::out|std::ios::app);
    if(myfile.is_open())
    {
        for(int i = 0;i<time.size() ;i++)
        {
            myfile<<std::to_string(time[i])<<std::endl;
        }
        myfile.close();
    }
}
Moments motion_detect(Mat src,float lr)
{
//  t_frame1= high_resolution_clock::now();
  Moments m = moments(src,true);
//  t_frame2= high_resolution_clock::now();
//  auto duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
//  cout<<"moments time = "<<duration2<<endl;
  return m;
}
