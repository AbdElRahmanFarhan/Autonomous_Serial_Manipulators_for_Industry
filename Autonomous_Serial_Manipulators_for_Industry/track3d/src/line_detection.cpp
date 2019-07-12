#include"opencv2/opencv.hpp"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/ahmedshehata/Libraries/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "/home/ahmedshehata/Libraries/librealsense/examples/example.hpp"
#include "/home/ahmedshehata/Libraries/librealsense/third-party/imgui/imgui.h"
#include "/home/ahmedshehata/Libraries/librealsense/third-party/imgui/imgui_impl_glfw.h"
#include "opencv2/tracking.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "/home/ahmedshehata/Libraries/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
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

using namespace std;
using namespace cv;
using namespace std::chrono;

using pixel = std::pair<int, int>;
int global_write_flag = 0;
pixel cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9,cp10,cp11,cp12,cp13,cp14,cp_dogga;
int width = 1280, height = 720;
const char* image_window = "Source Image";

vector<Vec3f> circles;
vector<Vec3f>control_points_dist;
high_resolution_clock::time_point t_frame1;
high_resolution_clock::time_point t_frame2;
Point3f dist_3d(const rs2::depth_frame& frame, pixel u);
void write_to_file(std::string, std::vector<Vec3f>, std::vector<double>);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
vector<Point2f> detect_chess_corners(Mat img);


//---------------------------------------------------------------
Mat src;
const char* source_window = "Source image";
int thresh_1 = 80, thresh_2 = 174, sobel = 3, votes = 240, max_len = 80, max_gap = 50;
int max_value = 255;
Vec4i x_axis, y_axis;
int x_flag = 0, y_flag = 0;
const String canny_trackbar_window = "canny_Trackbar";
const String lines_trackbar_window = "lines_Trackbar";
Rect2d r;
//Trackbar defs--------------------------------------------
static void thresh_1_trackbar(int, void *)
{
  thresh_1 = min(thresh_2-1, thresh_1);
  setTrackbarPos("thresh_1", canny_trackbar_window, thresh_1);
}
static void thresh_2_trackbar(int, void *)
{
  thresh_2 = max(thresh_2, thresh_1+1);
  setTrackbarPos("thresh_2", canny_trackbar_window, thresh_2);
}
static void sobel_trackbar(int, void *)
{
  if(sobel % 2 == 0)sobel += 1;
  setTrackbarPos("sobel", canny_trackbar_window, sobel);
}
static void votes_trackbar(int, void *)
{
  setTrackbarPos("votes", lines_trackbar_window, votes);
}
static void max_len_trackbar(int, void *)
{
  setTrackbarPos("max_len", lines_trackbar_window, max_len);
}
static void max_gap_trackbar(int, void *)
{
  setTrackbarPos("max_gap", lines_trackbar_window, max_gap);
}
//--------------- corner detection demo variables
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()
int main(int argc, char *argv[])try
{
    //ROS specifics:
    //----------------
    //Initializing ROS node with a name of demo_topic_publisher
    ros::init(argc, argv,"ball_detection");
    //Created a nodehandle object
    ros::NodeHandle node_obj;

    namedWindow(canny_trackbar_window,WINDOW_AUTOSIZE);
    createTrackbar("thresh_1", canny_trackbar_window, &thresh_1, max_value, thresh_1_trackbar);
    createTrackbar("thresh_2", canny_trackbar_window, &thresh_2, max_value, thresh_2_trackbar);
    createTrackbar("sobel", canny_trackbar_window, &sobel, max_value, sobel_trackbar);

    namedWindow(lines_trackbar_window,WINDOW_AUTOSIZE);
    createTrackbar("votes", lines_trackbar_window, &votes, max_value*10, votes_trackbar);
    createTrackbar("max_len", lines_trackbar_window, &max_len, max_value*10, max_len_trackbar);
    createTrackbar("max_gap", lines_trackbar_window, &max_gap, max_value*10, max_gap_trackbar);

    //ball position and velocity per frame variables:
    //------------------------------------------------
    const int points_num = 13;
    int frame_count = 0, crop_flag = 1, initialize_exposure_flag = 1;
    //vector<Point> detection_trail(20);

    //camera variables:
    //-------------------
    std::map<int, int> counters;

    namedWindow( image_window, WINDOW_AUTOSIZE );
    rs2::decimation_filter dec;
    rs2::spatial_filter spat;
    rs2::hole_filling_filter holes;
    rs2::temporal_filter temp;
    rs2::threshold_filter thresh_filter;
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    //processing options
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.2);
    temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,100);
    temp.set_option(RS2_OPTION_HOLES_FILL,1);
//    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    holes.set_option(RS2_OPTION_HOLES_FILL, 0);
//    thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 3.0); // Lab
    thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 2);

    std::mutex mutex;
    const unsigned int CAPACITY = 1; // allow max latency of 10 frames
    const unsigned int stream_CAPACITY = 1; // allow max latqency of 10 frames
    rs2::frame_queue stream_frames(stream_CAPACITY);
    // After initial post-processing, frames will flow into this queue:
    rs2::frame_queue postprocessed_frames(CAPACITY);
    //    rs2::frame_queue postprocessed_frames(std::numeric_limits<unsigned int>::max());
    //    rs2::frame_queue stream_frames(std::numeric_limits<unsigned int>::max());
    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (rs2::frameset fs = frame.as<rs2::frameset>())
      {
        //            // With callbacks, all synchronized stream will arrive in a single frameset
        //            for (const rs2::frame& f : fs)
        //                counters[f.get_profile().unique_id()]++;
  //      fs.keep();
        stream_frames.enqueue(fs);
      }
      else
      {
        // Stream that bypass synchronization (such as IMU) will produce single frames
        counters[frame.get_profile().unique_id()]++;
      }
    };

    //stream profile and starting
    rs2::pipeline pipe;
    rs2::config cfg;

    //cfg.enable_device_from_file();
    cfg.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,30);// Enable default depth
    cfg.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_RGBA8,30);
//    cfg.enable_stream(RS2_STREAM_INFRARED,1,1280,720,RS2_FORMAT_Y8,30);

    auto profile = pipe.start(cfg,callback);

//    rs2_extrinsics extrinsics = profile.get_stream(RS2_STREAM_COLOR).get_extrinsics_to(profile.get_stream(RS2_STREAM_DEPTH));
//                std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
//                std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
//                std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
//                std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;

    //aligning of the color and depth streams
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    rs2::align align(align_to);
    // Alive boolean will signal the worker threads to finish-up
    std::atomic_bool alive{ true };
    //frame capturing and alignment thread:
    //-----------------------------------------
    std::thread video_processing_thread([&]() {
        // In order to generate new composite frames, we have to wrap the processing
        // code in a lambda
        rs2::processing_block frame_processor(
                    [&](rs2::frameset data, // Input frameset (from the pipeline)
                    rs2::frame_source& source) // Frame pool that can allocate new frames
        {
//            data = data.apply_filter(dec);
            data = data.apply_filter(align); //Here we align
//            data = data.apply_filter(holes);
            // To make sure far-away objects are filtered proportionally
            // we try to switch to disparity domain
            data = data.apply_filter(depth2disparity);

//            // Apply spatial filtering
//            data = data.apply_filter(spat);

            // Apply temporal filtering
            data = data.apply_filter(temp);

            // If we are in disparity domain, switch back to depth
            data = data.apply_filter(disparity2depth);
//            data = data.apply_filter(thresh_filter);

            source.frame_ready(data);
        });

        // Indicate that we want the results of frame_processor
        // to be pushed into postprocessed_frames queue
        frame_processor >> postprocessed_frames;

        while (alive)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          std::lock_guard<std::mutex> lock(mutex);
          // Fetch frames from the pipeline and send them for processing
          rs2::frameset fs;
          //            if (pipe.poll_for_frames(&fs)) frame_processor.invoke(fs);
          if(stream_frames.poll_for_frame(&fs))
          {
            //               cout<<"thread__in"<<endl;
            frame_processor.invoke(fs);
          }
        }
    });

    //Our main while loop:
    //------------------------
    while ((waitKey(10) != int('q') && cvGetWindowHandle(image_window))&& ros::ok()) // Application still alive?
    {
        static rs2::frameset current_frameset;
        if(postprocessed_frames.poll_for_frame(&current_frameset))
        {
            frame_count += 1;
            if (current_frameset) //If frame is not yet constructed (ref_assign_flag==0), construct
            {
                auto depth_1 = current_frameset.get_depth_frame(); //Aligned depth frame
                auto color_1 = current_frameset.get_color_frame();
                Mat img = frame_to_mat(color_1).clone(); //source img ready
//                imshow(image_window, img);
//                waitKey(1);
                if(frame_count <= 100 && initialize_exposure_flag==1)
                {
                  if(frame_count >= 90)
                  {
                    initialize_exposure_flag = 0;
                    frame_count = 0;
                  }
                  continue;
                }
                if(crop_flag && x_flag == 0)
                {
                  // Select ROI
                  r = selectROI(img);
                  if(y_flag == 1)crop_flag=0;
                }
                Mat img_crop = img(r), gray_crop;
                if(x_flag == 1)
                {
                  circle(img,Point(x_axis[0],x_axis[1]),5,Scalar(0,0,0),2);
                  circle(img,Point(x_axis[2],x_axis[3]),5,Scalar(0,0,0),2);
                  imshow(image_window, img);
                  if(waitKey(10) == 'q')break;
                  Point3f p1 = dist_3d(depth_1, pixel(x_axis[0],x_axis[1]));
                  Point3f p2 = dist_3d(depth_1, pixel(x_axis[2],x_axis[3]));
                  cout<<"difference = ["<<p2.x-p1.x<<", "<<p2.y-p1.y<<", "<<p2.z-p1.z<<"]"<<endl;
                  cout<<"point_1 = [["<<p1.x<<"], ["<<p1.y<<"], ["<<p1.z<<"]]"<<endl;
                  cout<<"point_2 = [["<<p2.x<<"], ["<<p2.y<<"], ["<<p2.z<<"]]"<<endl;
                  continue;
                }
                while(x_flag == 0)
                {
                  Mat play_img = img.clone();
                  cvtColor(img_crop, gray_crop, CV_BGR2GRAY);
                  // Edge detection
                  Canny(gray_crop, gray_crop, thresh_1, thresh_2, sobel);
                  // Probabilistic Line Transform
                  vector<Vec4i> linesP; // will hold the results of the detection
                  HoughLinesP(gray_crop, linesP, 1, (CV_PI/180.0)*0.1, votes, max_len, max_gap ); // runs the actual detection
                  // Draw the lines
                  for( size_t i = 0; i < linesP.size(); i++ )
                  {
                    cout<<linesP.size()<<endl;
                    Vec4i l = linesP[i];
                    line( play_img, Point(l[0]+r.x, l[1]+r.y), Point(l[2]+r.x, l[3]+r.y), Scalar(0,0,255), 1, LINE_AA);
                  }
                  cout<<linesP.size()<<endl;
                  imshow( image_window, play_img );
                  imshow("cropped", gray_crop);
                  cout<<linesP.size()<<endl;
                  if(waitKey(10) == 'a' && linesP.size() > 0)
                  {
                    cout<<"Innnn"<<endl;
                    if(x_flag == 0)
                    {
                      x_axis[0] = linesP[0][0]+r.x;
                      x_axis[1] = linesP[0][1]+r.y;
                      x_axis[2] = linesP[0][2]+r.x;
                      x_axis[3] = linesP[0][3]+r.y;
                      x_flag = 1;
                      break;
                    }
                    else if(y_flag == 0)
                    {
                      y_axis[0] = linesP[0][0]+r.x;
                      y_axis[1] = linesP[0][1]+r.y;
                      y_axis[2] = linesP[0][2]+r.x;
                      y_axis[3] = linesP[0][3]+r.y;
                      y_flag = 1;
                      break;
                    }
                  }
                }
                if(y_flag == 0)continue;
                cout<<"Here"<<endl;
                //Find the intersection point
                int x_origin=0, y_origin=0, x=0, y=0;
                double m_x=0, m_y=0;
                m_x = double(x_axis[3]-x_axis[1])/double(x_axis[2]-x_axis[0]);
                m_y = double(y_axis[3]-y_axis[1])/max(double(y_axis[2]-y_axis[0]), 0.000001);
                cout<<"m_x = "<<m_x<<"m_y = "<<m_y<<endl;
                x_origin = round(double(x_axis[1]-y_axis[1]+m_y*y_axis[0]-m_x*x_axis[0])/double(m_y-m_x));
                y_origin = round(m_x*double(x_origin-x_axis[0]) + x_axis[1]);

                circle(img,Point(x_origin,y_origin),5,Scalar(255,255,255),2);
                circle(img,Point(x_axis[0],x_axis[1]),5,Scalar(0,0,0),2);
                circle(img,Point(y_axis[0],y_axis[1]),5,Scalar(255,0,0),2);
                line(img,Point(x_axis[0],x_axis[1]),Point(x_origin,y_origin),Scalar(0,0,255),2);
                line(img,Point(x_origin,y_origin),Point(y_axis[0],y_axis[1]),Scalar(0,0,255),2);
                Point3f origin = dist_3d(depth_1, pixel(x_origin,y_origin));
                Point3f right = dist_3d(depth_1, pixel(x_axis[2],x_axis[3]));
//                Point3f left = dist_3d(depth_1, pixel(x_axis[0],x_axis[1]));
//                while(true)
//                {
//                  if(left.z >= 4 || left.z <= 0.1)
//                  {
//                    y = round(m_x*double(x-x_axis[0]) + x_axis[1]);
//                    left = dist_3d(depth_1, pixel(x,y));
//                  }
//                  else break;
//                }
                Point3f upper = dist_3d(depth_1, pixel(y_axis[0],y_axis[1]));
                cout<<"Origin = [["<<origin.x<<"], ["<<origin.y<<"], ["<<origin.z<<"]]"<<endl;
                cout<<"right = [["<<right.x<<"], ["<<right.y<<"], ["<<right.z<<"]]"<<endl;
//                cout<<"left = [["<<left.x<<"], ["<<left.y<<"], ["<<left.z<<"]]"<<endl;
                cout<<"upper = [["<<upper.x<<"], ["<<upper.y<<"], ["<<upper.z<<"]]"<<endl;
                imshow(image_window, img);
                waitKey(0);
                break;
//                if(key == 'r')
//                {
//                  frame_count -= 1;
//                  cout<<"NOTICE:: RETAKE POINT"<<endl;
//                  continue;
//                }
            }
        }
    }
    // Signal threads to finish and wait until they do
    cvDestroyAllWindows();
    pipe.stop();
    alive = false;
    video_processing_thread.join();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}
Point3f dist_3d(const rs2::depth_frame& frame, pixel u)
{
    float upixel[2]; // From pixel
    float upoint[3]; // From point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;

    // Query the frame for distance
    // Note: this can be optimized
    // It is not recommended to issue an API call for each pixel
    // (since the compiler can't inline these)
    // However, in this example it is not one of the bottlenecks
    auto udist = frame.get_distance(upixel[0], upixel[1]);
    if (udist==0)
       udist = frame.get_distance(upixel[0], upixel[1]);

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

    Point3f point(upoint[0], upoint[1], upoint[2]);

    return point;
}


void write_to_file(std::string file_name , std::vector<Vec3f> data, std::vector<double> time)
{
    std::ofstream myfile;
    int status;
    if(global_write_flag == 0)
    {
        status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_x.txt");
        status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_y.txt");
        status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_z.txt");
        status = std::remove("/home/ahmedshehata/Loggings/ball_trajectory_ref_t.txt");
        global_write_flag = 1;
    }

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
vector<Point2f> detect_chess_corners(Mat img)
{
    // Crop image
    Mat imgCrop = img(r), src_gray;
    cvtColor( imgCrop, src_gray, COLOR_BGR2GRAY );
    Size patternsize(3,5); //interior number of corners
    vector<Point2f> corners; //this will be filled by the detected corners

//    float points_from_cam = {{-1.36998176, -1.37241788, -1.37731596, -1.40387318, -1.40129106, -1.4104493
//      -1.43659909, -1.43656647 ,-1.44021636, -1.47841649, -1.47024385, -1.47346846,
//      -1.50895434 ,-1.50557494 ,-1.50951598},
//     {-0.12935937 ,-0.08837283 ,-0.04721676, -0.1283801 , -0.08737281, -0.04690939,
//      -0.13024972 ,-0.08971852 ,-0.0464118  ,-0.1329619 , -0.08908602 ,-0.04893399,
//      -0.13179418 ,-0.09144758 ,-0.04856988},
//     { 2.7291201  , 2.73410291  ,2.73806639,  2.70885389,  2.70387572,  2.72180622,
//       2.68759666 , 2.68759864 , 2.69457209 , 2.68426868 , 2.66933415 , 2.67531077,
//       2.66102866  ,2.65505669,  2.65702769}};


    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    bool patternfound = findChessboardCorners(src_gray, patternsize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
            + CALIB_CB_FAST_CHECK);
    if(patternfound)
    {
      cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cout<<"Patterns_FOUND"<<endl;
    }
    else
    {
      cout<<"Patterns_NOT_FOUND"<<endl;
    }
    for(int i=0; i<corners.size(); i++)
    {
        corners[i] = Point2f(corners[i].x+r.x, corners[i].y+r.y);
    }
    drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
    return corners;
}

