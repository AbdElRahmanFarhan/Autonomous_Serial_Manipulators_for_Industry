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
float x_1st,y_1st,y_2nd,x_2nd,x_3rd,y_3rd,x_4th,y_4th,x_5th,y_5th,x_6th,y_6th;
const char* arr = "dst norm scaled";
Rect2d r;
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

    //ball position and velocity per frame variables:
    //------------------------------------------------
    const int points_num = 13;
    int frame_count = 0, crop_flag = 1, sz = 0, initialize_exposure_flag = 1;
    float actual_x[15], actual_y[15], actual_z[15];
    float cam_x[15], cam_y[15], cam_z[15], transform_x[points_num], transform_y[points_num], transform_z[points_num];
    Point3f rmse_accum(0,0,0), accumulator(0,0,0);
    //vector<Point> detection_trail(20);
    vector<Vec3f> ball_trajectory_data_ref;
    vector<double> time_stamp;
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

    //geting sensor info
//    auto sensor = profile.get_device().first<rs2::depth_sensor>();
//    if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
//    {
//        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter

//    }
//    if (sensor.supports(RS2_OPTION_LASER_POWER))
//    {

//        sensor.set_option(RS2_OPTION_LASER_POWER, 200);

//    }
//    auto range = sensor.get_option_range(RS2_OPTION_VISUAL_PRESET);
//    for (auto i = range.min; i < range.max; i += range.step)
//        if (std::string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "Default")
//            sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);

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
            t_frame1= high_resolution_clock::now();
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
            t_frame2 = high_resolution_clock::now();
        auto duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
//        cout<<"Alignment time="<<duration2<<endl;

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
    while ((waitKey(10) != int('q') && cvGetWindowHandle(image_window))&& frame_count<=1000 && ros::ok()) // Application still alive?
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
                if(crop_flag)
                {
                  // Select ROI
                  r = selectROI(img);
                  if(frame_count > points_num)crop_flag=0;
                }
                vector<Point2f> corners = detect_chess_corners(img);
                vector<Point3f> corners_from_cam, corners_actual, squared_error;
                sz = corners.size();
                cout<<"Here_5"<<"frame_no = "<<frame_count<<endl;
                circle(img, corners[sz-1], 5, Scalar(0,0,0),2);
                circle(img, corners[sz-3], 5, Scalar(0,0,0),2);
                circle(img, corners[sz-7], 5, Scalar(0,0,0),2);
                imshow(image_window, img);
                int key = waitKey(0);
                if(key == 'r')
                {
                  frame_count -= 1;
                  cout<<"NOTICE:: RETAKE POINT"<<endl;
                  continue;
                }
                for(int i=0; i<sz; i++)
                {
                  corners_from_cam.push_back(dist_3d(depth_1, pixel(corners[i].x,corners[i].y)));
                  if(frame_count>0)
                  {
                    cam_x[i] = corners_from_cam[i].x;
                    cam_y[i] = corners_from_cam[i].y;
                    cam_z[i] = corners_from_cam[i].z;
                    if(i == sz-1)
                    {
                      cout<<"origin = ["<<cam_x[i]<<"], ["<<cam_y[i]<<"], ["<<cam_z[i]<<"]"<<endl;
                      transform_x[frame_count-1] = cam_x[i];
                      transform_y[frame_count-1] = cam_y[i];
                      transform_z[frame_count-1] = cam_z[i];
                      if(frame_count == points_num)
                      {
                        cout<<"[";
                        for(int row =0;row<3;row++)
                        {
                          cout<<"[";
                          for(int col =0;col<points_num;col++)
                          {
                            if(row==0)cout<<transform_x[col];
                            if(row==1)cout<<transform_y[col];
                            if(row==2)cout<<transform_z[col];
                            if(col < points_num-1)cout<<",";
                          }
                          cout<<"]";
                          if(row < 2)cout<<","<<endl;
                        }
                        cout<<"]"<<endl;
                      }
                    }
                    if(i == sz-7)cout<<"right = ["<<cam_x[i]<<"], ["<<cam_y[i]<<"], ["<<cam_z[i]<<"]"<<endl;
                    if(i == sz-3)cout<<"upper = ["<<cam_x[i]<<"], ["<<cam_y[i]<<"], ["<<cam_z[i]<<"]"<<endl;
                  }
                }
                cout<<"Here_6"<<endl;
                Point3f Origin(corners_from_cam[2].x, corners_from_cam[2].y, corners_from_cam[2].z);
                for(int i=0; i<sz; i++)
                {
                  corners_from_cam[i] = Point3f(fabs(corners_from_cam[i].x-Origin.x),
                                                fabs(corners_from_cam[i].y-Origin.y),
                                                fabs(corners_from_cam[i].z-Origin.z));
                }
                cout<<"Here_7"<<endl;
                for(int col=0; col<5; col++)
                {
                    for(int row=2; row>=0; row--)
                    {
                        int indx = col*3 + (2-row);
                        corners_actual.push_back(Point3f((float(col))*0.043, (2-float(row))*0.043, 0.0));
                        squared_error.push_back(Point3f(pow(corners_actual[indx].x-corners_from_cam[indx].x,2),
                                                pow(corners_actual[indx].y-corners_from_cam[indx].y, 2),
                                                pow(corners_actual[indx].z-corners_from_cam[indx].z, 2)));
                        accumulator = Point3f(squared_error[indx].x+accumulator.x,
                                              squared_error[indx].y+accumulator.y,
                                              squared_error[indx].z+accumulator.z);
                    }
                }
                cout<<"Here_8"<<endl;
                if(frame_count!=0)
                {
                  for(int i=0; i<sz; i++)
                  {
                    actual_x[i] = corners_actual[i].x;
                    actual_y[i] = corners_actual[i].y;
                    actual_z[i] = corners_actual[i].z;
                  }
                  cout<<"points_from_cam = [";
                  for(int row =0;row<3;row++)
                  {
                    cout<<"[";
                    for(int col =0;col<sz;col++)
                    {
                      if(row==0)cout<<cam_x[sz-col-1];
                      if(row==1)cout<<cam_y[col];
                      if(row==2)cout<<cam_z[sz-col-1];
                      if(col != sz-1)cout<<",";
                    }
                    cout<<"]"<<endl;
                  }
                  cout<<"]"<<endl;

                  cout<<"points_from_ref = [";
                  for(int row =0;row<3;row++)
                  {
                    cout<<"[";
                    for(int col =0;col<sz;col++)
                    {
                      if(row==0)cout<<actual_x[col];
                      if(row==1)cout<<actual_y[col];
                      if(row==2)cout<<actual_z[col];
                      if(col != sz-1)cout<<",";
                    }
                    cout<<"]"<<endl;
                  }
                  cout<<"]"<<endl;
                }
//                accumulator = Point3f(sqrt(accumulator.x/float(sz)), sqrt(accumulator.y/float(sz)), sqrt(accumulator.z/float(sz)));
                cout<<"frame_no = "<<frame_count<<endl;
                cout<<"x_rmse = "<<accumulator.x<<", y_rmse = "<<accumulator.y<<", z_rmse = "<<accumulator.z<<endl;
                imshow( image_window, img );
                waitKey(1);
//                rmse_accum = Point3f(accumulator.x+rmse_accum.x,
//                                     accumulator.y+rmse_accum.y,
//                                     accumulator.z+rmse_accum.z);
            }
        }
    }
    cout<<"total_points = "<<frame_count*sz<<endl;
    rmse_accum = Point3f(sqrt(accumulator.x/(frame_count*sz)), sqrt(accumulator.y/(frame_count*sz)), sqrt(accumulator.z/(frame_count*sz)));

    cout<<"x_tot_rmse = "<<rmse_accum.x<<", y_tot_rmse = "<<rmse_accum.y<<", z_tot_rmse = "<<rmse_accum.z;

    // Signal threads to finish and wait until they do
    write_to_file("/home/ahmedshehata/Loggings/ball_trajectory_ref" , ball_trajectory_data_ref, time_stamp);
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
