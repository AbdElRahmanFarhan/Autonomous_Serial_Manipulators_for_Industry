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
int width = 480, height = 270;
const char* image_window = "Source Image";

vector<Vec3f> circles;
vector<Vec3f>control_points_dist;
high_resolution_clock::time_point t_frame1;
high_resolution_clock::time_point t_frame2;
vector<float> dist_3d(const rs2::depth_frame& frame, pixel u);
void write_to_file(std::string, std::vector<Vec3f>, std::vector<double>);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);


//---------------------------------------------------------------
Mat src, src_gray;
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
    int frame_count = 0;
    //vector<Point> detection_trail(20);
    vector<Vec3f> ball_trajectory_data_ref;
    vector<double> time_stamp;
    //camera variables:
    //-------------------
    namedWindow( image_window, WINDOW_AUTOSIZE );
    rs2::decimation_filter dec;
    rs2::spatial_filter spat;
    rs2::hole_filling_filter holes;

    //processing options
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    holes.set_option(RS2_OPTION_HOLES_FILL, 0);

    //stream profile and starting
    rs2::pipeline pipe;
    rs2::config cfg;

    //cfg.enable_device_from_file();
    cfg.enable_stream(RS2_STREAM_DEPTH,848,480,RS2_FORMAT_Z16,60);// Enable default depth
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGBA8,60);
    auto profile = pipe.start(cfg);

    //aligning of the color and depth streams
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    rs2::align align(align_to);

    //geting sensor info
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter

    }
    if (sensor.supports(RS2_OPTION_LASER_POWER))
    {

        sensor.set_option(RS2_OPTION_LASER_POWER, 200);

    }
    auto range = sensor.get_option_range(RS2_OPTION_VISUAL_PRESET);
    for (auto i = range.min; i < range.max; i += range.step)
        if (std::string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "Default")
            sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);

    // After initial post-processing, frames will flow into this queue:
    rs2::frame_queue postprocessed_frames;

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
            data = data.apply_filter(align); //Here we align
            t_frame2 = high_resolution_clock::now();
        auto duration2 = duration_cast<milliseconds>(t_frame2-t_frame1).count();
        cout<<"Alignment time="<<duration2<<endl;

            source.frame_ready(data);
        });

        // Indicate that we want the results of frame_processor
        // to be pushed into postprocessed_frames queue
        frame_processor >> postprocessed_frames;

        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs)) frame_processor.invoke(fs);
        }
    });

    //Our main while loop:
    //------------------------
    while (waitKey(1) != int('q') && cvGetWindowHandle(image_window)) // Application still alive?
    {
        static rs2::frameset current_frameset;
        if(postprocessed_frames.poll_for_frame(&current_frameset))
        {
            frame_count += 1;
            if (current_frameset) //If frame is not yet constructed (ref_assign_flag==0), construct
            {
                auto depth_1 = current_frameset.get_depth_frame(); //Aligned depth frame
                auto color_1 = current_frameset.get_color_frame();
                Mat img = frame_to_mat(color_1); //source img ready

                // Select ROI
                r = selectROI(img);
                // Crop image
                Mat imgCrop = img(r);

                cvtColor( imgCrop, src_gray, COLOR_BGR2GRAY );
                namedWindow( source_window );
                imshow( source_window, img );
            }

        }
    }

    // Signal threads to finish and wait until they do
    write_to_file("/home/ahmedshehata/Loggings/ball_trajectory_ref" , ball_trajectory_data_ref, time_stamp);
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
vector<float> dist_3d(const rs2::depth_frame& frame, pixel u)
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

    vector<float> point;
    point.push_back(upoint[0]);
    point.push_back(upoint[1]);
    point.push_back(upoint[2]);

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
