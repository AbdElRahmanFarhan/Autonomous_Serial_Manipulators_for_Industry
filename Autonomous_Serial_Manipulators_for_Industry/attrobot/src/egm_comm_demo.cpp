#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include "abb_libegm/egm_controller_interface.h"
#include "abb_libegm/egm_common_auxiliary.h"
#include "abb_libegm/egm_trajectory_interface.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <unistd.h>
#include "cmath"
#include "fstream"
#include "iostream"
#include "std_msgs/Bool.h"

using namespace std;
#define	NUMBER_OF_DOFS              6
#define SAMPLE_RATE                 8

void Output_Data(abb::egm::wrapper::Output* Output, int index, int angle, int vel);
trajectory_msgs::JointTrajectory motion_msg;
bool egm_received = false;
void egm_callback(const trajectory_msgs::JointTrajectory& msg )
{
    motion_msg.points.operator =(msg.points);
    egm_received = true;
}

int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "egm_comm_demo");
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_publisher = node_handle.advertise<sensor_msgs::JointState>("/robot_joint_state", 1);
    ros::Publisher Read_Loggings_publisher = node_handle.advertise<std_msgs::Bool>("/read_loggings", 1);
    ros::Publisher EGM_started_publisher = node_handle.advertise<std_msgs::Bool>("/EGM_started", 1);
    ros::Subscriber egm_sub = node_handle.subscribe("/robot_trajectory_planning", 5, &egm_callback);
    std_msgs::Bool read_loggings_msg;
    std_msgs::Bool egm_started_msg;
    std_msgs::Bool::ConstPtr stop_msg;
    std_msgs::Bool::ConstPtr start_msg;
    read_loggings_msg.data = true;
    egm_started_msg.data = true;
    bool wait = true;
    // ABB Communication initialization
    boost::asio::io_service io_service;
//    boost::asio::io_service::work work(io_service);
    abb::egm::BaseConfiguration config;
    config.use_logging = true;
    config.use_velocity_outputs = false;
    abb::egm::EGMControllerInterface Server(io_service,6510,config);
    boost::thread_group thread_group;
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
//    boost::thread thread([&io_service]() { io_service.run(); });
    abb::egm::wrapper::Input Input;
    abb::egm::wrapper::Output Output;

    for(int i=0;i<NUMBER_OF_DOFS;i++)
    {
//        Output.mutable_robot()->mutable_joints()->mutable_velocity()->add_values(i);
        Output.mutable_robot()->mutable_joints()->mutable_position()->add_values(i);
//        Output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(i,0);

    }
\
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(0, 28);
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(1, 34.72);
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(2, 1.52);
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(3, 34.86);
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(4, -42.73);
    Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(5, -27.7);

//    ROS_INFO("Wait for EGM Start Messege");
//    start_msg = ros::topic::waitForMessage <std_msgs::Bool> ("/Start_EGM_Motion", node_handle);
//    ROS_INFO("EGM Start Messege Received");

    // waiting for new trajectory message
//    ROS_INFO("waiting for motion msg");
//    motion_msg = ros::topic::waitForMessage <trajectory_msgs::JointTrajectory> ("/robot_trajectory_planning", node_handle);
//    ROS_INFO("EGM_NODE :: motion msg recieved");
//    EGM_started_publisher.publish(egm_started_msg);


//    ROS_INFO("1: Wait for an EGM communication session to start...");
//    while(ros::ok() && wait)
//    {
//      if(Server.isConnected())
//      {
//        ROS_INFO("<<<<<<<<<<<<<<<<<Connected>>>>>>>>>>>>>>>>>>>>>>>>>");
//        if(Server.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
//        {
//          ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
//          io_service.stop();
//      //    thread.join();
//          thread_group.join_all();
//          return 0;
//        }
//        else
//        {
//            wait = Server.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
//        }
//      }
//      ros::Duration(0.5).sleep();
//    }
//    ros::Duration(0.01).sleep();

//    Server.waitForMessage();
//    Server.read(&Input);
//    sensor_msgs::JointState state_msg;
//    for(int j_num=0; j_num<NUMBER_OF_DOFS; j_num++)
//    {
//      state_msg.position.push_back(Input.feedback().robot().joints().position().values().Get(j_num)*(M_PI/180.0));
//    }
//    joint_state_publisher.publish(state_msg);


    int sequence_number = 0, prev_msgs_counter = 0;
    bool moved_flag = false, motion_error = false, measure_time = false;
    double t1 = 0, t2 = 0, delta_t = 0;
    while (ros::ok())
    {
        if (egm_received)
        {
            int p_sz = motion_msg.points.size();
            ROS_INFO("trajectory size = %d ",p_sz);
            for(int p_num = 0; p_num < p_sz; p_num++)
            {
//                t1 = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*(pow(10,-9));
//                ROS_INFO("server waiting");
                if(Server.waitForMessage(500))
                {
//                    t2 = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*(pow(10,-9));
//                    delta_t = t2 - t1;
//                    if(delta_t <= 0.0001)
//                    {
//                        p_num--;
//                        if(!measure_time)ROS_INFO("Flushing the buffer");
//                        measure_time = true;
//        //                Server.write(Output);
//                        continue;
//                    }
                    Server.read(&Input);
                    sequence_number = Input.header().sequence_number();
//                    ROS_INFO("Waited %d ", p_num);
                    ROS_INFO("seq_no_traj = %d", sequence_number);
                    if(!moved_flag)
                    {
                        for(int j_num=0;j_num<6;j_num++)
                        {
                            if(fabs(Input.feedback().robot().joints().velocity().values(j_num)) > 0.00000001 )
                            {
                                moved_flag = true;
                                ROS_INFO("Moved!!!!!!!!!!!!!!!!!!!!!!!!!!");
                                break;
                            }
                        }
                    }

                    for(int j_num=0;j_num<NUMBER_OF_DOFS;j_num++)
                    {
//                        Output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(j_num, motion_msg.points[p_num].velocities[j_num] * (180.0/M_PI));
                        Output.mutable_robot()->mutable_joints()->mutable_position()->set_values(j_num, motion_msg.points[p_num].positions[j_num] * (180.0/M_PI));
                    }
                    Server.write(Output);
                }

                if(Server.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_STOPPED )
                {
                    ROS_INFO("<<<<<<<<<<<<<<Motion Error>>>>>>>>>>>>>>");
                    motion_error = true;
                    break;
                }
            }
            break;
        }
        else
        {
            if(Server.waitForMessage(500))
            {
                Server.read(&Input);
                sequence_number = Input.header().sequence_number();
                if(sequence_number % 100 == 0)
                    ROS_INFO("seq_no = %d", sequence_number);
                Server.write(Output);
                for (int i=0; i<8; i++)
                {
                    ros::spinOnce();
                    ros::Duration(0.001).sleep();
                }
            }
        }
    }

    moved_flag = true;
    int counter = 0;
    while(moved_flag)
    {
        counter = 0;
        if(Server.waitForMessage(500))
        {
            Server.read(&Input);
            ROS_INFO("Waiting to stop, vel_1 = %f",fabs(Input.feedback().robot().joints().velocity().values(1)));
            for(int j_num=0;j_num<6;j_num++)
            {
                if(fabs(Input.feedback().robot().joints().velocity().values(j_num)) > 10 )
                {
                    counter++;
                }
            }
            if(counter == 0)
            {
                moved_flag = false;
                ROS_INFO("<<<<<<<<<<<<<<<STOPPED>>>>>>>>>>>>>>>>>>>");
            }
            Server.write(Output);

        }
        if(Server.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_STOPPED )
        {
            ROS_INFO("<<<<<<<<<<<<<<Motion Error>>>>>>>>>>>>>>");
            motion_error = true;
            break;
        }
    }
//    ros::Duration(2).sleep();
    EGM_started_publisher.publish(egm_started_msg);

    if(!motion_error){
    ROS_INFO("waiting for Stopping msg");
    stop_msg = ros::topic::waitForMessage <std_msgs::Bool> ("/Stop_EGM_Motion", node_handle);
    ROS_INFO("Stop msg recieved");
    }

    if(!motion_error)ros::Duration(2).sleep();
    Read_Loggings_publisher.publish(read_loggings_msg);

    io_service.stop();
//    thread.join();
    thread_group.join_all();
    return 0;
}
//void Output_Data(abb::egm::wrapper::Output* Output, int index, int angle, int vel)
//{
//    Output->mutable_robot()->mutable_joints()->mutable_position()->set_values(index, angle);
//    Output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(index, vel);
//}
