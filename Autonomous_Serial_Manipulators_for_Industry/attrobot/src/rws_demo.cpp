#include "abb_librws/rws_state_machine_interface.h"
#include "stdio.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <cstdlib>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "attrobot/control_params.h"
#include  "math.h"

using namespace abb;
using namespace rws;
using namespace std;

void Reset_Signals();
void Reset_EGM_STOP();
void Reset_EGM_START();
void Reset_RUN_RAPID();
void Set_EGM_STOP();
void Set_EGM_START();
void Set_RUN_RAPID();
void Restart_System();

//
// state machine constants
//
#define     NUM_STATES              8
#define     RESTART_STATE           0
#define     IDLE_STATE              1
#define     EGM_START_STATE         2
#define     EGM_MOVING_STATE        3
#define     EGM_STOP_STATE          4
#define     RAPID_START_STATE       5
#define     RAPID_MOVING_STATE      6
#define     RAPID_STOP_STATE        7
#define     IS_STATIONARY           2
#define     MOTOR_OFF               2
#define     ROBOT_AT_HOME           2
#define     STATE_MACHINE_DELAY     0.3
//
// rws constants
//
#define     POS_TOLERANCE       0.01
#define     COMM_TIMEOUT        1
#define     RAMP_IN_TIME        0.005
#define     RAMP_OUT_TIME       0.005
#define     COND_TIME           100 //Changed from 0.01
#define     COMM_DELAY          0.01
#define     V_TCP               700
#define     V_ORI               100
#define     PORT_REAL_ROBOT     80
#define     PORT_ROBOT_STUDIO   8080
#define     LP                  0
#define     K                   0.4
#define     DAMPING_FACTOR      1.2
#define     MAX_SPEED           4.4
#define     MAX_SPEED_DEV       DAMPING_FACTOR * (MAX_SPEED * (180/M_PI))
#define     SAMPLE_RATE         8
#define     COND_MIN_MAX        1

const string IP_ADDRESS_REAL_ROBOT = "192.168.1.100";
const string IP_ADDRESS_DHCP = "192.168.10.12";
const string IP_ADDRESS_WIFI = "192.168.1.106";

RWSStateMachineInterface IRB_120(IP_ADDRESS_WIFI, PORT_ROBOT_STUDIO);

const string TASK = SystemConstants::RAPID::TASK_ROB_1;
const string ROBOT = SystemConstants::General::MECHANICAL_UNIT_ROB_1;
const string SIGNAL_EGM_STOP = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_STOP;
const string SIGNAL_EGM_START = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_JOINT;
const string SIGNAL_RUN_RAPID = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::RUN_RAPID_ROUTINE;
const string HIGH = SystemConstants::IOSignals::HIGH;
const string LOW = SystemConstants::IOSignals::LOW;

int main(int argc, char** argv)
{
    //
    // ros initializations
    //
    ros::init(argc, argv, "rws_demo");
    ros::NodeHandle node_handle;
    trajectory_msgs::JointTrajectory::ConstPtr egm_started_msg;
    std_msgs::Bool stop_msg;
    std_msgs::Bool start_msg;
    start_msg.data = true;
    stop_msg.data = true;

    ros::Publisher EGM_Stop_publisher = node_handle.advertise<std_msgs::Bool>("/Stop_EGM_Motion", 1);
    ros::Publisher EGM_Start_publisher = node_handle.advertise<std_msgs::Bool>("/Start_EGM_Motion", 1);
    trajectory_msgs::JointTrajectory::ConstPtr motion_msg;
//    ros::Duration(1).sleep();

    JointTarget start_pos, wait_pos, current_joint_values;
    SpeedData speed_data_start, speed_data_wait;
    //
    // start position
    //
    speed_data_start.v_tcp = V_TCP;
    speed_data_start.v_ori = V_ORI;
    start_pos.robax.rax_1 = 0;
    start_pos.robax.rax_2 = 0;
    start_pos.robax.rax_3 = 0;
    start_pos.robax.rax_4 = 0;
    start_pos.robax.rax_5 = 0;
    start_pos.robax.rax_6 = 0;

    //
    // waiting position
    //
    speed_data_wait.v_tcp = V_TCP;
    speed_data_wait.v_ori = V_ORI;

    wait_pos.robax.rax_1 = 28;
    wait_pos.robax.rax_2 = 34.72;
    wait_pos.robax.rax_3 = 1.52;
    wait_pos.robax.rax_4 = 34.86;
    wait_pos.robax.rax_5 = -42.73;
    wait_pos.robax.rax_6 = -27.7;
//    wait_pos.robax.rax_1 = 63.13;
//    wait_pos.robax.rax_2 = 40.94;
//    wait_pos.robax.rax_3 = 25.21;
//    wait_pos.robax.rax_4 = 62.5;
//    wait_pos.robax.rax_5 = -81.6;
//    wait_pos.robax.rax_6 = -19.9;
//    wait_pos.robax.rax_1 = -21.9;
//    wait_pos.robax.rax_2 = 30.18;
//    wait_pos.robax.rax_3 = 47.41;
//    wait_pos.robax.rax_4 = -23.8;
//    wait_pos.robax.rax_5 = -81.5;
//    wait_pos.robax.rax_6 = 6.14;


    //
    // egm settings
    //
    RWSStateMachineInterface::EGMSettings egm_settings;

    egm_settings.activate.tool.robhold = true;
    egm_settings.activate.tool.tframe.pos.x = -66.5 ;
    egm_settings.activate.tool.tframe.pos.y = -8;
    egm_settings.activate.tool.tframe.pos.z = 188.3;
    egm_settings.activate.tool.tframe.rot.q1 = 0.7071068;
    egm_settings.activate.tool.tframe.rot.q2 = 0;
    egm_settings.activate.tool.tframe.rot.q3 = -0.7071068;
    egm_settings.activate.tool.tframe.rot.q4 = 0;
    egm_settings.activate.tool.tload.mass = 0.001;
    egm_settings.activate.tool.tload.cog.x = 0;
    egm_settings.activate.tool.tload.cog.y = 0;
    egm_settings.activate.tool.tload.cog.z = 0.001;
    egm_settings.activate.tool.tload.aom.q1 = 1;
    egm_settings.activate.tool.tload.aom.q2 = 0;
    egm_settings.activate.tool.tload.aom.q3 = 0;
    egm_settings.activate.tool.tload.aom.q4 = 0;
    egm_settings.activate.tool.tload.ix = 0;
    egm_settings.activate.tool.tload.iy = 0;
    egm_settings.activate.tool.tload.iz = 0;

    egm_settings.activate.wobj.robhold = false;
    egm_settings.activate.wobj.ufprog = true;
    egm_settings.activate.wobj.ufmec.value = "\0";
    egm_settings.activate.wobj.uframe.pos.x = 0;
    egm_settings.activate.wobj.uframe.pos.y = 0;
    egm_settings.activate.wobj.uframe.pos.z = 0;
    egm_settings.activate.wobj.uframe.rot.q1 = 1;
    egm_settings.activate.wobj.uframe.rot.q2 = 0;
    egm_settings.activate.wobj.uframe.rot.q3 = 0;
    egm_settings.activate.wobj.uframe.rot.q4 = 0;
    egm_settings.activate.wobj.oframe.pos.x = 0;
    egm_settings.activate.wobj.oframe.pos.y = 0;
    egm_settings.activate.wobj.oframe.pos.z = 0;
    egm_settings.activate.wobj.oframe.rot.q1 = 1;
    egm_settings.activate.wobj.oframe.rot.q2 = 0;
    egm_settings.activate.wobj.oframe.rot.q3 = 0;
    egm_settings.activate.wobj.oframe.rot.q4 = 0;

    egm_settings.allow_egm_motions = true;
    egm_settings.setup_uc.comm_timeout = COMM_TIMEOUT;
    egm_settings.setup_uc.use_filtering = true;
    egm_settings.run.ramp_in_time = RAMP_IN_TIME;
    egm_settings.stop.ramp_out_time = RAMP_OUT_TIME;
    egm_settings.run.cond_time = COND_TIME;

    egm_settings.activate.lp_filter = LP;
    egm_settings.activate.max_speed_deviation = MAX_SPEED_DEV;
    egm_settings.run.pos_corr_gain = K;
    egm_settings.activate.sample_rate = SAMPLE_RATE;
    egm_settings.activate.cond_min_max = COND_MIN_MAX;

    //
    // moore state machine
    // inputs
    //
    int is_stationary, motors_off, robot_at_home;

    //
    // states
    //
    int current_state = RESTART_STATE;
    const int GET_NEXT_STATE [NUM_STATES][IS_STATIONARY][MOTOR_OFF][ROBOT_AT_HOME] = {
        {{{RESTART_STATE, RESTART_STATE}, {RESTART_STATE, RESTART_STATE}},
         {{IDLE_STATE, IDLE_STATE}, {RESTART_STATE, RESTART_STATE}}
        },
        {{{RESTART_STATE, RESTART_STATE}, {RESTART_STATE, RESTART_STATE}},
         {{RAPID_START_STATE, EGM_START_STATE}, {RESTART_STATE, RESTART_STATE}}
        },
        {{{EGM_MOVING_STATE, EGM_MOVING_STATE}, {EGM_STOP_STATE, EGM_STOP_STATE}},
         {{EGM_STOP_STATE, EGM_MOVING_STATE}, {EGM_STOP_STATE, EGM_STOP_STATE}}
        },
        {{{EGM_MOVING_STATE, EGM_MOVING_STATE}, {EGM_STOP_STATE, EGM_STOP_STATE}},
         {{EGM_STOP_STATE, EGM_STOP_STATE}, {EGM_STOP_STATE, EGM_STOP_STATE}}
        },
        {{{RESTART_STATE, RESTART_STATE}, {RESTART_STATE, RESTART_STATE}},
         {{IDLE_STATE, EGM_START_STATE}, {RESTART_STATE, RESTART_STATE}}
        },
        {{{RAPID_MOVING_STATE, RAPID_MOVING_STATE}, {RAPID_STOP_STATE, RAPID_STOP_STATE}},
         {{RAPID_STOP_STATE, RAPID_STOP_STATE}, {RAPID_STOP_STATE, RAPID_STOP_STATE}}
        },
        {{{RAPID_MOVING_STATE, RAPID_MOVING_STATE}, {RAPID_STOP_STATE, RAPID_STOP_STATE}},
         {{RAPID_STOP_STATE, RAPID_STOP_STATE}, {RAPID_STOP_STATE, RAPID_STOP_STATE}}
        },
        {{{RESTART_STATE, RESTART_STATE}, {RESTART_STATE, RESTART_STATE}},
         {{RESTART_STATE, IDLE_STATE}, {RESTART_STATE, RESTART_STATE}}
        }};

    while(ros::ok())
    {
        //
        // update ouptputs
        //
        switch (current_state)
        {
            case RESTART_STATE:
                cout <<"current state is RESTART_STATE"<< endl;
                cout <<"output is restart the system"<<endl;
                Reset_Signals();
                Restart_System();
                break;

            case IDLE_STATE:
                cout <<"current state is IDLE_STATE"<< endl;
                cout <<"output is reset all signals"<<endl;
                Reset_Signals();
                break;

            case EGM_START_STATE:
                cout <<"current state is EGM_START_STATE"<< endl;
                cout <<"output is start EGM node"<<endl;
                IRB_120.services().egm().setSettings(TASK, egm_settings);
//                EGM_Start_publisher.publish(start_msg);
                // waiting for new trajectory message
//                ROS_INFO("1 :: waiting for egm to start");
//                egm_started_msg = ros::topic::waitForMessage <std_msgs::Bool> ("/EGM_started", node_handle);
//                ROS_INFO("1 :: egm started");

//                ros::Duration(COMM_DELAY).sleep();

                //
                // run EGM session
                //
                Reset_EGM_STOP();
                Set_EGM_START();
//                ros::Duration(COMM_DELAY).sleep();
                ROS_INFO("2 :: waiting for egm to start");
                egm_started_msg = ros::topic::waitForMessage <trajectory_msgs::JointTrajectory> ("/robot_trajectory_planning", node_handle);
                ros::Duration(1.5).sleep();
                ROS_INFO("2 :: egm started");
                break;

            case EGM_MOVING_STATE:
                cout <<"current state is EGM_MOVING_STATE"<< endl;
                cout <<"output is do nothing"<<endl;
                break;

            case EGM_STOP_STATE:
                cout <<"current state is EGM_STOP_STATE"<< endl;
                cout <<"output is stop EGM node"<<endl;
                EGM_Stop_publisher.publish(stop_msg);
                ros::Duration(COMM_DELAY).sleep();
                Reset_EGM_START();
                ros::Duration(COMM_DELAY).sleep();
                Set_EGM_STOP();
                break;

            case RAPID_START_STATE:
                cout <<"current state is RAPID_START_STATE"<< endl;
                cout <<"output is MoveAbsJ"<<endl;
//                IRB_120.services().rapid().setMoveSpeed(TASK, speed_data_start);
//                IRB_120.services().rapid().runMoveAbsJ(TASK, start_pos);
//                ros::Duration(5).sleep();
                IRB_120.services().rapid().setMoveSpeed(TASK, speed_data_wait);
                IRB_120.services().rapid().runMoveAbsJ(TASK, wait_pos);
                break;

            case RAPID_MOVING_STATE:
                cout <<"current state is RAPID_MOVING_STATE"<< endl;
                cout <<"output is do nothing"<<endl;
                break;

            case RAPID_STOP_STATE:
                cout <<"current state is RAPID_STOP_STATE"<< endl;
                cout <<"output is stop RAPID routine"<<endl;
                Reset_RUN_RAPID();
                break;

            default:
                cout <<"current state is unidentified"<<endl;
                break;
        }

        ros::Duration(STATE_MACHINE_DELAY).sleep();
        //
        // read all inputs
        //
        is_stationary = int(IRB_120.services().main().isStationary(ROBOT).isTrue());
        motors_off = int(IRB_120.isRAPIDRunning().isFalse());
        IRB_120.getMechanicalUnitJointTarget(ROBOT, &current_joint_values);
        if(fabs(current_joint_values.robax.rax_1.value-wait_pos.robax.rax_1.value) <= POS_TOLERANCE &&
                            fabs(current_joint_values.robax.rax_2.value-wait_pos.robax.rax_2.value) <= POS_TOLERANCE &&
                            fabs(current_joint_values.robax.rax_3.value-wait_pos.robax.rax_3.value) <= POS_TOLERANCE &&
                            fabs(current_joint_values.robax.rax_4.value-wait_pos.robax.rax_4.value) <= POS_TOLERANCE &&
                            fabs(current_joint_values.robax.rax_5.value-wait_pos.robax.rax_5.value) <= POS_TOLERANCE &&
                            fabs(current_joint_values.robax.rax_6.value-wait_pos.robax.rax_6.value) <= POS_TOLERANCE)
            robot_at_home = 1;
        else
            robot_at_home = 0;
        //
        // print inputs
        //
        cout <<"----------------------------------------------"<<endl;
        cout <<"is_stationary = "<<is_stationary<<endl;
        cout <<"motors_off = "<<motors_off<<endl;
        cout <<"robot_at_home = "<<robot_at_home<<endl;
        //
        // update the current state
        //
        current_state = GET_NEXT_STATE [current_state][is_stationary][motors_off][robot_at_home];
    }
    return 0;
}

void Reset_Signals()
{
    Reset_EGM_STOP();
    Reset_EGM_START();
    Reset_RUN_RAPID();
}
void Reset_EGM_STOP()
{
    if(IRB_120.getIOSignal(SIGNAL_EGM_STOP) == HIGH)
        IRB_120.setIOSignal(SIGNAL_EGM_STOP, LOW);

}
void Reset_EGM_START()
{
    if(IRB_120.getIOSignal(SIGNAL_EGM_START) == HIGH)
        IRB_120.setIOSignal(SIGNAL_EGM_START, LOW);

}
void Reset_RUN_RAPID()
{
    if(IRB_120.getIOSignal(SIGNAL_RUN_RAPID) == HIGH)
        IRB_120.setIOSignal(SIGNAL_RUN_RAPID, LOW);

}
void Set_EGM_STOP()
{
    if(IRB_120.getIOSignal(SIGNAL_EGM_STOP) == LOW)
        IRB_120.setIOSignal(SIGNAL_EGM_STOP, HIGH);

}
void Set_EGM_START()
{
    if(IRB_120.getIOSignal(SIGNAL_EGM_START) == LOW)
        IRB_120.setIOSignal(SIGNAL_EGM_START, HIGH);
}
void Set_RUN_RAPID()
{
    if(IRB_120.getIOSignal(SIGNAL_RUN_RAPID) == LOW)
        IRB_120.setIOSignal(SIGNAL_RUN_RAPID, HIGH);

}

void Restart_System()
{
    if(IRB_120.isRAPIDRunning().isTrue())
    {
        IRB_120.stopRAPIDExecution();
        ros::Duration(COMM_DELAY).sleep();
        IRB_120.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        IRB_120.startRAPIDExecution();
    }
    else
    {
        IRB_120.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        if(IRB_120.isMotorOn().isFalse())
            IRB_120.setMotorsOn();
        ros::Duration(COMM_DELAY).sleep();
        IRB_120.startRAPIDExecution();
    }
}
