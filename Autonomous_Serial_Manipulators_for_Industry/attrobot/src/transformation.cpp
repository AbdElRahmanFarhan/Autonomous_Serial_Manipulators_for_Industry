#include "abb_librws/rws_state_machine_interface.h"
#include "stdio.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <cstdlib>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include  "math.h"
#include  "std_msgs/Float32MultiArray.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"


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
// rws constants
//

#define     DOF                 6
#define     NUM_OF_DESIRED_POS  13
#define     WAITING_TO_REACH    7
#define     POS_TOLERANCE       0.1
#define     COMM_TIMEOUT        1000
#define     RAMP_IN_TIME        0.005
#define     RAMP_OUT_TIME       0.005
#define     COND_TIME           60
#define     COMM_DELAY          0.01
#define     V_TCP               300
#define     V_ORI               30
#define     PORT_REAL_ROBOT     80
#define     LP                  0
#define     K                   0
#define     DAMPING_FACTOR      0.8
#define     MAX_SPEED           0.1
#define     MAX_SPEED_DEV       DAMPING_FACTOR*(MAX_SPEED * (180/M_PI))

const string IP_ADDRESS_REAL_ROBOT = "192.168.1.100";
RWSStateMachineInterface IRB_120(IP_ADDRESS_REAL_ROBOT, PORT_REAL_ROBOT);

const string TASK = SystemConstants::RAPID::TASK_ROB_1;
const string ROBOT = SystemConstants::General::MECHANICAL_UNIT_ROB_1;
const string SIGNAL_EGM_STOP = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_STOP;
const string SIGNAL_EGM_START = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_JOINT;
const string SIGNAL_RUN_RAPID = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::RUN_RAPID_ROUTINE;
const string HIGH = SystemConstants::IOSignals::HIGH;
const string LOW = SystemConstants::IOSignals::LOW;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transformation");
    ros::NodeHandle node_handle;

    //
    // desired positions
    //
    JointTarget start_pos;
    SpeedData speed_data;
    speed_data.v_tcp = V_TCP;
    speed_data.v_ori = V_ORI;

    const float desired_positions[NUM_OF_DESIRED_POS][DOF] =
    {{-43.10, 10.42, -46.38, 59.77, 53.61, -37.63},
     {0, 0, 0, 0, 0, 0},
     {-33.59, 32.04, -7.85, -57.98, -40.85, 53.67},
     {-36.92, 27.77, -13.81, -37.72, -17, 26.42},
     {-55.89, -33.81, 26.41, -97.88, -54.78, 108.53},
     {4.91, 53.11, -20.42, 33.48, -35.93, -14.08},
     {-51.41, 68.24, -48.02, -79.36, -63.96, 69.92},
     {-74.23, 56.63, -30.45, -82.91, -74.17, 68.95},
     {-73.89, 45.09, -97.41, 56.63, 74.93, -31.41},
     {-87.16, 50.73, -49.15, -89.97, -97.77, 90.79},
     {-15.2, 60.94, -42.99, -3.25, -18.92, 6.13},
     {-41.25, 58.46, 26.54, -16.13, -84.03, 0.54},
     {-75.33, 8.36, -21.04, -100.96, -53.66, 108.76}};

    //
    // egm settings
    //
    RWSStateMachineInterface::EGMSettings egm_settings;

    egm_settings.activate.tool.robhold = true;
    // racket
    egm_settings.activate.tool.tframe.pos.x = -66.5 ;
    egm_settings.activate.tool.tframe.pos.y = -8;
    egm_settings.activate.tool.tframe.pos.z = 188.3;
    // chessboard
//    egm_settings.activate.tool.tframe.pos.x = -50-63;
//    egm_settings.activate.tool.tframe.pos.y = -13;
//    egm_settings.activate.tool.tframe.pos.z = 338.3 - (63+4*43);
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

    //
    // reset all signals and start the system
    //
    Reset_Signals();
    Restart_System();
    Restart_System();
    ros::Duration(2).sleep();

    //
    // move to the start target
    //
    start_pos.robax.rax_1 = 0;
    start_pos.robax.rax_2 = 0;
    start_pos.robax.rax_3 = 0;
    start_pos.robax.rax_4 = 0;
    start_pos.robax.rax_5 = 0;
    start_pos.robax.rax_6 = 0;
    IRB_120.services().rapid().setMoveSpeed(TASK, speed_data);
    IRB_120.services().rapid().runMoveAbsJ(TASK, start_pos);

    int wait_for_key;
    float x[NUM_OF_DESIRED_POS], y[NUM_OF_DESIRED_POS], z[NUM_OF_DESIRED_POS];

        for (int i=0; i<NUM_OF_DESIRED_POS; i++)
        {
            //
            // move to the desired target
            //
            cin >> wait_for_key;
            if(i >= NUM_OF_DESIRED_POS)
                break;
            cout<< "done"<<endl;

            JointTarget pos;
            pos.robax.rax_1 = desired_positions[i][0];
            pos.robax.rax_2 = desired_positions[i][1];
            pos.robax.rax_3 = desired_positions[i][2];
            pos.robax.rax_4 = desired_positions[i][3];
            pos.robax.rax_5 = desired_positions[i][4];
            pos.robax.rax_6 = desired_positions[i][5];
            IRB_120.services().rapid().setMoveSpeed(TASK, speed_data);
            IRB_120.services().rapid().runMoveAbsJ(TASK, pos);

            //
            // get current cartesian state
            //
            cin >> wait_for_key;

            RobTarget cartesian_pos;
            while (!IRB_120.getMechanicalUnitRobTarget(ROBOT, &cartesian_pos))
                ros::Duration(COMM_DELAY).sleep();

            x[i] = cartesian_pos.pos.x.value;
            y[i] = cartesian_pos.pos.y.value;
            z[i] = cartesian_pos.pos.z.value;
        }
    cout << "[";
    for(int j=0;j<3;j++)
    {
        cout<<"[";
        for(int i=0;i<NUM_OF_DESIRED_POS;i++)
        {
            if(j==0)cout<<x[i]*0.001;
            else if(j==1)cout<<y[i]*0.001;
            else cout<<z[i]*0.001;
            if(i != (NUM_OF_DESIRED_POS-1))cout<<",";
        }
        if (j<2)
            cout << "],";
        else
            cout << "]";
    }
    cout<<"]"<<endl;

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
