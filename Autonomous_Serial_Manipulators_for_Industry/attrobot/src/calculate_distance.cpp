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
#include  "std_msgs/Float32MultiArray.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "cmath"


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

#define     RACKET_X            191
#define     RACKET_Y            -9
#define     RACKET_Z            67
#define     DOF                 6
#define     NUM_OF_DESIRED_POS  64
#define     WATING_TO_REACH     7
#define     POS_TOLERANCE       0.1
#define     COMM_TIMEOUT        1000
#define     RAMP_IN_TIME        0.005
#define     RAMP_OUT_TIME       0.005
#define     COND_TIME           60
#define     COMM_DELAY          0.01
#define     V_TCP               300
#define     V_ORI               70
#define     PORT_ROBOT_STUDIO   8080
#define     LP                  0
#define     K                   0
#define     DAMPING_FACTOR      0.8
#define     MAX_SPEED           0.1
#define     MAX_SPEED_DEV       DAMPING_FACTOR*(MAX_SPEED * (180/M_PI))

const string IP_ADDRESS_DHCP = "192.168.10.12";
RWSStateMachineInterface IRB_120(IP_ADDRESS_DHCP, PORT_ROBOT_STUDIO);

const string TASK = SystemConstants::RAPID::TASK_ROB_1;
const string ROBOT = SystemConstants::General::MECHANICAL_UNIT_ROB_1;
const string SIGNAL_EGM_STOP = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_STOP;
const string SIGNAL_EGM_START = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_JOINT;
const string SIGNAL_RUN_RAPID = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::RUN_RAPID_ROUTINE;
const string HIGH = SystemConstants::IOSignals::HIGH;
const string LOW = SystemConstants::IOSignals::LOW;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculate_distance");
    ros::NodeHandle node_handle;

    //
    // desired positions
    //
    JointTarget start_pos;
    SpeedData speed_data;
    speed_data.v_tcp = V_TCP;
    speed_data.v_ori = V_ORI;
    const float desired_positions[NUM_OF_DESIRED_POS][DOF] =
    {
    { 30.0 , 30.0 , 30.0 , 30.0 , 30.0 , 30.0 },
    { 30.0 , 30.0 , 30.0 , 30.0 , 30.0 , -30.0 },
    { 30.0 , 30.0 , 30.0 , 30.0 , -30.0 , 30.0 },
    { 30.0 , 30.0 , 30.0 , 30.0 , -30.0 , -30.0 },
    { 30.0 , 30.0 , 30.0 , -30.0 , 30.0 , 30.0 },
    { 30.0 , 30.0 , 30.0 , -30.0 , 30.0 , -30.0 },
    { 30.0 , 30.0 , 30.0 , -30.0 , -30.0 , 30.0 },
    { 30.0 , 30.0 , 30.0 , -30.0 , -30.0 , -30.0 },
    { 30.0 , 30.0 , -30.0 , 30.0 , 30.0 , 30.0 },
    { 30.0 , 30.0 , -30.0 , 30.0 , 30.0 , -30.0 },
    { 30.0 , 30.0 , -30.0 , 30.0 , -30.0 , 30.0 },
    { 30.0 , 30.0 , -30.0 , 30.0 , -30.0 , -30.0 },
    { 30.0 , 30.0 , -30.0 , -30.0 , 30.0 , 30.0 },
    { 30.0 , 30.0 , -30.0 , -30.0 , 30.0 , -30.0 },
    { 30.0 , 30.0 , -30.0 , -30.0 , -30.0 , 30.0 },
    { 30.0 , 30.0 , -30.0 , -30.0 , -30.0 , -30.0 },
    { 30.0 , -30.0 , 30.0 , 30.0 , 30.0 , 30.0 },
    { 30.0 , -30.0 , 30.0 , 30.0 , 30.0 , -30.0 },
    { 30.0 , -30.0 , 30.0 , 30.0 , -30.0 , 30.0 },
    { 30.0 , -30.0 , 30.0 , 30.0 , -30.0 , -30.0 },
    { 30.0 , -30.0 , 30.0 , -30.0 , 30.0 , 30.0 },
    { 30.0 , -30.0 , 30.0 , -30.0 , 30.0 , -30.0 },
    { 30.0 , -30.0 , 30.0 , -30.0 , -30.0 , 30.0 },
    { 30.0 , -30.0 , 30.0 , -30.0 , -30.0 , -30.0 },
    { 30.0 , -30.0 , -30.0 , 30.0 , 30.0 , 30.0 },
    { 30.0 , -30.0 , -30.0 , 30.0 , 30.0 , -30.0 },
    { 30.0 , -30.0 , -30.0 , 30.0 , -30.0 , 30.0 },
    { 30.0 , -30.0 , -30.0 , 30.0 , -30.0 , -30.0 },
    { 30.0 , -30.0 , -30.0 , -30.0 , 30.0 , 30.0 },
    { 30.0 , -30.0 , -30.0 , -30.0 , 30.0 , -30.0 },
    { 30.0 , -30.0 , -30.0 , -30.0 , -30.0 , 30.0 },
    { 30.0 , -30.0 , -30.0 , -30.0 , -30.0 , -30.0 },
    { -30.0 , 30.0 , 30.0 , 30.0 , 30.0 , 30.0 },
    { -30.0 , 30.0 , 30.0 , 30.0 , 30.0 , -30.0 },
    { -30.0 , 30.0 , 30.0 , 30.0 , -30.0 , 30.0 },
    { -30.0 , 30.0 , 30.0 , 30.0 , -30.0 , -30.0 },
    { -30.0 , 30.0 , 30.0 , -30.0 , 30.0 , 30.0 },
    { -30.0 , 30.0 , 30.0 , -30.0 , 30.0 , -30.0 },
    { -30.0 , 30.0 , 30.0 , -30.0 , -30.0 , 30.0 },
    { -30.0 , 30.0 , 30.0 , -30.0 , -30.0 , -30.0 },
    { -30.0 , 30.0 , -30.0 , 30.0 , 30.0 , 30.0 },
    { -30.0 , 30.0 , -30.0 , 30.0 , 30.0 , -30.0 },
    { -30.0 , 30.0 , -30.0 , 30.0 , -30.0 , 30.0 },
    { -30.0 , 30.0 , -30.0 , 30.0 , -30.0 , -30.0 },
    { -30.0 , 30.0 , -30.0 , -30.0 , 30.0 , 30.0 },
    { -30.0 , 30.0 , -30.0 , -30.0 , 30.0 , -30.0 },
    { -30.0 , 30.0 , -30.0 , -30.0 , -30.0 , 30.0 },
    { -30.0 , 30.0 , -30.0 , -30.0 , -30.0 , -30.0 },
    { -30.0 , -30.0 , 30.0 , 30.0 , 30.0 , 30.0 },
    { -30.0 , -30.0 , 30.0 , 30.0 , 30.0 , -30.0 },
    { -30.0 , -30.0 , 30.0 , 30.0 , -30.0 , 30.0 },
    { -30.0 , -30.0 , 30.0 , 30.0 , -30.0 , -30.0 },
    { -30.0 , -30.0 , 30.0 , -30.0 , 30.0 , 30.0 },
    { -30.0 , -30.0 , 30.0 , -30.0 , 30.0 , -30.0 },
    { -30.0 , -30.0 , 30.0 , -30.0 , -30.0 , 30.0 },
    { -30.0 , -30.0 , 30.0 , -30.0 , -30.0 , -30.0 },
    { -30.0 , -30.0 , -30.0 , 30.0 , 30.0 , 30.0 },
    { -30.0 , -30.0 , -30.0 , 30.0 , 30.0 , -30.0 },
    { -30.0 , -30.0 , -30.0 , 30.0 , -30.0 , 30.0 },
    { -30.0 , -30.0 , -30.0 , 30.0 , -30.0 , -30.0 },
    { -30.0 , -30.0 , -30.0 , -30.0 , 30.0 , 30.0 },
    { -30.0 , -30.0 , -30.0 , -30.0 , 30.0 , -30.0 },
    { -30.0 , -30.0 , -30.0 , -30.0 , -30.0 , 30.0 },
    { -30.0 , -30.0 , -30.0 , -30.0 , -30.0 , -30.0 }};

    //
    // egm settings
    //
    RWSStateMachineInterface::EGMSettings egm_settings;

    egm_settings.activate.tool.robhold = true;
    egm_settings.activate.tool.tframe.pos.x = -67;
    egm_settings.activate.tool.tframe.pos.y = -9;
    egm_settings.activate.tool.tframe.pos.z = 191;
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
    ros::Duration(2).sleep();

    start_pos.robax.rax_1 = 0;
    start_pos.robax.rax_2 = 0;
    start_pos.robax.rax_3 = 0;
    start_pos.robax.rax_4 = 0;
    start_pos.robax.rax_5 = 0;
    start_pos.robax.rax_6 = 0;
    IRB_120.services().rapid().setMoveSpeed(TASK, speed_data);
    IRB_120.services().rapid().runMoveAbsJ(TASK, start_pos);
    ros::Duration(4).sleep();

    //
    // get robot pose at home
    //
    RobTarget home_pos;
    while (!IRB_120.getMechanicalUnitRobTarget(ROBOT, &home_pos))
        ros::Duration(COMM_DELAY).sleep();
    const float x_home = home_pos.pos.x.value;
    const float y_home = home_pos.pos.y.value;
    const float z_home = home_pos.pos.z.value;

    cout << "x_home = " << x_home << " y_home = " << y_home << " z_home = " << z_home << endl;

    float x, y, z, distance, max_distance = -9999;

    for (int i=0; i<NUM_OF_DESIRED_POS; i++)
    {
        //
        // move to the start target
        //
        IRB_120.services().rapid().setMoveSpeed(TASK, speed_data);
        IRB_120.services().rapid().runMoveAbsJ(TASK, start_pos);
        ros::Duration(4).sleep();
        //
        // move to the desired target
        //
        JointTarget pos;
        pos.robax.rax_1 = desired_positions[i][0];
        pos.robax.rax_2 = desired_positions[i][1];
        pos.robax.rax_3 = desired_positions[i][2];
        pos.robax.rax_4 = desired_positions[i][3];
        pos.robax.rax_5 = desired_positions[i][4];
        pos.robax.rax_6 = desired_positions[i][5];
        IRB_120.services().rapid().setMoveSpeed(TASK, speed_data);
        IRB_120.services().rapid().runMoveAbsJ(TASK, pos);
        ros::Duration(4).sleep();

        //
        // get current cartesian state
        //

        RobTarget cartesian_pos;
        while (!IRB_120.getMechanicalUnitRobTarget(ROBOT, &cartesian_pos))
            ros::Duration(COMM_DELAY).sleep();

        x = cartesian_pos.pos.x.value;
        y = cartesian_pos.pos.y.value;
        z = cartesian_pos.pos.z.value;
        cout << "x = " << x << " y = " << y << " z = " << z << endl;

        distance = sqrt(pow((x - x_home), 2) + pow((y - y_home), 2) + pow((z - z_home), 2));
        if(distance >= max_distance)
            max_distance = distance;
        cout << "distance = " << distance << endl;
    }
    cout << "min_distance = " << max_distance << endl;

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
