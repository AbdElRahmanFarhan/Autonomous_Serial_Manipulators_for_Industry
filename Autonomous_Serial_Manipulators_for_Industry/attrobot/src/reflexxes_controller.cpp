#include "ros/ros.h"
#include "std_msgs/String.h"
#include "abb_libegm/egm_controller_interface.h"
#include "abb_libegm/egm_common_auxiliary.h"
#include "abb_libegm/egm_trajectory_interface.h"
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

//Don't forgrt to renew protobuff files when needed :D.
//# In the directory you installed Caffe to
//protoc src/caffe/proto/caffe.proto --cpp_out=.
//mkdir include/caffe/proto
//mv src/caffe/proto/caffe.pb.h include/caffe/proto

//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.02
#define NUMBER_OF_DOFS                          3

void Output_Data(abb::egm::wrapper::Output* Output, int index, int angle, int vel);

int main(int argc, char *argv[])
{
    double Input_data[NUMBER_OF_DOFS],Vel_data[NUMBER_OF_DOFS];
    ros::init(argc, argv, "Node");
    ros::NodeHandle n;
    ROS_INFO("1");

    int                         ResultValue                 =   0       ;
    ReflexxesAPI                *RML                        =   NULL    ;
    RMLPositionInputParameters  *IP                         =   NULL    ;
    RMLPositionOutputParameters *OP                         =   NULL    ;
    RMLPositionFlags            Flags                                   ;

    boost::asio::io_service io_service;
    boost::asio::io_service::work work(io_service);
    abb::egm::BaseConfiguration config;
    abb::egm::EGMControllerInterface Server(io_service,6510,config);
    abb::egm::wrapper::Input Input;
    abb::egm::wrapper::Output Output;

    RML =   new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  =   new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP  =   new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    IP->CurrentPositionVector->VecData      [0] =      0.0      ;
    IP->CurrentPositionVector->VecData      [1] =      0.0      ;
    IP->CurrentPositionVector->VecData      [2] =      0.0      ;

    IP->CurrentVelocityVector->VecData      [0] =      0.0      ;
    IP->CurrentVelocityVector->VecData      [1] =      0.0      ;
    IP->CurrentVelocityVector->VecData      [2] =      0.0      ;

    IP->MaxVelocityVector->VecData          [0] =    400.0      ;
    IP->MaxVelocityVector->VecData          [1] =    400.0      ;
    IP->MaxVelocityVector->VecData          [2] =    400.0      ;

    IP->MaxAccelerationVector->VecData      [0] =     80.0      ;
    IP->MaxAccelerationVector->VecData      [1] =     80.0      ;
    IP->MaxAccelerationVector->VecData      [2] =     80.0      ;

    IP->TargetPositionVector->VecData       [0] =     90.0      ;
    IP->TargetPositionVector->VecData       [1] =     80.0      ;
    IP->TargetPositionVector->VecData       [2] =     -80.0      ;

    IP->TargetVelocityVector->VecData       [0] =     0.0       ;
    IP->TargetVelocityVector->VecData       [1] =     0.0       ;
    IP->TargetVelocityVector->VecData       [2] =     0.0       ;

    IP->SelectionVector->VecData            [0] =   true        ;
    IP->SelectionVector->VecData            [1] =   true        ;
    IP->SelectionVector->VecData            [2] =   true        ;



    for(int i=0;i<NUMBER_OF_DOFS;i++){
        Output.mutable_robot()->mutable_joints()->mutable_position()->add_values(i);
        Output.mutable_robot()->mutable_joints()->mutable_velocity()->add_values(i);
    }


    //Output_Data(&Output,0,140,20);

    boost::thread thread([&io_service]() { io_service.run(); });
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED){
        try
        {
            ResultValue =   RML->RMLPosition(*IP, OP, Flags);

            if (ResultValue < 0)
            {
                ROS_INFO("An error occurred (%d).\n", ResultValue );
                break;
            }

            Server.waitForMessage();
            ROS_INFO("Waited");
            Server.read(&Input);

            for(int i=0;i<NUMBER_OF_DOFS;i++){

                Output_Data(&Output,i,OP->NewPositionVector->VecData[i],OP->NewVelocityVector->VecData[i]);
                ROS_INFO("Value = %f",OP->NewPositionVector->VecData[i] );
                //usleep(20000);
            }

            Server.write(Output);
            ROS_INFO("Wrote the output");
            Vel_data[0] = Input.feedback().robot().joints().velocity().values().Get(0);
            ROS_INFO(" JointVel = %f ", Vel_data[0]);

            *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
            *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
            *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
        }
        catch(std::exception e)
        {
            ROS_INFO("Error");
        }
    }
    //Server.waitForMessage();
    //Server.read(&Input);
    //int i=5;
    while(true){
        //Server.waitForMessage();
        //Server.read(&Input);
        //Output_Data(&Output,0,i,0.1);
        //Server.write(Output);

        Vel_data[0] = Input.feedback().robot().joints().velocity().values().Get(0);
        ROS_INFO(" JointVel = %f ", Vel_data[0]);
        //usleep(10000);
        //i+=1;

    }
    thread.join();
    io_service.stop();
	return 0;
}
void Output_Data(abb::egm::wrapper::Output* Output, int index, int angle, int vel)
{
    Output->mutable_robot()->mutable_joints()->mutable_position()->set_values(index, angle);
    Output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(index, vel);
}

