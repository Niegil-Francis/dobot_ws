#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/GetPose.h"
#include "dobot/SetEndEffectorSuctionCup.h"

#include "dobot/SetWAITCmd.h"
#include "dobot/SetJOGCmd.h"
#include <cstdlib>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <iostream.h>
int counter;
int repeat;
double pose[50][4];
int pos1;
int pos2;


int getch()
	{
	  static struct termios oldt, newt;
	  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON);                 // disable buffering      
	  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	  int c = getchar();  // read character (non-blocking)

	  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	  return c;
	}


int getint()
	{
	  static struct termios oldt, newt;
	  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON);                 // disable buffering      
	  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	  int c = getchar();  // read character (non-blocking)
	  int i = c - '0';

	  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	  return i;
	}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    ros::ServiceClient client;

    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);


    counter=0;	

    for(;;)
    {
    
    for(;;)
    {

	// Move the dobot to a starting position
	// Getting the positon
	ROS_INFO("\nEnter Y when number %d position is set \n",counter);
	int c = getch();
	if(c=='y' || c=='Y')
	{

        client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        dobot::GetPose srv;
	client.call(srv);
	ROS_INFO("\nThe %d position is set \n",counter);
	pose[counter][0]=srv.response.x;
	pose[counter][1]=srv.response.y;
	pose[counter][2]=srv.response.z;
	pose[counter][3]=srv.response.r;

	ROS_INFO("\nx: %lf",pose[counter][0]);
	ROS_INFO("y: %lf",pose[counter][1]);
	ROS_INFO("z: %lf",pose[counter][2]);
	ROS_INFO("r: %lf \n",pose[counter][3]);
	break;
	}
	else
	{
	continue;
	}
    }
	ROS_INFO("\nEnter Y to add another point else press any other key \n");
	int option = getch();
        if(option=='y' || option=='Y')
	{
		counter+=1;
	}
	else
	{
		if(counter==0)
		{
		ROS_INFO("\n Min two points need to be added \n");
		counter+=1;
		}
		else
		{
		break;
		}	
        }
    }
    
    ROS_INFO("\n The positions added are: \n");
    std::cout<<"[";
    for(int j=0;j<=counter;j++)
	{
        //ROS_INFO("\n Position %d\n",counter);
    	std::cout<<pose[j][0]<<","<<pose[j][1]<<","<<pose[j][2]<<";"<<"\n";
	/*
	ROS_INFO("y: %d",pose[counter][1]);
	ROS_INFO("z: %d",pose[counter][2]);
	ROS_INFO("r: %d \n",pose[counter][3]);
	*/
	}
	std::cout<<"]\n";

    


    /*
    ROS_INFO("\nEnter the number of times to repeat the process \n");
    repeat = getint();

    ROS_INFO("\nEnter the position to pick the object \n");
    pos1 = getint();
    ROS_INFO("\nPick at: %d \n",pos1);
    ROS_INFO("\nEnter the position to release the object \n");
    pos2 = getint();
    ROS_INFO("\nDrop at: %d \n",pos2);


    while (ros::ok()) {

	for(int j=0;j<=counter;j++)
	{
        do {
	    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
            dobot::SetPTPCmd srv;
            srv.request.ptpMode = 1;
            srv.request.x = pose[j][0];
            srv.request.y = pose[j][1];
            srv.request.z = pose[j][2];
            srv.request.r = pose[j][3];
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }

            if (ros::ok() == false) {

                break;
            }
        } while (1);

	do {
	if(j== pos1)
	    {    
            ros::spinOnce();
	    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
            dobot::SetEndEffectorSuctionCup srv;

            srv.request.enableCtrl=1;
            srv.request.suck = 1;
            srv.request.isQueued = 1;
            client.call(srv);
            if (srv.response.result == 0) 
		{	
		client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
            	dobot::SetWAITCmd srv;
		srv.request.timeout=1000;
		client.call(srv);
                break;
            } 
            }

	    else if(j == pos2)
	    {
            ros::spinOnce();
	    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
            dobot::SetEndEffectorSuctionCup srv;

            srv.request.enableCtrl=1;
            srv.request.suck = 0;
            srv.request.isQueued = 1;
            client.call(srv);
	    if (srv.response.result == 0) 
            {
                break;
            } 
            }
	    else
	    {
	    break;
	    }
	    if (ros::ok() == false) 
            {	
                break;
            }
	
        } while (1);

	}

        if(repeat>1)
	{
	ros::spinOnce();
	repeat-=1;
	}
	else
	{
	break;
	}	
    }
    */

    return 0;
}

