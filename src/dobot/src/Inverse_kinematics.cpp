
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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
#include<unistd.h>
using namespace std;
using namespace cv;

int counter;
int repeat;
int pose[50][4];
int pos1;
int pos2;

/*
int getch()
	{
	  static struct termios oldt, newt;
	  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON);                 // disable buffering      
	  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	  int c = getchar();  // read character (non-blocking)

	  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	char c;
	cin>>c;
	  return c;
	}


int getint()
	{
	  //static struct termios oldt, newt;
	  //tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	  //newt = oldt;
	  //newt.c_lflag &= ~(ICANON);                 // disable buffering      
	  //tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	  //int c = getchar();  // read character (non-blocking)
	  //int i = c - '0';
	  //cin.clear();
	  int i;
	  cin>>i;
	  //tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	  return i;
	}

*/
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

	client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        dobot::GetPose srv;
	client.call(srv);
	double cur_x=srv.response.x;	
	double cur_y=srv.response.y;
	double cur_z=srv.response.z;
	double cur_r=srv.response.r;
	
 	double J1=srv.response.jointAngle[0];
	double J2=srv.response.jointAngle[1];
	double J3=srv.response.jointAngle[2];
	double J4=srv.response.jointAngle[3];


	ROS_INFO("Current Values:\n");

	ROS_INFO("x: %f",cur_x);
	ROS_INFO("y: %f",cur_y);
	ROS_INFO("z: %f",cur_z);
	ROS_INFO("r: %f \n",cur_r);


	ROS_INFO("J1: %f",J1);
	ROS_INFO("J2: %f",J2);
	ROS_INFO("J3: %f",J3);
	ROS_INFO("J4: %f \n",J4);
    
	// Getting the positon
	ROS_INFO("Enter x value to go to:");	
	double x_new;
	cin>>x_new;
	cout<<"\n";
	ROS_INFO("Enter y value to go to:");
	double y_new;
	cin>>y_new;
	cout<<"\n";
	ROS_INFO("Enter z value to go to:");
	double z_new;
	cin>>z_new;
	cout<<"\n";
	

	int flag=0;

	while(true)
	{
	flag=0;

	client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        dobot::GetPose srv;
	client.call(srv);
	double cur_x=srv.response.x;	
	double cur_y=srv.response.y;
	double cur_z=srv.response.z;
	double cur_r=srv.response.r;
	
 	double J1=srv.response.jointAngle[0];
	double J2=srv.response.jointAngle[1];
	double J3=srv.response.jointAngle[2];
	double J4=srv.response.jointAngle[3];


	ROS_INFO("Current Values:\n");

	ROS_INFO("x: %f",cur_x);
	ROS_INFO("y: %f",cur_y);
	ROS_INFO("z: %f",cur_z);
	ROS_INFO("r: %f \n",cur_r);


	ROS_INFO("J1: %f",J1);
	ROS_INFO("J2: %f",J2);
	ROS_INFO("J3: %f",J3);
	ROS_INFO("J4: %f \n",J4);




	Mat J=cv::Mat::zeros(3,3, CV_64F);
	double pi = 2*acos(0.0);

	J.at<double>(0,0)=(135*sin(J2*pi/180)+147*sin((J2+J3)*pi/180))*sin(J1*pi/180);
	J.at<double>(0,1)=147*sin((J2+J3)*pi/180)*sin(J1*pi/180);
	J.at<double>(0,2)=-(135*cos(J2*pi/180)+147*cos((J2+J3)*pi/180))*cos(J1*pi/180);

	J.at<double>(1,0)=-(135*sin(J2*pi/180)+147*sin((J2+J3)*pi/180))*cos(J1*pi/180);
	J.at<double>(1,1)=-147*sin((J2+J3)*pi/180)*cos(J1*pi/180);
	J.at<double>(1,2)=-(135*cos(J2*pi/180)+147*cos((J2+J3)*pi/180))*sin(J1*pi/180);

	J.at<double>(2,0)=135*cos(J2*pi/180)+147*cos((J2+J3)*pi/180);
	J.at<double>(2,1)=147*cos((J2+J3)*pi/180);
	J.at<double>(2,2)=0.0;

	Mat J_inv=cv::Mat::zeros(3,3, CV_64F);
	cv::invert(J,J_inv);
	
	Mat delta_x=cv::Mat::zeros(3,1, CV_64F);
	delta_x.at<double>(0,0)=x_new-cur_x;
	delta_x.at<double>(1,0)=y_new-cur_y;
	delta_x.at<double>(2,0)=z_new-cur_z;

	Mat del_q=J_inv*delta_x;
	

	double J1_del=del_q.at<double>(0,0);
	double J2_del=del_q.at<double>(1,0);
	double J3_del=del_q.at<double>(2,0);

	J1_del=J1_del*180/pi;
	J2_del=J2_del*180/pi;
	J3_del=J3_del*180/pi;

	ROS_INFO("del of J1 %f",J1_del);
	ROS_INFO("del of J2: %f",J2_del);
	ROS_INFO("del of J3: %f",J3_del);
	


	double new_J1=J1+J1_del;
	double new_J2=J2+J2_del;
	double new_J3=J3+J3_del;
    

	


	

	
	client = n.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
        dobot::SetJOGCmd srv1;
        srv1.request.isJoint = true;

	srv1.request.cmd=0;
	if(new_J1<=90 && new_J1>=-90 && abs(new_J1-J1)>1)
	{
		if(J1_del>0)
		{
			srv1.request.cmd=1;
		}
		if(J1_del<0)
		{
			srv1.request.cmd=2;
		}			
	}
	else if(new_J1>90 || new_J1<-90)
	{
		break;
		flag=1;
	}

        if (client.call(srv1)) {
	        ROS_INFO("Result:%d", srv.response.result);
        } else {
            ROS_ERROR("Failed to call SetJOGCmd");
        }

	srv1.request.cmd=0;
	if(new_J2<=85 && new_J2>=0 && abs(new_J2-J2)>1)
	{
		if(J2_del>0)
		{
			srv1.request.cmd=3;
		}
		if(J2_del<0)
		{
			srv1.request.cmd=4;
		}			
	}
	else if(new_J2>85|| new_J1<0)
	{
		break;
		flag=1;
	}

        if (client.call(srv1)) {
	        ROS_INFO("Result:%d", srv.response.result);
        } else {
            ROS_ERROR("Failed to call SetJOGCmd");
        }


	srv1.request.cmd=0;

	if(new_J3<=95 && new_J3>=-10 && abs(new_J3-J3)>1)
	{
		if(J3_del>0)
		{
			srv1.request.cmd=5;
		}
		if(J3_del<0)
		{
			srv1.request.cmd=6;
		}			
	}
	else if(new_J3>95|| new_J3<-10)
	{
		break;
		flag=1;
	}

        if (client.call(srv1)) {
	        ROS_INFO("Result:%d", srv.response.result);
        } else {
            ROS_ERROR("Failed to call SetJOGCmd");
        }

	srv1.request.cmd=0;
	if (client.call(srv1)) {
	        ROS_INFO("Result:%d", srv.response.result);
        } else {
            ROS_ERROR("Failed to call SetJOGCmd");
        }

	if(abs(new_J1-J1)<=1 && abs(new_J2-J2)<=1  && abs(new_J3-J3)<=1)
	{
		ROS_INFO("Position Reached!!!!");
		break;
	}
	}


	
	if(flag==1)
	{
		ROS_INFO("Position was not reached due to joint limits \n");
	}
	
	 
	ROS_INFO("\nEnter q to quit or any other key to continue \n");
    	double repeat;
	cin>>repeat;	
	if(repeat== 'q')
	{
	 break;
	}
	}
    return 0;

    }




