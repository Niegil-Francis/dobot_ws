#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
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
#include <cstdlib>


#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_P 0x70
#define KEYCODE_G 0x67
#define KEYCODE_T 0x74

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop(ros::NodeHandle &n)
{
    static float pose[100][4];
    unsigned char c;
    int counter=0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    // SetCmdTimeout
    ros::ServiceClient client;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if(num > 0) {
            if(read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            continue;
        }
	
	client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    	dobot::SetPTPCmd srv;

        srv.request.ptpMode = 1;


	ros::ServiceClient client_curr;
	client_curr = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        dobot::GetPose srv_curr;

        switch(c) {
            case KEYCODE_W:
                ROS_INFO("W");
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x)+1;
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_S:
                ROS_INFO("S");
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x)-1;
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_A:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y)-1;
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_D:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y)+1;
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_U:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z)+1;
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_I:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z)-1;
            	srv.request.r = int(srv_curr.response.r);

            break;
            case KEYCODE_J:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r)+1;

            break;
            case KEYCODE_K:
		client_curr.call(srv_curr);
            	srv.request.x = int(srv_curr.response.x);
            	srv.request.y = int(srv_curr.response.y);
            	srv.request.z = int(srv_curr.response.z);
            	srv.request.r = int(srv_curr.response.r)-1;

            break;
	    case KEYCODE_G:
                ROS_INFO("G");
		client_curr.call(srv_curr);
		ROS_INFO("\nThe %d position is set \n",counter);
		pose[counter][0]=srv_curr.response.x;
		pose[counter][1]=srv_curr.response.y;
		pose[counter][2]=srv_curr.response.z;
		pose[counter][3]=srv_curr.response.r;

		ROS_INFO("\nx: %lf",pose[counter][0]);
		ROS_INFO("y: %lf",pose[counter][1]);
		ROS_INFO("z: %lf",pose[counter][2]);
		ROS_INFO("r: %lf \n",pose[counter][3]);
		counter+=1;
            break;
	    case KEYCODE_T:
                ROS_INFO("T");
		counter-=1;
		ROS_INFO("\nRemoved %d \n",counter);
            break;
	    case KEYCODE_P:
                ROS_INFO("P");
		ROS_INFO("\n The positions added are: \n");
		std::cout<<"[";
		for(int j=0;j<counter;j++)
		{

		std::cout<<pose[j][0]<<","<<pose[j][1]<<","<<pose[j][2]<<";"<<"\n";
		}
		std::cout<<"]\n";
            break;
            default:
                ROS_INFO("DEFAULT:0x%02x", c);
            break;
        }

        do {
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);

}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    // SetCmdTimeout
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

  

    boost::thread t = boost::thread(boost::bind(&keyboardLoop, boost::ref(n)));
    ros::spin();
    t.interrupt();
    t.join();

    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}

