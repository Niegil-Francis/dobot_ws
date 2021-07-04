#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
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
#include <librealsense2/rsutil.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include <vector>
#include <stdlib.h>
#include "dobot/GetPose.h"
#include "dobot/SetEndEffectorGripper.h"

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
#include <geometry_msgs/Twist.h>

void poseCallback(const geometry_msgs::Pose& msg)
{
  ros::NodeHandle m;
  ros::ServiceClient client_sub;
  dobot::SetPTPCmd srv_s;

  client_sub = m.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
  ROS_INFO("I got a pose");

  srv_s.request.ptpMode = 2;
  srv_s.request.x = msg.position.x;
  srv_s.request.y = msg.position.y;
  srv_s.request.z = msg.position.z;
  srv_s.request.r = 10;
  client_sub.call(srv_s);   
}
/*
int Arr[5];
void poseCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  ros::NodeHandle m;
  ros::ServiceClient client_sub;
  ros::ServiceClient client_grip;
  ros::ServiceClient client_wait;
  dobot::SetPTPCmd srv_s;

  client_sub = m.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
  ROS_INFO("I got a pose");

  srv_s.request.ptpMode = 2;
  int i=0;
  for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
  {
		Arr[i] = *it;
		i++;
  }
  srv_s.request.x = Arr[0];
  srv_s.request.y = Arr[1];
  srv_s.request.z = Arr[2];
  srv_s.request.r = Arr[3];
  if(Arr[4]!= -1)
    {
    client_grip = m.serviceClient<dobot::SetEndEffectorGripper>("/DobotServer/SetEndEffectorGripper");
    dobot::SetEndEffectorGripper srv_g;

    srv_g.request.enableCtrl=1;
    srv_g.request.grip = Arr[4];
    srv_g.request.isQueued = 1;
    client_grip.call(srv_g);
    if (srv_g.response.result == 0) 
    {	
	client_wait = m.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
    	dobot::SetWAITCmd srv_w;
	srv_w.request.timeout=500;
	client_wait.call(srv_w);
    } 
    }
   else if(Arr[4]== -1)
    {
    client_grip = m.serviceClient<dobot::SetEndEffectorGripper>("/DobotServer/SetEndEffectorGripper");
    dobot::SetEndEffectorGripper srv_g;
    srv_g.request.enableCtrl=0;
    srv_g.request.isQueued = 1;
    client_grip.call(srv_g);
    } 

  ROS_INFO("x %d\n",Arr[0]);
  ROS_INFO("y %d\n",Arr[1]);
  ROS_INFO("z %d\n",Arr[2]);
  ROS_INFO("r %d\n",Arr[3]);
  ROS_INFO("gripper %d\n",Arr[4]);

  
  client_sub.call(srv_s);   
}
*/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient_Topic");
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

        srv.request.xyzVelocity = 200;
        srv.request.xyzAcceleration = 200;
        srv.request.rVelocity = 200;
        srv.request.rAcceleration = 200;
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

    //client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    //dobot::SetPTPCmd srv;
    
    ros::Subscriber geom = n.subscribe("geometry_pose", 1000, poseCallback);
    ros::spin();

}

