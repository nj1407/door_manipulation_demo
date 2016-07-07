#include <ros/ros.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>

//action for pushing
#include <door_manipulation_demo/PushDoorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include "door_manipulation_demo/door_perception.h"

#define NUM_JOINTS 8 //6+2 for the arm
//global variables for storing data

int main(int argc, char **argv) {
	//create the action client
	actionlib::SimpleActionClient<door_manipulation_demo::PushDoorAction> ac("door_open_as", true);
	ac.waitForServer();
    ROS_INFO("Waiting for action server to start.");

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    door_manipulation_demo::PushDoorGoal goal;
    goal.pushDoor = true;
    ac.sendGoal(goal);
	ROS_INFO("waiting for lift and verify action server result....");
	ac.waitForResult();
    //wait for the action to return


	ROS_INFO(" push door action	 finished.");

	if(ac.getResult()){
		ROS_INFO("door moved.");
	}else{
		ROS_WARN("door didn't move");
	}

}

