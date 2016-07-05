#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>

//including package services 
#include "door_manipulation_demo/door_perception.h"


#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <time.h>
#include <stdint.h>

#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8

Eigen::Vector4f centroid;

geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped first_goal;
geometry_msgs::PoseStamped second_goal;
geometry_msgs::PoseStamped arm_pose;
sensor_msgs::JointState joint_state_outofview;
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Quaternion plane_coeff;
geometry_msgs::Quaternion orig_plane_coeff;
bool heardPose = false;
bool heardJoinstState = false;
bool heardGoal = false;
bool g_caught_sigint = false;

ros::Publisher first_goal_pub;
ros::Publisher second_goal_pub;

/* what happens when ctr-c is pressed */

bool similar(float x1, float x2){
	if(x1 - .05 < x2 && x2 < x1 +.05){
		return true;
	}
	return false;
}	

bool didReachPose (){
	
	
}
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
	ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
	
	
	moveit_msgs::GetPositionIK::Request ikine_request;
	moveit_msgs::GetPositionIK::Response ikine_response;
	ikine_request.ik_request.group_name = "arm";
	ikine_request.ik_request.pose_stamped = p;
	
	/* Call the service */
	if(ikine_client.call(ikine_request, ikine_response)){
		ROS_INFO("IK service call success:");
		//ROS_INFO_STREAM(ikine_response);
	} else {
		ROS_INFO("IK service call FAILED. Exiting");
	}
	
	return ikine_response;
}

//get the recorded topics

void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
  //ROS_INFO_STREAM(current_state);
}


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

//listen for where the arm is 
void listenForArmData(float rate){
	heardPose = false;
	heardJoinstState = false;
	ros::Rate r(rate);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardPose && heardJoinstState)
			return;
		
		r.sleep();
	}
}

//get the first approach goal
void goal_cb (const geometry_msgs::PoseStampedConstPtr& input)
{
		ROS_INFO("entered goal_cb");
		first_goal.header = input->header;
		first_goal.pose = input->pose; 
		second_goal.header = input->header;
		second_goal.pose = input->pose;
		second_goal.pose.position.x += .1;
		heardGoal = true;
		
}
void toolpos_cb (const geometry_msgs::PoseStampedConstPtr& input)
{
		ROS_INFO("entered toolposecb");
		current_pose.header = input->header;
		current_pose.pose = input->pose;
		
}

void plane_coeff_cb (const geometry_msgs::QuaternionConstPtr& input){
	plane_coeff = *input;
}	

//get the second goal *may not be nesscecary
/*
void goal_cb_2 (const geometry_msgs::PoseStampedConstPtr& input)
{
		ROS_INFO("entered goal_cb2");
		second_goal.header = input->header;
		second_goal.pose = input->pose; 
}*/

int main (int argc, char** argv)
{
	
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_door_open_detector");
	ros::NodeHandle n;
	
	//tested to be you of way of xtion camera for starting pose
	start_pose.header.frame_id = "mico_link_base";
	start_pose.pose.position.x = 0.161036163568;
	start_pose.pose.position.y = -0.37887275219;
	start_pose.pose.position.z = 0.242402374744;
	start_pose.pose.orientation.x = 0.419507177788;
	start_pose.pose.orientation.y = 0.365710866911;
	start_pose.pose.orientation.z = 0.458010463645;
	start_pose.pose.orientation.w = 0.693177974837;
	
	first_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_picked", 1);
	second_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_picked_second", 1);
	
	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//subsrcibe to goals
	ros::Subscriber goal_sub = n.subscribe ("/goal_to_go", 1,goal_cb);

	ros::Subscriber plane_coeff_sub = n.subscribe ("/plane_coeff", 1,plane_coeff_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	//service
	
	ros::ServiceClient client = n.serviceClient<door_manipulation_demo::door_perception>("/door_handle_detection/door_perception");
	
	ros::ServiceClient client_move = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
	
	
	signal(SIGINT, sig_handler);
	orig_plane_coeff = plane_coeff; 
	
	//get arm position
	segbot_arm_manipulation::closeHand();
	//listenForArmData(30.0);
	//joint_state_outofview = current_state;
	
	//set goal for mico service
	door_manipulation_demo::door_perception door_srv;
	moveit_utils::MicoMoveitCartesianPose mico_srv;
	mico_srv.request.target = first_goal;
	

	//make calls to get vision
	if(client.call(door_srv)){
		ros::spinOnce();
		//if(client.call(door_srv)){
			ROS_INFO("entered");
			//ros::spinOnce();
		//}	
	} else {
		ROS_INFO("didn't enter vision");
	}		
	
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
	
	//if(heardGoal){
		//may not need used for robustness
		
		//checks to see if can reach both poses though inverse kinematics
		int changex1 = 0;
		int changey1 = 0;
		int changex2 = 0;
		int changey2 = 0;
		bool isReachable = false;
		while( changex1 < .2 && !isReachable){
			first_goal.pose.position.x += changex1;
			
			while( changey1 < .2 && !isReachable){
				first_goal.pose.position.y += changey1;
				moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,first_goal);
				
				if(ik_response_approach.error_code.val == 1){
					ROS_INFO("entered first pose passed");
					first_goal_pub.publish(first_goal);
					
					while( changex2 < .2 && !isReachable){
						second_goal.pose.position.x += changex2;
						
						while( changey2 < .2 && !isReachable){
							second_goal.pose.position.y += changey2;
							moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,first_goal);
							if(ik_response_approach.error_code.val == 1){
								first_goal_pub.publish(first_goal);
								second_goal_pub.publish(second_goal);
								ROS_INFO("entered second pose passed");
								isReachable = true;
							}	
							changey2 += .05;
						}		
						
					changex2 += .05;
					}
					
				}	
				
				changey1 += .05;
			}	
			changex1 += .05;
		}	

	    if(isReachable = true){	
			pressEnter();
			ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
			//segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
			first_goal_pub.publish(first_goal);
			
			//made vision calls check in rviz to see if correct then procede
			pressEnter();
			
			//ROS_INFO("Demo starting...Move the arm to a 'ready' position .");
			//segbot_arm_manipulation::homeArm(n);
			//segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
			
			ros::spinOnce();
			
			
			//pressEnter();
			//ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
			segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
			
			ros::spinOnce();                                            
			
			//ros::spinOnce();
			
			
			ROS_INFO("2nd goal picked...check if pose is what you want in rviz if not ctr c.");
			pressEnter();
			segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
			ros::spinOnce();                 
			segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
			ros::spinOnce();  
			pressEnter();
			ROS_INFO("Demo ending...arm will move back 'ready' position .");
			//segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
			segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
			segbot_arm_manipulation::homeArm(n);
			if(client.call(door_srv)){
			ros::spinOnce();
			ROS_INFO("entered");

			} else {
				ROS_INFO("didn't enter vision");
			}
			if(similar(orig_plane_coeff.x, plane_coeff.x) && similar(orig_plane_coeff.y, plane_coeff.y) && similar(orig_plane_coeff.z, plane_coeff.z)
				&& similar(orig_plane_coeff.w, plane_coeff.w)){
					ROS_INFO("didn't move door");
			} else {
					ROS_INFO("moved door");
			}	
			//refresh rate
			//double ros_rate = 3.0;
			//ros::Rate r(ros_rate);
			
	} else {
	
		ROS_INFO("Demo ending...didn't find an approac point .");
	}	
	
};
