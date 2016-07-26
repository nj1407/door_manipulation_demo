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
#include <segbot_arm_manipulation/grasp_utils.h>
#include <agile_grasp/Grasps.h>
#include <door_manipulation_demo/PushDoorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TwistStamped.h>

#define NUM_JOINTS 8
#define HAND_OFFSET_GRASP -0.02
#define HAND_OFFSET_APPROACH -0.13
#define ANGULAR_DIFF_THRESHOLD 20.0

Eigen::Vector4f centroid;

geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped first_goal;
geometry_msgs::PoseStamped second_goal;

sensor_msgs::JointState joint_state_outofview;
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Quaternion plane_coeff;
geometry_msgs::Quaternion orig_plane_coeff;

agile_grasp::Grasps current_grasps;
bool heardPose = false;
bool heardJoinstState = false;
bool heardGoal = false;
bool g_caught_sigint = false;

ros::Publisher first_goal_pub;
ros::Publisher second_goal_pub;
ros::Publisher pub_velocity;

//declare subscribers

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
		first_goal.pose.position.x -= .05;
		second_goal.header = input->header;
		second_goal.pose = input->pose;
		second_goal.pose.position.x += .20;
	//	first_goal_pub.publish(first_goal);
	//	second_goal_pub.publish(second_goal);
		heardGoal = true;
		
}
void toolpos_cb (const geometry_msgs::PoseStampedConstPtr& input)
{
		//ROS_INFO("entered toolposecb");
		current_pose.header = input->header;
		current_pose.pose = input->pose;
		
}

void plane_coeff_cb (const geometry_msgs::QuaternionConstPtr& input){
	plane_coeff = *input;
}	

class PushDoorActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<door_manipulation_demo::PushDoorAction> as_; 
  
  std::string action_name_;
  
  door_manipulation_demo::PushDoorFeedback feedback_;
  door_manipulation_demo::PushDoorResult result_;
  ros::Subscriber sub_angles;
  ros::Subscriber goal_sub;
  ros::Subscriber plane_coeff_sub;
  ros::Subscriber sub_tool;
  ros::ServiceClient client ;
  ros::ServiceClient client_move;
  
 public:

	PushDoorActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PushDoorActionServer::executeCB, this, _1), false),
    action_name_(name)
  { 
	
		//create subscriber to joint angles
		sub_angles = nh_.subscribe ("/joint_states", 1, joint_state_cb);
		
		//subsrcibe to goals
		goal_sub = nh_.subscribe ("/goal_to_go", 1,goal_cb);

		plane_coeff_sub = nh_.subscribe ("/plane_coeff", 1,plane_coeff_cb);
		
		//create subscriber to tool position topic
		sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
		
		//publish velocities
		pub_velocity = nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
		
		//service
		
		client = nh_.serviceClient<door_manipulation_demo::door_perception>("/door_handle_detection/door_perception");
		
		client_move = nh_.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
		
		ROS_INFO("Starting push door action server...");
		as_.start();
 }
 
   ~PushDoorActionServer(void)
  {
  }
  
  //check if values are similar to each other within 10 cm
  bool similar(float x1, float x2){
	if(x1 - .05 < x2 && x2 < x1 +.05){
		return true;
	}
	return false; 
  }	

 

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

	void executeCB(const door_manipulation_demo::PushDoorGoalConstPtr  &goal){
		//tested to be you of way of xtion camera for starting pose
	start_pose.header.frame_id = "mico_link_base";
    start_pose.pose.position.x = 0.14826361835;
    start_pose.pose.position.y = -0.323001801968;
    start_pose.pose.position.z = 0.233884751797;
    start_pose.pose.orientation.x = 0.49040481699;
    start_pose.pose.orientation.y = 0.468191160046;
    start_pose.pose.orientation.z = 0.461722946003;
    start_pose.pose.orientation.w = 0.571937124396;
	
	first_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("goal_picked", 1);
	second_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("goal_to_go_2", 1);
	
	signal(SIGINT, sig_handler);
	
	
	//get arm position
	segbot_arm_manipulation::closeHand();
	//listenForArmData(30.0);
	//joint_state_outofview = current_state;
	
	//set goal for mico service
	door_manipulation_demo::door_perception door_srv;
	moveit_utils::MicoMoveitCartesianPose mico_srv;
	mico_srv.request.target = first_goal;
	
	segbot_arm_manipulation::homeArm(nh_);
	ros::spinOnce();
	segbot_arm_manipulation::moveToPoseMoveIt(nh_,start_pose);
	ros::spinOnce();
	segbot_arm_manipulation::moveToPoseMoveIt(nh_,start_pose);
	ros::spinOnce();
	segbot_arm_manipulation::moveToPoseMoveIt(nh_,start_pose);
	ros::spinOnce();
	//make calls to get vision
	if(client.call(door_srv)){
		ros::spinOnce();
		//if(client.call(door_srv)){
			ROS_INFO("entered");
			//ros::spinOnce();
		//}	
	} else {
		ROS_INFO("didn't enter vision");
		result_.success = false;
		as_.setSucceeded(result_);
	}
	orig_plane_coeff = plane_coeff; 
	//make an array of poses for first goal		
	//segbot_arm_manipulation::homeArm(nh_);
	//segbot_arm_manipulation::homeArm(nh_);		
	double timeoutSeconds = 3.85;
	int rateHertz = 100;
	geometry_msgs::TwistStamped velocityMsg;
							
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
								
		velocityMsg.twist.linear.x = 1.25;
		velocityMsg.twist.linear.y = 0.0;
		velocityMsg.twist.linear.z = 0.0;
								
		velocityMsg.twist.angular.x = 0.0;
		velocityMsg.twist.angular.y = 0.0;
		velocityMsg.twist.angular.z = 0.0;
								
								
		pub_velocity.publish(velocityMsg);
								
		r.sleep();
	}					
		
		segbot_arm_manipulation::homeArm(nh_);
		if(client.call(door_srv)){
			ros::spinOnce();

		} else {
			ROS_INFO("didn't enter vision");
		}
		//compare plane coefficents to see if door has moved
		if(similar(orig_plane_coeff.x, plane_coeff.x) && similar(orig_plane_coeff.y, plane_coeff.y) && similar(orig_plane_coeff.z, plane_coeff.z)
		&& similar(orig_plane_coeff.w, plane_coeff.w)){
			ROS_INFO("didn't move  door");
			result_.success = false;
			as_.setSucceeded(result_);
		} else {
			ROS_INFO("moved door");
			result_.success = true;
			as_.setSucceeded(result_);
		}	
					
	}		
		
};
int main (int argc, char** argv)
{
	
  ros::init(argc, argv, "door_open_as");

  PushDoorActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
};
