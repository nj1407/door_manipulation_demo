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
#include "agile_grasp/Grasps.h"

#define NUM_JOINTS 8
#define HAND_OFFSET_GRASP -0.02
#define HAND_OFFSET_APPROACH -0.13
#define ANGULAR_DIFF_THRESHOLD 12.0

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

/* what happens when ctr-c is pressed */
/*
public:

  TabletopGraspActionServer(std::string name) :
	as_(nh_, name, boost::bind(&TabletopGraspActionServer::executeCB, this, _1), false),
    action_name_(name)
    as_.start();
  }

  ~TabletopGraspActionServer(void)
  {
  }
*/


bool similar(float x1, float x2){
	if(x1 - .05 < x2 && x2 < x1 +.05){
		return true;
	}
	return false;
}	

/*
void grasps_cb(const agile_grasp::Grasps &msg){
		ROS_INFO("Heard grasps!");
		current_grasps = msg;
		heardGrasps = true;
}*/

/*struct GraspCartesianCommand {
	sensor_msgs::JointState approach_q;
	geometry_msgs::PoseStamped approach_pose;
	
	sensor_msgs::JointState grasp_q;
	geometry_msgs::PoseStamped grasp_pose;
	
	
};*/

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
		first_goal_pub.publish(first_goal);
		second_goal_pub.publish(second_goal);
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
	tf::TransformListener listener;
	//tested to be you of way of xtion camera for starting pose
	start_pose.header.frame_id = "mico_link_base";
    start_pose.pose.position.x = 0.2531;
    start_pose.pose.position.y = -0.24;
    start_pose.pose.position.z = 0.31;
    start_pose.pose.orientation.x = 0.6048;
    start_pose.pose.orientation.y = 0.466;
    start_pose.pose.orientation.z = 0.424;
    start_pose.pose.orientation.w = 0.485;
	
	first_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_picked", 1);
	second_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_to_go_2", 1);
	
	//subscriber for grasps
	//ros::Subscriber sub_grasps = n.subscribe("/find_grasps/grasps_handles",1, &TabletopGraspActionServer::grasps_cb,this);  
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
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
	//make an array of poses for first goal		
	geometry_msgs::PoseArray poses_msg_first;
	poses_msg_first.header.seq = 1;
	poses_msg_first.header.stamp = first_goal.header.stamp;
	poses_msg_first.header.frame_id = "mico_api_origin";
	int changex1 = 0;
	int changey1 = 0;
	poses_msg_first.poses.push_back(first_goal.pose);
	//potential_approach = first_goal.pose;
	ROS_INFO("passed .5");
	for(int changex = 0; changex < 10; changex++){
		int occurances = 0;
		for(int changey = 0; changey < 10; changey++){
			geometry_msgs::Pose potential_approach;
			potential_approach = first_goal.pose;
			ROS_INFO("passed .5");
			//potential_approach.position.z += .05;
			potential_approach.position.y += .05;
			poses_msg_first.poses.push_back(potential_approach);
			//changey1 += .05;
			occurances++;
			ROS_INFO("occured %d", occurances);
		}	
		//changex1 += .05;
	}
			ROS_INFO("passed 1");
	//make array of poses for the second goal
	geometry_msgs::PoseArray poses_msg_2nd;
	poses_msg_first.header.seq = 1;
	poses_msg_first.header.stamp = second_goal.header.stamp;
	poses_msg_first.header.frame_id = "mico_api_origin";
	poses_msg_first.poses.push_back(second_goal.pose);
	int changex = 0;
	int changey = 0;
	while(changex < 10){
		while( changey < 10){
			geometry_msgs::Pose push_point;
		push_point = second_goal.pose;
			//push_point.position.z += .05;
			push_point.position.y += .05;
			poses_msg_first.poses.push_back(push_point);
			changey ++;
		}	
		changex ++;
	}
			ROS_INFO("passed 2");
	//bool isReachable = false;
	//pick points to publish at
	/*
	std::vector<geometry_msgs::PoseStamped> pose1;
	std::vector<geometry_msgs::PoseStamped> pose2;
	while( !isReachable & i < sizeof(poses_msg_first.poses)){
		geometry_msgs::PoseStamped temp;
		temp.header = first_goal.header;
		temp.pose = poses_msg_first.poses.at(i);
			moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,temp);
			if(ik_response_approach.error_code.val == 1){
				moveit_msgs::GetPositionIK::Response  ik_response_grasp = segbot_arm_manipulation::computeIK(n_,gc_i.grasp_pose);
				if(ik_response_approach.error_code.val == 1){
					ROS_INFO("entered first pose passed");
					first_goal_pub.publish(poses_msg_first.poses.at(i));
					first_goal.pose = poses_msg_first.poses.at(i);
					pose1.push_back(first_goal);
					int j = 0;
					while( !isReachable & i < sizeof(poses_msg_2nd.poses)){
						geometry_msgs::PoseStamped temp2;
						temp2.header = first_goal.header;
						temp2.pose = poses_msg_2nd.poses.at(j);
						moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,temp2);
						if(ik_response_approach.error_code.val == 1){
							
							//std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state);
							//double sum_d = 0;
							//for (int p = 0; p < D.size(); p++){
								//sum_d += D[p];
							//}
							
							ROS_INFO("entered second pose passed");
							second_goal_pub.publish(poses_msg_2nd.poses.at(j));
							second_goal.pose = poses_msg_2nd.poses.at(j);
							pose2.push_back(second_goal);
							isReachable = true;
						}	
						j++;
					}
				}	
			}	
			i++;
	}	
	*/
	//here, we'll store all grasp options that pass the filters
			std::vector<geometry_msgs::PoseStamped> push_commands;
			
			for (unsigned int i = 0; i < poses_msg_first.poses.size(); i++){
				
				geometry_msgs::PoseStamped temp_first_goal;	
				temp_first_goal.header = poses_msg_first.header;
				temp_first_goal.pose = poses_msg_first.poses.at(i);

					//filter two -- if IK fails
					moveit_msgs::GetPositionIK::Response  ik_response_approach = segbot_arm_manipulation::computeIK(n,temp_first_goal);
					
					if (ik_response_approach.error_code.val == 1){
							
							//now check to see how close the two sets of joint angles are -- if the joint configurations for the approach and grasp poses differ by too much, the grasp will not be accepted
							std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(current_state, ik_response_approach.solution.joint_state);
							
							double sum_d = 0;
							for (int p = 0; p < D.size(); p++){
								sum_d += D[p];
							}
						
							
							if (sum_d < ANGULAR_DIFF_THRESHOLD){
								//ROS_INFO("Angle diffs for grasp %i: %f, %f, %f, %f, %f, %f",(int)grasp_commands.size(),D[0],D[1],D[2],D[3],D[4],D[5]);
								
								ROS_INFO("Sum diff: %f",sum_d);
								ROS_INFO("added to push commands size"); //%d", push_commands.size());
								//store the IK results
								
								push_commands.push_back(temp_first_goal);
							}
						
					}
				
			}
					ROS_INFO("passed 3");
			//check to see if all potential grasps have been filtered out
			if (push_commands.size() == 0){
				ROS_WARN("[segbot_tabletop_grasp_as.cpp] No feasible grasps found demo done.");
		
			} else{
					
					//listenForArmData(30.0);
			
					int selected_grasp_index = -1;
			
			
						//find the grasp with closest orientatino to current pose
						double min_diff = 1000000.0;
						for (unsigned int i = 0; i < push_commands.size(); i++){
							double d_i = segbot_arm_manipulation::grasp_utils::quat_angular_difference(push_commands.at(i).pose.orientation, current_pose.pose.orientation);
							
							ROS_INFO("Distance for pose %i:\t%f",(int)i,d_i);
							if (d_i < min_diff){
								selected_grasp_index = (int)i;
								min_diff = d_i;
								ROS_INFO("picked orientation");
							}
						}
								
					if (selected_grasp_index == -1){
						ROS_WARN("selection failed. kill.");
						//as_.setAborted(result_);
						
					} else {
								ROS_INFO("passed 4");
						first_goal = push_commands.at(selected_grasp_index); 
						//	GraspCartesianCommand gc_i = segbot_arm_manipulation::grasp_utils::constructGraspCommand(current_grasps.grasps.at(i),HAND_OFFSET_APPROACH,HAND_OFFSET_GRASP, sensor_frame_id);
					
					//if(heardGoal){
					/* //just get first poitns that works doeesn't check for anything else
						//may not need used for robustness
						//checks to see if can reach both poses though inverse kinematics
						int changex1 = 0;
						int changey1 = 0;
						int changex2 = 0;
						int changey2 = 0;
						bool isReachable = false;
						while( changex1 < 3 && !isReachable){
							first_goal.pose.position.x += .05;
							
							while( changey1 < 3 && !isReachable){
								first_goal.pose.position.y += .05;
								moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,first_goal);
								
								if(ik_response_approach.error_code.val == 1){
									ROS_INFO("entered first pose passed");
									first_goal_pub.publish(first_goal);
									
									while( changex2 < 3 && !isReachable){
										second_goal.pose.position.x += .05;
										
										while( changey2 < 3 && !isReachable){
											second_goal.pose.position.y += .05;
											moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,first_goal);
											if(ik_response_approach.error_code.val == 1){
												first_goal_pub.publish(first_goal);
												second_goal_pub.publish(second_goal);
												ROS_INFO("entered second pose passed");
												isReachable = true;
											}	
											changey2 ++;
										}		
										
									changex2 ++;
									}
									
								}	
								
								changey1 ++;
							}	
							changex1 ++;
						}	
						*/
						//if(isReachable = true){	
							pressEnter();
							ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
							//segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
							first_goal_pub.publish(first_goal);
							//first_goal_pub.publish(push_commands.at(selected_grasp_index));
							
							
							//made vision calls check in rviz to see if correct then procede
							pressEnter();
							
							ROS_INFO("Demo starting...Move the arm to a 'ready' position .");
							//segbot_arm_manipulation::homeArm(n);
							//segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
							
							ros::spinOnce();
							
							
							pressEnter();
							ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
							segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
							
							ros::spinOnce();                                            
							segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
							ros::spinOnce(); 
							segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
							ros::spinOnce(); 
							//ros::spinOnce();
							bool isReachable = false;
							while( changex1 < 3 && !isReachable){
								second_goal.pose.position.x -= .05;
								
								while( changey1 < 3 && !isReachable){
									second_goal.pose.position.y += .05;
									moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,first_goal);
									
									if(ik_response_approach.error_code.val == 1){
										ROS_INFO("entered first pose passed");
										second_goal_pub.publish(second_goal);
										isReachable = true;
									}	
									
									changey1 ++;
								}	
								changex1 ++;
							}	
							
							ROS_INFO("2nd goal picked...check if pose is what you want in rviz if not ctr c.");
							pressEnter();
							segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
							ros::spinOnce();                 
							segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
							ros::spinOnce();  
							pressEnter();
							ROS_INFO("Demo ending...arm will move back 'ready' position .");
							//segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
							//segbot_arm_manipulation::moveToPoseMoveIt(n,start_pose);
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
					}		
	
			}
};
