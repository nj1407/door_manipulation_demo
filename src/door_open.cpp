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

//tf stuff
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


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
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>


#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <time.h>
#include <stdint.h>

Eigen::Vector4f centroid;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud (new PointCloudT);
std::string frame_ids;
ros::Time save_Time;
//std::uint32_t save_seq;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
		frame_ids = input->header.frame_id;
		save_Time = input->header.stamp;
		//seqs = input->header.seq;
		//convert to PCL format
		pcl::fromROSMsg (*input, *cloud);
		//get centroid and move it up .1 m 
		//used to get goal xyz
		pcl::compute3DCentroid(*cloud,centroid);
		centroid.x() += .1;
	    
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_door_open_detector");
	ros::NodeHandle n;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/xtion_camera/depth_registered/points";
	ros::Subscriber sub = n.subscribe (param_topic, 1, cloud_cb);
	
	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	ros::Subscriber cloud = n.subscribe ("/door_handle_detection/cloud", 1,cloud_cb);

	//service
	
	ros::ServiceClient client = n.serviceClient<door_manipulation_demo::door_perception>("/door_handle_detection/door_perception");
	
	ros::ServiceClient client_move = n.serviceClient<moveit_utils::MicoMoveitCartesianPose>("mico_cartesianpose_service");
	
	geometry_msgs::PoseStamped goal;
	geometry_msgs::Pose goal_pose;
	//get pose
    goal_pose.position.x = centroid.x();
    goal_pose.position.y = centroid.y();
    goal_pose.position.z = centroid.z();
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
    goal.pose = goal_pose;
    //get header
    std_msgs::Header goal_header;
    goal_header.seq = 0;
    goal_header.stamp = save_Time;
    goal_header.frame_id = frame_ids;
    goal.header = goal_header;
    
	door_manipulation_demo::door_perception door_srv;
	moveit_utils::MicoMoveitCartesianPose mico_srv;
	mico_srv.request.target = goal;
	
	//make calls to get vision
	if(client.call(door_srv)){
		ros::spinOnce();
		if(client.call(door_srv)){
			ROS_INFO("entered");
			ros::spinOnce();
		}	
	} else {
		ROS_INFO("didn't enter");
	}		
	ros::spinOnce();
	if(client_move.call(mico_srv)){
		ROS_INFO(" entered srv 2");
	} else {
		ROS_INFO("didn't entered srv 2");
	}
	
	goal_pose.position.y += .1;
	
	
	ros::spinOnce();
	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);
	

	
};
