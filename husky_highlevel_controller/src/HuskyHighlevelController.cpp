#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <cmath>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle) {
	if (!readParam()) {
		ROS_ERROR("No Parameters found. Requesting shutdown");
		ros::requestShutdown();
	}
	subscriber_ = nodeHandle_.subscribe(sub_topic_Name, sub_queue_Size,
			&HuskyHighlevelController::scanCallback, this);
	publisher_vel = nodeHandle_.advertise<geometry_msgs::Twist>(
			pub_topic_Name_Vel, pub_queue_Size_Vel);
	pub_marker_ = nodeHandle_.advertise<visualization_msgs::Marker>(
			pub_topic_Name_Marker, pub_queue_Size_Marker);
	serviceServer_ = nodeHandle_.advertiseService("SetBool",
			&HuskyHighlevelController::serviceCallback, this);

		ROS_INFO("Successfully launched node.");
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

bool HuskyHighlevelController::readParam() {/*sub_topic_Name is for cpp, sub_topic_name is for config.yaml*/
	if (nodeHandle_.getParam("sub_topic_name", sub_topic_Name)
			&& nodeHandle_.getParam("sub_queue_size", sub_queue_Size)
			&& nodeHandle_.getParam("pub_queue_size_vel", pub_queue_Size_Vel)
			&& nodeHandle_.getParam("pub_topic_name_vel", pub_topic_Name_Vel)
			&& nodeHandle_.getParam("p_gain", p_Gain)
			&& nodeHandle_.getParam("forward_drive", forward_Drive)
			&& nodeHandle_.getParam("pub_topic_name_marker",
					pub_topic_Name_Marker))
		return true;
	return false;
}
// callback function
void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan &msg) {
	int size = msg.ranges.size(); //size of the array
	int minIndex = 0; //the index where minimum is read
	int closestIndex = -1;
	double minVal = 999; // the value of the min distance
	float alpha = 1; // angle of the measurements

	// find minimum distance
	for (int i = minIndex; i < size; ++i) {
		if ((msg.ranges[i] <= minVal) && (msg.ranges[i] >= msg.range_min)
				&& (msg.ranges[i] <= msg.range_max)) {
			minVal = msg.ranges[i];
			closestIndex = i;
		}
	}

	/*
	 * find the angle from your distance measurement, create a geometry message,
	 * multiply your angle by a gain defined in config.yaml
	 * and publish that through your publisher.
	 */
	alpha = msg.angle_min + closestIndex * msg.angle_increment;
	cmd_msg.angular.z = -alpha * p_Gain;
	cmd_msg.linear.x = forward_Drive;

	position_x = minVal * cos(-alpha);
	position_y = minVal * sin(-alpha);

	if (!start_move_) {
		cmd_msg.linear.x = 0.0;
		cmd_msg.angular.z = 0.0;
	}
	publisher_vel.publish(cmd_msg);
	visualizationMarker(position_x, position_y);

	ROS_INFO_STREAM("The distance is: " << msg.ranges[closestIndex]);
	ROS_INFO_STREAM("The angle is: " << alpha);
	ROS_INFO_STREAM("The Position X is: " << position_x);
	ROS_INFO_STREAM("The Position Y is: " << position_y);

}

void HuskyHighlevelController::visualizationMarker(double x, double y) {
	marker.header.frame_id = "base_laser";
	marker.header.stamp = ros::Time();
	marker.ns = "husky_highlevel_controller";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	pub_marker_.publish(marker);
}

bool HuskyHighlevelController::serviceCallback(
		std_srvs::SetBool::Request &request,
		std_srvs::SetBool::Response &response) {
	if (request.data)
		start_move_ = true;
	else
		start_move_ = false;
	response.success = true;

	ROS_INFO("request: %i", request.data);
	ROS_INFO("sending back response: [%i]", response.success);

	return true;
}

} /* namespace */
