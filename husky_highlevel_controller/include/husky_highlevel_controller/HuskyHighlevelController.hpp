#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_srvs/SetBool.h>

//#include <string>
#include "visualization_msgs/MarkerArray.h"

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle &nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	bool readParam();
	void scanCallback(const sensor_msgs::LaserScan &msg);
	void visualizationMarker(double x, double y);

	ros::Subscriber subscriber_;
	ros::Publisher publisher_vel;
	ros::Publisher pub_marker_;
	ros::NodeHandle nodeHandle_;
	ros::ServiceServer serviceServer_;

	bool serviceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

	geometry_msgs::Twist cmd_msg;
	visualization_msgs::Marker marker;

	std::string sub_topic_Name;
	int sub_queue_Size;
	std::string pub_topic_Name_Vel;
	int pub_queue_Size_Vel;
	float p_Gain;
	float forward_Drive;
	std::string pub_topic_Name_Marker;
	int pub_queue_Size_Marker;
	float position_x, position_y;
	bool start_move_;
};

} /* namespace */
