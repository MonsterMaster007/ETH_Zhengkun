#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	  /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
         bool readParameters();

	 /*!
      	* ROS topic callback method.
   	* @param message the received message.
  	 */
  	void scanCallback(const sensor_msgs::LaserScan& scan_msg);

  	/*!
   	* ROS service server callback.
   	* @param request the request of the service.
   	* @param response the provided response.
   	* @return true if successful, false otherwise.
   	*/

 	 /*!
   	* ROS service server callback.
   	* @param request the request of the service.
   	* @param response the provided response.
   	* @return true if successful, false otherwise.
   	*/
  	bool srvCallback(std_srvs::SetBool::Request& request,
                       	     std_srvs::SetBool::Response& response);

  	//! ROS node handle.
  	ros::NodeHandle& nodeHandle_;

  	//--ROS topic subscriber--// //Subscriber to /scan
  	ros::Subscriber scan_sub_;

  	//! ROS topic name to subscribe to.
  	std::string subscriberTopic_;

  	//! ROS topic queue_size 
 	double queue_size;

	//-----Pillar info----
	////pillar position
	float x_pillar;
	float y_pillar;
	// the orientation of the pillar with respect to the x_axis
	float alpha_pillar;

	//--ROS topic Publisher--// //Publisher to /cmd_vel
	ros::Publisher cmd_pub_;
	//--ROS vis-marker Publisher--// Publisher to visualization_marker
	ros::Publisher vis_pub_;
	
	//msg twist
	geometry_msgs::Twist vel_msgs_;

 	//! ROS service server.
  	ros::ServiceServer serviceServer_;
	
	bool start_move = true;
};

} /* namespace */
