#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber scan_sub_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS topic queue_size 
  double queue_size;

};

} /* namespace */
