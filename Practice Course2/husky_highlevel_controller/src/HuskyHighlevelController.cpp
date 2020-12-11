#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

// STD
#include <string>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) 
	: nodeHandle_(nodeHandle)
{
  nodeHandle.getParam("scan_sub_topic",subscriberTopic_);
  nodeHandle.getParam("scan_sub_queue_size",queue_size);

  scan_sub_ = nodeHandle_.subscribe(subscriberTopic_, queue_size,		  
				   &HuskyHighlevelController::scanCallback, this);
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}


void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& scan_msg)
    {
        float smallest_distance = 100;
        // the angle corresponding to the minimum distance
      
 
        //number of the elements in ranges array
        int arr_size = floor((scan_msg.angle_max-scan_msg.angle_min + 1)/scan_msg.angle_increment);
        for (int i=0 ; i< arr_size ;i++)
        {
            if (scan_msg.ranges[i] < smallest_distance)
            {
                smallest_distance = scan_msg.ranges[i];
            }
        }
  ROS_INFO("ROS_INFO Minimum laser distance(m): %lf", smallest_distance);
    }

} /* namespace */
