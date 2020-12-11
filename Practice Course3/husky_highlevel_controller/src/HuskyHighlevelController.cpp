#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

// STD
#include <string>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) 
	: nodeHandle_(nodeHandle)
{
  if (!readParameters()) 
  {
      ROS_ERROR("Could not read parameters.");
      ros::requestShutdown();
  }

  scan_sub_ = nodeHandle_.subscribe(subscriberTopic_, queue_size,		  
				   &HuskyHighlevelController::scanCallback, this);
  cmd_pub_=nodeHandle_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );
  vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>( "/visualization_marker", 10 );
}

HuskyHighlevelController::~HuskyHighlevelController()
{

}
bool HuskyHighlevelController::readParameters()
    {
        if (!nodeHandle_.getParam("scan_sub_topic", subscriberTopic_)) 
	{
		ROS_ERROR("Could not find scan_sub_topic parameter!");
		return false;
	}
       
        if (!nodeHandle_.getParam("scan_sub_queue_size", queue_size))
	{	
		ROS_ERROR("Could not find scan_sub_queue_size parameter!");
		return false;
	}
       
        return true;
    }

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan &scan_msg)
    {
        float smallest_distance = 100;
        // the angle corresponding to the minimum distance
      
 
        //number of the elements in ranges array
        int arr_size = floor((scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment)+1;
        for (int i=0 ; i< arr_size ;i++)
        {
            if (scan_msg.ranges[i] < smallest_distance)
            {
                smallest_distance = scan_msg.ranges[i];
		alpha_pillar = (scan_msg.angle_min + i*scan_msg.angle_increment);

            }
       
        }

	//Pillar Husky pffset pose
 	x_pillar = smallest_distance*cos(alpha_pillar);
	y_pillar = smallest_distance*sin(alpha_pillar);


	ROS_INFO("Pillar pffset angle(rad):%lf", alpha_pillar);
	ROS_INFO("pillar x distance(m):%lf", x_pillar);
	ROS_INFO("pillar y distance(m):%lf", y_pillar);

	float p_gain_vel = 0.1;
	float p_gain_ang = 0.4;

	 if (x_pillar > 0.2)
        {
	     if (x_pillar <= 0.4)
		{
		   vel_msgs_.linear.x = 0;
	           vel_msgs_.angular.z = 0;
		}
	     else
		{
		   vel_msgs_.linear.x = x_pillar * p_gain_vel;
		   vel_msgs_.angular.z = -(y_pillar * p_gain_ang);
		}
		
	}
	else 
	{
		   vel_msgs_.linear.x = 0;
		   vel_msgs_.angular.z = 0;
	}
     
	cmd_pub_.publish(vel_msgs_);

	//Rviz Marker
	visualization_msgs::Marker marker;
    	marker.header.frame_id = "base_laser";
    	marker.header.stamp = ros::Time();
    	marker.ns = "pillar";
    	marker.id = 0;
   	marker.type = visualization_msgs::Marker::SPHERE;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.position.x = x_pillar;
    	marker.pose.position.y = y_pillar;
   	marker.scale.x = 0.2;
   	marker.scale.y = 0.2;
   	marker.scale.z = 2.0;
   	marker.color.a = 1.0; // Don't forget to set the alpha!
   	marker.color.r = 0.1;
   	marker.color.g = 0.1;
   	marker.color.b = 0.1;
   	vis_pub_.publish( marker );	

    }
} /* namespace */
