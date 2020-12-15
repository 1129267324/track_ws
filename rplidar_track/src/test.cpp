#include "ros/ros.h"
#include "ros/rate.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "rplidar_track/ylog.h"

int m_WallPos = 0;
ros::Publisher m_TwistPub;
void SaveMinDis(const sensor_msgs::LaserScan::ConstPtr &msg, int start, int end)
{
	// m
	double minDis = 12;
	int minIdx = 0;
	for(int i = start ; i < end; i++)
	{
		if(isnormal(msg->ranges[i]) && msg->ranges[i] < minDis)
		{
			minDis = msg->ranges[i];
			minIdx = i;
		}
	}
	YLog log(YLog::INFO, "min_distance_log.txt", YLog::ADD);
	log.w_int_double(minIdx, minDis);	
}
void rplidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	int start = m_WallPos == 1 ? 225 : 45;
	int end = m_WallPos == 1 ? 315 : 135;
    SaveMinDis(msg, start, end);
}
void publishTwistStamped(const bool &can_get_curvature, const double &kappa) 
{
  	geometry_msgs::Twist ts;
	double velocity = 0.2;
  	ts.linear.x = can_get_curvature ? velocity : 0;
  	ts.angular.z = can_get_curvature ? kappa * ts.linear.x : 0;
  	m_TwistPub.publish(ts);
}
void TrackCalcCallback(const ros::TimerEvent&)
{
publishTwistStamped(true, 2);
}
void diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr &msg)
{
	YLog log(YLog::INFO, "diagnostic_log.txt", YLog::ADD);
	double d[5];
	int count = 5;
	std::string::size_type sz;
	d[0] = std::stof(msg->values[0].value, &sz);
	log.w_double_arr(__FILE__, __LINE__, YLog::INFO,"", d, count, false);	
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;
	n.param<int>("/track_node/wall_position", m_WallPos, 1);
	//printf("main m_WallPos = %d\n", m_WallPos);
    ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &rplidar_callback);	
	ros::Subscriber diag_sub = n.subscribe<diagnostic_msgs::DiagnosticStatus>("track_node/diagnostic_status", 1000, &diagnostic_callback);
	m_TwistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Timer param_pub_timer = n.createTimer(ros::Duration(1), &TrackCalcCallback);
    ros::spin();
	return 0;
}
