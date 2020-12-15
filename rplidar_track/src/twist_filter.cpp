#include "rplidar_track/twist_filter.h"

TwistFilter::TwistFilter()
{
    m_lateral_accel_limit = 5.0;
    m_lowpass_gain_linear_x = 0.0;
    m_lowpass_gain_angular_z = 0.0;
}
TwistFilter::~TwistFilter()
{

}
void TwistFilter::Set(double lateral_accel_limit, double lowpass_gain_linear, double lowpass_gain_angular)
{
    m_lateral_accel_limit = lateral_accel_limit;
    m_lowpass_gain_linear_x = lowpass_gain_linear;
    m_lowpass_gain_angular_z = lowpass_gain_angular;
}
void TwistFilter::Filter(const geometry_msgs::Twist &in_twist, geometry_msgs::Twist  &out_twist)
{

  double v = in_twist.linear.x;
  double omega = in_twist.angular.z;

  if(fabs(omega) < ERROR){
    out_twist = in_twist;
    return;
  }

  double max_v = m_lateral_accel_limit / omega;

  geometry_msgs::TwistStamped tp;
  //tp.header = msg->header;

  double a = v * omega;
  //ROS_INFO("lateral accel = %lf", a);

  tp.twist.linear.x = fabs(a) > m_lateral_accel_limit ? max_v : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  lowpass_linear_x = m_lowpass_gain_linear_x * lowpass_linear_x + (1 - m_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = m_lowpass_gain_angular_z * lowpass_angular_z + (1 - m_lowpass_gain_angular_z) * tp.twist.angular.z;

  out_twist.linear.x = lowpass_linear_x;
  out_twist.angular.z = lowpass_angular_z;

  //ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  //g_twist_pub.publish(tp);
}