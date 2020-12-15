#ifndef TWIST_FILTER_h
#define TWIST_FILTER_h

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
constexpr double RADIUS_MAX = 9e10;
constexpr double ERROR = 1e-8;
class TwistFilter
{
public:
    TwistFilter();
    ~TwistFilter();
    void Set(double lateral_accel_limit, double lowpass_gain_linear, double lowpass_gain_angular);
    void Filter(const geometry_msgs::Twist &in_twist, geometry_msgs::Twist  &out_twist);
protected:
    double m_lateral_accel_limit;
    double m_lowpass_gain_linear_x;
    double m_lowpass_gain_angular_z;
};
#endif