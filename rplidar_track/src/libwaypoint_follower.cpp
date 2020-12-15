/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rplidar_track/libwaypoint_follower.h"

int WayPoints::getSize() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;
  else
    return current_waypoints_.waypoints.size();
}

double WayPoints::getInterval() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;

  // interval between 2 waypoints
  tf::Vector3 v1(current_waypoints_.waypoints[0].pose.pose.position.x,
                 current_waypoints_.waypoints[0].pose.pose.position.y, 0);

  tf::Vector3 v2(current_waypoints_.waypoints[1].pose.pose.position.x,
                 current_waypoints_.waypoints[1].pose.pose.position.y, 0);
  return tf::tfDistance(v1, v2);
}

geometry_msgs::Point WayPoints::getWaypointPosition(int waypoint) const
{
  geometry_msgs::Point p;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return p;

  p = current_waypoints_.waypoints[waypoint].pose.pose.position;
  return p;
}

geometry_msgs::Quaternion WayPoints::getWaypointOrientation(int waypoint) const
{
  geometry_msgs::Quaternion q;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return q;

  q = current_waypoints_.waypoints[waypoint].pose.pose.orientation;
  return q;
}

geometry_msgs::Pose WayPoints::getWaypointPose(int waypoint) const
{
  geometry_msgs::Pose pose;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return pose;

  pose = current_waypoints_.waypoints[waypoint].pose.pose;
  return pose;
}

double WayPoints::getWaypointVelocityMPS(int waypoint) const
{
  if (waypoint > getSize() - 1 || waypoint < 0)
    return 0;

  return current_waypoints_.waypoints[waypoint].twist.twist.linear.x;
}

bool WayPoints::isFront(int waypoint, geometry_msgs::Pose current_pose) const
{
  double x = calcRelativeCoordinate(current_waypoints_.waypoints[waypoint].pose.pose.position, current_pose).x;
  if (x < 0)
    return false;
  else
    return true;
}

double DecelerateVelocity(double distance, double prev_velocity)
{
  double decel_ms = 1.0;  // m/s
  double decel_velocity_ms = sqrt(2 * decel_ms * distance);

  std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
  if (decel_velocity_ms < prev_velocity)
  {
    return decel_velocity_ms;
  }
  else
  {
    return prev_velocity;
  }
}

// calculation relative coordinate of point from current_pose frame
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// calculation absolute coordinate of point on current_pose frame
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = inverse * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

// distance between target 1 and target2 in 2-D
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

double getRelativeAngle(geometry_msgs::Pose waypoint_pose, geometry_msgs::Pose vehicle_pose)
{
  geometry_msgs::Point relative_p1 = calcRelativeCoordinate(waypoint_pose.position, vehicle_pose);
  geometry_msgs::Point p2;
  p2.x = 1.0;
  geometry_msgs::Point relative_p2 = calcRelativeCoordinate(calcAbsoluteCoordinate(p2, waypoint_pose), vehicle_pose);
  tf::Vector3 relative_waypoint_v(relative_p2.x - relative_p1.x, relative_p2.y - relative_p1.y,
                                  relative_p2.z - relative_p1.z);
  relative_waypoint_v.normalize();
  tf::Vector3 relative_pose_v(1, 0, 0);
  double angle = relative_pose_v.angle(relative_waypoint_v) * 180 / M_PI;
  // ROS_INFO("angle : %lf",angle);

  return angle;
}

// get closest waypoint from current pose
int getClosestWaypoint(const rplidar_track::Lane &current_path, geometry_msgs::Pose current_pose)
{
  WayPoints wp;
  wp.setPath(current_path);

  if (wp.isEmpty())
    return -1;

  // search closest candidate within a certain meter
  double search_distance = 5.0;
  std::vector<int> waypoint_candidates;
  for (int i = 1; i < wp.getSize(); i++)
  {
    if (getPlaneDistance(wp.getWaypointPosition(i), current_pose.position) > search_distance)
      continue;

    if (!wp.isFront(i, current_pose))
      continue;

    double angle_threshold = 90;
    if (getRelativeAngle(wp.getWaypointPose(i), current_pose) > angle_threshold)
      continue;

    waypoint_candidates.push_back(i);
  }

  // get closest waypoint from candidates
  if (!waypoint_candidates.empty())
  {
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (auto el : waypoint_candidates)
    {
      // ROS_INFO("closest_candidates : %d",el);
      double d = getPlaneDistance(wp.getWaypointPosition(el), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = el;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
  else
  {
    ROS_INFO("no candidate. search closest waypoint from all waypoints...");
    // if there is no candidate...
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (int i = 1; i < wp.getSize(); i++)
    {
      if (!wp.isFront(i, current_pose))
        continue;

      // if (!wp.isValid(i, current_pose))
      //  continue;

      double d = getPlaneDistance(wp.getWaypointPosition(i), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = i;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
{
  //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return d;
}

tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

geometry_msgs::Point vector2point(tf::Vector3 vector)
{
  geometry_msgs::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();
  return point;
}

tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree)
{
  tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
                 sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
  tf::Vector3 unit_w1 = w1.normalize();

  return unit_w1;
}

geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}
/*
double xa:点A坐标
double ya:点A坐标
double ka:过点A的直线斜率
double s: 距离A点的距离
double x1: 返回的目标点1的坐标
double y1: 返回的目标点1的坐标
double x2: 返回的目标点2的坐标
double y2: 返回的目标点2的坐标
说明：已知过A点(xa,ya)的直线斜率为ka，求直线上距离A点距离为s的点的坐标
	对接近0,90,180,270的情况单独讨论，其他情况通过一元二次方程求解。
	当与x轴或y轴距离0.2度时，等效于平行x轴或者y轴, 相应k的值如下：
	当与y轴距离0.2度时，k： tan(89.8) = 286.4777 ; tan(90.2) = -286.4777
	当与y轴距离0.2度时，k： tan(0.2) = 0.000349 ; tan(179.8) = -0.00349
	fabs(k)的值随接近y轴而增大，随接近x轴而减小。

返回：返回true，表示存在两个解，两个点的坐标，(x1,y1)(x2,y2)；
	返回false，表示没有解。
*/
bool getIntersectionBetweenCircleAndLine(double xa, double ya, double ka, double s, double *x1, double *y1, double *x2, double *y2)
{

	// 等效于平行x轴
	if(fabs(ka) < 0.0035)
	{
		*x1 = xa - s;
		*x2 = xa + s;

		*y1 = ya;
		*y2 = ya;

		return true;
	}
	// 等效于平行y轴
	else if(fabs(ka) > 286)
	{
		*x1 = xa;
		*x2 = xa;
		
		*y1 = ya + s;
		*y2 = ya - s;

		return true;
	}
	// 一元二次方程系数
	double a, b, c;
	a = 1 + pow(ka, 2);
	b = 2 * ka * (ya - ka * xa) - 2 * xa - 2 * ya * ka;
	c = pow(xa,2) + pow(ya - ka*xa, 2) - 2 * ya * (ya - ka * xa) + pow(ya, 2) - pow(s , 2);
	if( (pow(b,2) - 4*a*c) >= 0)
	{
		*x1 = (-1 * b + sqrt(pow(b,2) - 4 * a * c) ) / (2 * a);
		*x2 = (-1 * b - sqrt(pow(b,2) - 4 * a * c) ) / (2 * a);


		*y1 = ka * (*x1 - xa) + ya;
		*y2 = ka * (*x2 - xa) + ya;

		return true;
	}
	else
	{
		return false;
	}
  
}
/*
double la: 直线参数
double lb: 直线参数
double lc: 直线参数
double x0:圆心坐标
double y0:圆心坐标
double r：圆半径
double x1: 返回的目标点1的坐标
double y1: 返回的目标点1的坐标
double x2: 返回的目标点2的坐标
double y2: 返回的目标点2的坐标
说明：
已知直线方程：la * x + lb * y + lc = 0
已知圆参数：圆心(x0,y0), 半径:r
求圆和直线的两个交点.
联立方程
(x-x0)^2 + (y-y0)^2 = r^2
ax+by+c = 0

https://blog.csdn.net/u013013797/article/details/86148421
*/
bool getIntersectionBetweenCircleAndLine(double la, double lb, double lc, double x0, double y0, double r, double *x1, double *y1, double *x2, double *y2)
{
  // (x0, y0)到直线距离 > r 返回false
  if(la ==0 && lb == 0) return false;
  if(lb == 0)
  {
    *x1 = -lc / la;
    *x2 = -lc / la;

    *y1 = y0 + sqrt(pow(r, 2) - pow(*x1 - x0, 2));
    *y2 = y0 - sqrt(pow(r, 2) - pow(*x2 - x0, 2));
     return true;
  }
  //  ax + by + c = 0 等效 y = ka *x + kb
  double ka = -la / lb;
  double kb = -lc / lb;
 // 等效于平行x轴
	if(fabs(ka) < 0.0035)
	{
		*y1 = -lc / lb;
		*y2 = -lc / lb;

		*x1 = x0 + sqrt(pow(r, 2) - pow(*y1 - y0, 2));
		*x2 = x0  - sqrt(pow(r, 2) - pow(*y2 - y0, 2));
		return true;
	}
	// 等效于平行y轴
	else if(fabs(ka) > 286)
	{
    *x1 = -lc / la;
    *x2 = -lc / la;

    *y1 = y0 + sqrt(pow(r, 2) - pow(*x1 - x0, 2));
    *y2 = y0 - sqrt(pow(r, 2) - pow(*x2 - x0, 2));
		return true;
	}
	// 一元二次方程系数
	double a, b, c;
	a = 1 + pow(ka, 2);
	b = 2 * ka *(kb - y0) - 2 * x0;
	c = pow(x0,2) + pow(kb - y0, 2) - pow(r, 2);

	if( (pow(b,2) - 4*a*c) >= 0)
	{
		*x1 = (-1 * b + sqrt(pow(b,2) - 4 * a * c) ) / (2 * a);
		*x2 = (-1 * b - sqrt(pow(b,2) - 4 * a * c) ) / (2 * a);


		*y1 = ka * (*x1) + kb;
		*y2 = ka * (*x2) + kb;

		return true;
	}
	else
	{
		return false;
	}

}
