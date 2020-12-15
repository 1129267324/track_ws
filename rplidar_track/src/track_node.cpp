#include "ros/ros.h"
#include "ros/time.h"
#include <string.h> 
#include "stdio.h"
#include "time.h"  
#include "stdlib.h"
#include <vector> 
#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "rplidar_track/pure_pursuit.h"
#include "rplidar_track/MyRANSAC.h"
#include "rplidar_track/LineParamEstimator.h"
#include "rplidar_track/ylog.h"
#include "rplidar_track/PID.h"
#include "rplidar_track/twist_filter.h"

#include <boost/thread.hpp> 
#include <boost/foreach.hpp>

#include <exception>
using  namespace std;
using namespace boost;

#ifndef DEBUG  
#define DEBUG  
#endif  
#ifndef D_EXECUTE_TIME
//#define D_EXECUTE_TIME
#endif 
#ifndef DIAGNOSTIC_PUB
#define DIAGNOSTIC_PUB
#endif
class TraceCtrl
{
	public:
		TraceCtrl();
		~TraceCtrl();

	// 当前位置与AB点的位置关系
	enum 
	{
		BETWEEN_IN_AB,
		A_SIDE,
		B_SIDE
	};
	// 墙相对车的位置
	enum 
	{
		// 墙在车左侧
		LEFT_SIDE = 1,
		RIGHT_SIDE
	};

	public:
		bool Init();
		void Run();
	protected:
		void LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
		void TrackCalcCallback(const ros::TimerEvent&);
		void Algorithm_1();
		void Algorithm_2();
		void Algorithm_3();
		// 计算当前位置距离墙面最小距离点的坐标
		bool GetClosestPoint(geometry_msgs::Point &p) ;
		// 计算点的坐标
		geometry_msgs::Point CalcCoordinateBaseLaser(int idx, float distance) const;
		// 获取车辆前方墙面指定一侧合适的点A, 返回A点在车辆坐标系下的坐标，及斜率k
		bool GetWallPtA(geometry_msgs::Point & ptA, double &k)  const;
		bool GetPtAFromRanges(int idx, geometry_msgs::Point & ptA, double &k) const;
		bool GetPtAFromRansac(int idx, geometry_msgs::Point & ptA, double &k) const;
		// 车辆与墙切线存在夹角，在A点的基础上另取A2代替A
		bool GetWallPtA2(geometry_msgs::Point & ptA, double &k) const;
		// 计算直线上到指定点的距离等于给定值的坐标点
		bool getDistancePoint(double xa, double ya, double ka, double s, double *x1, double *y1, double *x2, double *y2) const;
		// 在A点法线上选择点B，使|A-B|= d，返回B点在车辆坐标系下的坐标
		bool GetFrontViewPtB(const geometry_msgs::Point & ptA, const double &k, geometry_msgs::Point & ptB)  const;
		// 计算以B为目标点的曲率Kappa
		bool  CalcCurvature(const geometry_msgs::Point & ptB, const geometry_msgs::Pose &curpose, double *output_kappa)  const;
		// 发布计算结果
		void publishTwistStamped(const bool &can_get_curvature, const double &kappa);
		// 计算线速度
		double computeCommandVelocity(const double &kappa) const;
		// 数据预处理
		void DataPreprocess();
		// 转换曲率到转角, 弧度
		double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa);
protected:
		// 计算转向角度，度
		double CalcSteerAngle(const geometry_msgs::Pose &curpose, const geometry_msgs::Point &ptB) const;
		// PID计算以B点为目标点，以转向角为误差的曲率
		bool CalcPIDCurvature(const double &steerAngle, double *output_kappa);
		// 
		void InitPID();
		// PID 参数 
		PID m_PID;
		double m_Setpoint;
		double m_Input;
		double m_Output;
		double m_Kp;
		double m_Ki; 
		double m_Kd;  
protected:
		// 显示Marker
		void ShowMarker(double x, double y, std_msgs::ColorRGBA color) const;
		// 显示线条
		void ShowLine(const std::vector<geometry_msgs::Point> &pts, const ros::Publisher &pub) const ;
		// 显示当前点到B点的曲率路径
		void ShowCircle(const geometry_msgs::Point &target, const double &kappa) const;
		// 显示A处切线
		void ShowALine(const geometry_msgs::Point &ptA, const double &k) const;
		// 诊断数据
		void publishDiognostic();
protected:
		// 后轴中心前视距离, 固定, cm
		double m_Lfd; 
  		// 前视距离比率
  		double m_lookahead_distance_ratio;
  		// 最小前视距离
  		double m_minimum_lookahead_distance; 
		// 前后轮之间的轴距, m
		double m_L;		
		// 距离墙的保持距离, cm
		double m_KeepDistance;
		// 墙相对车的位置 1: 在车左侧， 2：右侧
		int m_WallPos;
		// 计算转向的周期，s
		double m_ClacCtrlPeriod;
		// 目标速度 m/s
		double m_KeepSpeed;
		// 最小目标速度 m/s
		double m_MinKeepSpeed;
protected:
		// 部分雷达参数
		std::vector<float> m_Ranges;
		boost::mutex m_RangesMutex;	
		float m_AngleIncrement;
		float m_RangeMin;
		float m_RangeMax;
		
protected:
		ros::Publisher m_TwistPub;
		ros::Publisher m_MakerPub;
		ros::Publisher m_LinePub;
		// 过A点切线
		ros::Publisher m_ALinePub;
		// AB线
		ros::Publisher m_ABLinePub;
		// 曲率半径
		ros::Publisher m_OBCirclePub;
		string m_FrameID;
		// 诊断
		diagnostic_msgs::DiagnosticStatus m_DiagStat;
		ros::Publisher m_DiagPub;
protected:
	// 速度滤波 
	TwistFilter m_TwistFilter;
	bool m_IsFilter;
protected:
	// 算法选项 1：pure 2:pid
	int m_Method; 
};
TraceCtrl::TraceCtrl():
m_AngleIncrement(1),
m_RangeMin(0.15),
m_RangeMax(12)
{
m_Ranges.resize(360);
}
TraceCtrl::~TraceCtrl()
{

}
void TraceCtrl::LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
	m_RangesMutex.lock();
	m_Ranges.assign(msg->ranges.begin(), msg->ranges.end());
	m_AngleIncrement = msg->angle_increment;
	m_RangeMin = msg->range_min;
	m_RangeMax = msg->range_max;
	DataPreprocess();
	m_RangesMutex.unlock();
}
void TraceCtrl::TrackCalcCallback(const ros::TimerEvent&)
{
#ifdef D_EXECUTE_TIME
   	clock_t start, end;
	double duration;
	start = clock();
#endif
	switch(m_Method)
	{
		case 1:
			Algorithm_1();
		break;
		case 2:
			Algorithm_2();		
		break;
		default:
		break;
	}

#ifdef D_EXECUTE_TIME
 	end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    printf( "Algorithm_1 : %f seconds\n", duration );	
#endif	

}

void TraceCtrl::Algorithm_1()
{
	// 1. 求车辆前方墙上的目标点A，以及A点处的斜率k
	geometry_msgs::Point ptA ;
	geometry_msgs::Point ptB;
	geometry_msgs::Pose curpose;
	double k;
	if(!GetWallPtA(ptA, k)) return;
	ShowALine(ptA, k);
	// 2. 在墙面法线上取点B，使B到墙的距离|A-B|等于保持距离
	if(!GetFrontViewPtB(ptA, k, ptB)) return;
	// 3. 计算当前位置到点B的曲率
	double kappa ;
	memset(&curpose, 0, sizeof(curpose));
	bool can_get_kappa = CalcCurvature(ptB, curpose, &kappa);
	// 4.发布
	publishTwistStamped(can_get_kappa,  kappa) ;

	ShowCircle(ptB, kappa);
}

void TraceCtrl::Algorithm_2()
{
	// 1. 求车辆前方墙上的目标点A，以及A点处的斜率k
	geometry_msgs::Point ptA ;
	geometry_msgs::Point ptB;
	geometry_msgs::Pose curpose;
	double k;
	if(!GetWallPtA(ptA, k)) return;
	ShowALine(ptA, k);
	// 2. 在墙面法线上取点B，使B到墙的距离|A-B|等于保持距离
	if(!GetFrontViewPtB(ptA, k, ptB)) return;
	// 3. 计算当前位置到B点的转向角度,度
	double angle;
	memset(&curpose, 0, sizeof(curpose));	
	angle = CalcSteerAngle(curpose, ptB);
	// 4.PID计算曲率
	double kappa ;
	bool can_get_kappa = CalcPIDCurvature(angle, &kappa);		
	// 5. 发布
	publishTwistStamped(can_get_kappa,  kappa) ;

	ShowCircle(ptB, kappa);
}
void TraceCtrl::Algorithm_3()
{
	
}
/*
说明：
计算墙面上距离车辆最小距离点的坐标
以车辆坐标系为参考坐标系, rplidar x轴正向要与车辆正向一致
*/
bool TraceCtrl::GetClosestPoint(geometry_msgs::Point &p) 
{ 	
	int idx = 360;
	auto f = [&, this](int s, int e)->bool{
		bool b = false;
		double d = 15;
		this->m_RangesMutex.lock();
		for(int i = s; i < e; i++)
		{
			if(!isnormal(this->m_Ranges[i])) continue;
			if(m_Ranges[i] < d)
			{
				d = m_Ranges[i];
				idx = i;
				b = true;
			}
		}
		this->m_RangesMutex.unlock();
		return b;
		};
 
	int s, e;
	if(m_WallPos == LEFT_SIDE)
	{
		s = 225;  e = 315;
	}
	else if(m_WallPos == RIGHT_SIDE)
	{
		s = 45; e = 90;
	}

	if(f(s, e))
	{
		p = CalcCoordinateBaseLaser(idx, m_Ranges[idx]);
		return true;
	}
	return false;
}
/*
说明：
m_Ranges[idx] = distance
根据序号和距离值计算该点相对车辆坐标系的相对坐标
rplidar x轴正向要与车辆正向一致
*/
geometry_msgs::Point TraceCtrl::CalcCoordinateBaseLaser(int idx, float distance) const
{
	// 从x正向逆时针旋转的角度, 弧度
	double alpha = M_PI + idx * m_AngleIncrement;
	if(alpha > 2 * M_PI) alpha -= 2*M_PI;
	geometry_msgs::Point p;
	p.x = distance * cos(alpha);
	p.y = distance * sin(alpha);
	p.z = 0;
	//printf("CalcCoordinateBaseLaser m_AngleIncrement = %f, alpha = %lf, x = %lf, y = %lf\n",  \
	m_AngleIncrement * 180/M_PI, alpha* 180/M_PI, p.x, p.y);
	return p;
}
/*
说明：
 在车辆前方墙面指定一侧选择一个合适的点A, 
 点A的选择是按前视距离/保持距离计算的角度换算为扫描点的序号，
 返回A点在车辆坐标系下的坐标，及斜率k
 前视距离m_Lfd的取值须在激光雷达的最大范围12m内，可取1m-2m
 1.5m范围内rplidar测距精度<0.5mm, 全程误差小于测量距离的1%
 如果点A以及前后两个关键点没有合理的扫描值即Ranges[i]无效，
 启动ransac算法对指定范围内的点集拟合直线，在直线上计算点A和k
 存在的问题：
 更适合墙面纵向曲率较小的情况
 A1,A2点的选取可能会失败
 ptA, A1, A2单位m
*/
bool TraceCtrl::GetWallPtA(geometry_msgs::Point & ptA, double &k)  const
{	
	// m_Lfd 前视距离 cm, m_KeepDistance 保持距离cm
	// 前视点A到y轴的夹角，度
	double theta = atan(m_Lfd / m_KeepDistance) * 180 / M_PI;

	int idx = (m_WallPos == LEFT_SIDE) ? (269 - static_cast<int> (theta + 0.5)) : (89 + static_cast<int> (theta + 0.5));
	// 关键点无效
	if(!isnormal(m_Ranges[idx]) || !isnormal(m_Ranges[idx + 5]) || !isnormal(m_Ranges[idx - 5]))
	{
		if(!GetPtAFromRansac(idx, ptA, k)) return false;
	}
	else
	{
		if(!GetPtAFromRanges(idx, ptA, k)) return false;
	}
	//printf("GetWallPtA  idx = %d, m_Ranges[idx] = %lf, A.x = %lf, A.y = %lf, k = %lf , atan(k) = %lf\n", idx, m_Ranges[idx], ptA.x, ptA.y, k, atan(k) * 180 / M_PI);
	// 车辆姿态与墙面切线不平行且A点到车辆距离小于前视距离时的情况
	// m_Ranges[] :m , m_Lfd :cm
	if((m_Ranges[idx] < m_Lfd * 0.01) && (fabs(k) > tan(M_PI/9) ))
	{
		return GetWallPtA2(ptA, k);
	}
	return true;
}
/*
说明：
 在A点法线上选择点B，使|A-B|= d，返回B点在车辆坐标系下的坐标
 */
bool TraceCtrl::GetFrontViewPtB(const geometry_msgs::Point & ptA, const double &k, geometry_msgs::Point & ptB)  const
{
	// 弧度
	double theta = atan(k);
	// m
	double d = m_KeepDistance * 0.01 * (m_WallPos == LEFT_SIDE ? -1 : 1);
	ptB.y = cos(theta) * d + ptA.y;
	ptB.x = sin(theta) * (-d) + ptA.x;
	//===============
	//printf("GetFrontViewPtB xa = %f, ya = %f, theta = %lf , k = %lf,  d = %lf,  xb = %f, yb = %f\n", \
	ptA.x, ptA.y, theta, k , d, ptB.x , ptB.y);
	vector<geometry_msgs::Point> pts;
	pts.push_back(ptA); pts.push_back(ptB);
	ShowLine(pts, m_ABLinePub);	
	//===============	
	return true;
}
/*
说明： 计算以B为目标点的曲率Kappa
kappa < 0 右转， kappa > 0 左转 
*/
bool  TraceCtrl::CalcCurvature(const geometry_msgs::Point & ptB, const geometry_msgs::Pose &curpose, double *output_kappa)  const
{
  double denominator = pow(getPlaneDistance(ptB, curpose.position), 2);
  double numerator = 2 * ptB.y;
//printf("CalcCurvature denominator = %lf, numerator = %lf, kappa = %lf\n", denominator, numerator, numerator / denominator);

  if (denominator != 0)
    *output_kappa = numerator / denominator;
  else
  {
	  return false;
  }	
	return true;
}
/*
说明：
	发布控制结果：线速度和角速度
*/
void TraceCtrl::publishTwistStamped(const bool &can_get_curvature, const double &kappa) 
{
  	geometry_msgs::Twist ts;
  //ts.header.stamp = ros::Time::now();
  //ts.header.frame_id = "laser";
  
  	ts.linear.x = can_get_curvature ? computeCommandVelocity(kappa) : 0;
  	ts.angular.z = can_get_curvature ? kappa * ts.linear.x : 0;
  	geometry_msgs::Twist out_ts;
   if(m_IsFilter)
		m_TwistFilter.Filter(ts, out_ts);	   
   else
	   out_ts = ts;
  	m_TwistPub.publish(out_ts);
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "kappa";
	kv.value = std::to_string(kappa);
	m_DiagStat.values.push_back(kv);	
	

}
/*
说明：
	计算线速度
	与设定匀速速度、前方墙面曲率、终点位置有关
*/
double TraceCtrl::computeCommandVelocity(const double &kappa) const
{
	// 最小速度 m/s
	double minSpeed = m_MinKeepSpeed;
	// 减速半径R m
	double R = 2;
	// 返回速度 m/s
	double speed = m_KeepSpeed;
	if(fabs(1/ kappa) < R )
	{
		speed = m_KeepSpeed / (kappa * R) < minSpeed ? minSpeed : m_KeepSpeed / (kappa * R);
	}
	//printf("computeCommandVelocity r = %lf m, speed = %lf\n", 1/kappa, speed);
	return speed;
}
/*
说明：
	扫描数据预处理,车辆距墙10cm，激光雷达离墙至少30cm，无需处理inf数据
*/
void TraceCtrl::DataPreprocess()
{
	auto f = [&](int s, int e)->void{

	};
	m_WallPos == LEFT_SIDE ?  f(179, 269) : f(89, 179);
 	
}
// 显示Marker
void TraceCtrl::ShowMarker(double x, double y, std_msgs::ColorRGBA color) const
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = m_FrameID;
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = rand();
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
 
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color.r;
	marker.color.g = color.g;
	marker.color.b = color.b;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	m_MakerPub.publish( marker );

}
// 显示线条
void TraceCtrl::ShowLine(const std::vector<geometry_msgs::Point> &pts, const ros::Publisher &pub) const
{
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = m_FrameID;
	geometry_msgs::PoseStamped pose;
	BOOST_FOREACH(auto p, pts)
	{
		pose.pose.position.x = p.x;
		pose.pose.position.y = p.y;
		pose.pose.position.z = 0;
		path.poses.push_back(pose);
	}
	pub.publish(path);
}
/*
参数：
target: 目标点
kappa:曲率
说明：
一段圆弧经过当前坐标 (0,0) 和目标点target (x,y),且曲率是kappa,
圆心在y轴上，半径为1/kappa，圆心坐标(0, +/- 1/kappa)，
正负方向视目标点相对当前点的位置。
显示从当前点到目标点的曲率路径
*/
void TraceCtrl::ShowCircle(const geometry_msgs::Point &target, const double &kappa) const
{
	vector<geometry_msgs::Point> pts;
	// 圆心(x0, y0)
	double x0 = 0;
	//  kappa 与 target.y同符号
	double y0 = 1 / kappa;
	// 取点数
	double count = 10;
	// 步长
	double step = 1 / (fabs(kappa) * count);	
	auto f = [&](int s, int e, int sign)->void{
		geometry_msgs::Point pt;
		int i = s;
		while(s < e ? i <= e : i >= e)
		{
			pt.x  = i * step;
			//  (x - x0)^2 + (y - y0)^2 = (1/kappa)^2
			pt.y = sqrt(pow(1/kappa, 2) - pow(pt.x - x0, 2)) * sign  + y0;
			pts.push_back(pt);
			if(fabs(pt.y) > fabs(target.y)) break;			
			s < e ? i++: i--;
		}
	};
	if(target.y > 0 && fabs(target.y) >= fabs(1/kappa))
	{
		f(0, count, -1);
		f(count, 0, 1);
	}
	else if(target.y > 0 && fabs(target.y) < fabs(1/kappa))
	{
		f(0, count, -1);
	}
	else if(target.y < 0 && fabs(target.y) >= fabs(1/kappa))
	{
		f(0, count, 1);
		f(count, 0, -1);		
	}
	else if(target.y < 0 && fabs(target.y) < fabs(1/kappa))
	{
		f(0, count, 1);		
	}
	ShowLine(pts, m_OBCirclePub);
	//printf("ShowCircle kappa = %lf, t.x = %lf, t.y = %lf, x0 = %lf, y0 = %lf, r = %lf\n",  kappa, target.x, target.y, x0, y0, 1/ kappa);
}
/*
说明：
根据ranges[idx]均有效，选择A点计算相对坐标和A点处斜率
*/
bool TraceCtrl::GetPtAFromRanges(int idx, geometry_msgs::Point & ptA, double &k) const
{
	
	//if(!isnormal(m_Ranges[idx])) return false;
	//m_RangesMutex.lock();	
	ptA = CalcCoordinateBaseLaser(idx, m_Ranges[idx]);
	//m_RangesMutex.unlock();	
	// 前后各取一点，相隔10度
	//if(!isnormal(m_Ranges[idx + 5])) return false;	
	geometry_msgs::Point A1 = CalcCoordinateBaseLaser(idx + 5, m_Ranges[idx + 5]);
	//if(!isnormal(m_Ranges[idx - 5])) return false;	
	geometry_msgs::Point A2 = CalcCoordinateBaseLaser(idx - 5, m_Ranges[idx - 5]);	
	k =  (A1.x != A2.x ?  (A1.y - A2.y) / (A1.x - A2.x) : 500);

	//======================
#if(0)
	//printf("GetWallPtA theta = %lf, m_Lfd = %f, m_KeepDistance = %f, idx = %d, m_Ranges[idx] = %lf, ptA = (%lf, %lf),  A1=(%lf, %lf), A2 = (%lf, %lf), k = %lf, alpha = %lf\n ",\
	theta, m_Lfd, m_KeepDistance, idx, m_Ranges[idx], ptA.x, ptA.y, A1.x, A1.y, A2.x, A2.y, k, atan(k) * 180/M_PI);
	vector<geometry_msgs::Point> pts;
	pts.push_back(A1); pts.push_back(ptA); pts.push_back(A2);
	ShowLine(pts, m_ALinePub);
#endif
	//======================
	return true;
}
/*
说明：ranges[idx] 无效，选择一个合适的扫描范围内的点，进行拟合，在拟合的直线上选择点A和斜率K.
拟合算法为RANSAC， 直线参数保存在lineParameters，其确定的直线方程为:
y = (- lineParameters[0]/ lineParameters[1]  ) * (x -  lineParameters[2]) + lineParameters[3];
*/
bool TraceCtrl::GetPtAFromRansac(int idx, geometry_msgs::Point & ptA, double &k) const
{
 // (m_WallPos == LEFT_SIDE) ? (269 - static_cast<int> (theta + 0.5)) : (89 + static_cast<int> (theta + 0.5));
	// 直线参数计算结果
	std::vector<double> lineParameters;
	// 小于2cm作为聚类的标准
	LineParamEstimator lpEstimator(0.02); //for a point to be on the line it has to be closer than 0.5 units from the line
	// 参与估算的数据点
	std::vector<Point2D> pointData;
	// 确定直线参数所需最小点的个数
	int numForEstimate = 2;
	// 选择数据点的范围
	int start_idx, end_idx;
	if(m_WallPos == LEFT_SIDE)
	{
		start_idx = 185;
		end_idx = 265;
	}
	else
	{
		start_idx = 95;
		end_idx = 175;
	}
	// 选择数据点
	geometry_msgs::Point  pt;
	for(int i =  start_idx; i < end_idx; i++)
	{
		if(!isnormal(m_Ranges[i])) continue;
		pt = CalcCoordinateBaseLaser(i, m_Ranges[i]);
		Point2D pt2D(pt.x, pt.y);
		pointData.push_back(pt2D);
	}
	if(pointData.size() < 3) return false;
	// 估算直线参数
	MyRANSAC myRansac;
	double usedData =myRansac.compute(lineParameters, 
									&lpEstimator , 
									pointData, 
									numForEstimate);
	k = - lineParameters[0]/ lineParameters[1] ;
	double b = 	lineParameters[3] - k *  lineParameters[2];
	// 过O-m_Ranges[idx]的直线与拟合直线的交点A
	double k2 = tan(idx * M_PI / 180 - M_PI);
	ptA.x = b / (k2 -k);
	ptA.y = k2 * b / (k2 -k);

	//======================
#if(0)
	geometry_msgs::Point  A1;
	geometry_msgs::Point  A2;
	k2 = tan((idx + 10) * M_PI / 180 - M_PI);
	A1.x = b / (k2 -k);
	A1.y = k2 * b / (k2 -k);
	k2 = tan((idx - 10) * M_PI / 180 - M_PI);	
	A2.x = b / (k2 -k);
	A2.y = k2 * b / (k2 -k);	
	//printf("GetWallPtA theta = %lf, m_Lfd = %f, m_KeepDistance = %f, idx = %d, m_Ranges[idx] = %lf, ptA = (%lf, %lf),  A1=(%lf, %lf), A2 = (%lf, %lf), k = %lf, alpha = %lf\n ",\
	theta, m_Lfd, m_KeepDistance, idx, m_Ranges[idx], ptA.x, ptA.y, A1.x, A1.y, A2.x, A2.y, k, atan(k) * 180/M_PI);
	vector<geometry_msgs::Point> pts;
	pts.push_back(A1); pts.push_back(ptA); pts.push_back(A2);
	ShowLine(pts, m_ALinePub);
#endif
	//======================	
	return true;
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

返回：返回true，表示存在两个解，两个点的坐标，(x1,y1)(x2,y2)；
	返回false，表示没有解。
*/
bool TraceCtrl::getDistancePoint(double xa, double ya, double ka, double s, double *x1, double *y1, double *x2, double *y2) const
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
ptA : m
m_Lfd : cm

说明： 车辆姿态与墙面切线不平行且A点到车辆距离小于前视距离时的情况
在A点前方选择A2点，使A2到A的距离为m_Lfd
*/
bool TraceCtrl::GetWallPtA2(geometry_msgs::Point & ptA, double &k) const
{
	double x1, y1, x2, y2;
	if(!getDistancePoint(ptA.x, ptA.y, k, m_Lfd * 0.01, &x1, &y1, &x2, &y2)) return false;
	//printf("GetWallPtA2 x = %lf , y = %lf ", ptA.x, ptA.y);
	if(x1 > x2)
	{
		ptA.x = x1;
		ptA.y = y1;
	}
	else
	{
		ptA.x = x2;
		ptA.y = y2;
	}
	//printf(" ptA.x = %lf , ptA.y = %lf , x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf\n", ptA.x, ptA.y, x1, y1, x2, y2);
	return true;
}
/*
说明：根据A点坐标和斜率，发布A点切线
*/
void TraceCtrl::ShowALine(const geometry_msgs::Point &ptA, const double &k) const
{
	geometry_msgs::Point  A1;
	geometry_msgs::Point  A2;
	double s, x1, y1, x2, y2;
	// m
	s = 0.5; 
	getDistancePoint(ptA.x, ptA.y, k, s, &x1, &y1, &x2, &y2);
	A1.x = x1;
	A1.y = y1;
	A2.x = x2;
	A2.y = y2;

	vector<geometry_msgs::Point> pts;
	pts.push_back(A1); pts.push_back(ptA); pts.push_back(A2);
	ShowLine(pts, m_ALinePub);
}

/*
说明：转换曲率到转角
*/
double TraceCtrl::convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
	return atan(wheel_base * kappa);
}
/*
说明：计算从当前位置到B点的转向角，单位度
小于0时右转
*/
double TraceCtrl::CalcSteerAngle(const geometry_msgs::Pose &curpose, const geometry_msgs::Point &ptB) const
{
	return atan(ptB.y/ptB.x) * 180 / M_PI;
}
/*
参数：
steerAngle：从当前位置到B点的转向角，度

说明：PID计算以B点为目标点，以转向角为误差的曲率
*/
bool TraceCtrl::CalcPIDCurvature(const double &steerAngle, double *output_kappa)
{
	static bool bLastinputInited = false;
	m_Input = steerAngle;
/*
	if(bLastinputInited == false)
	{
		m_PID.InitLastInput(m_Input);
		bLastinputInited = true;
	}*/
	if(!m_PID.Compute3()) return false;
	*output_kappa = m_Output;
	return true;
}
/*
说明：初始化PID参数
  kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

   SampleTimeInSec默认 0.1s
   确认 kd 与 m_KD的差值 
*/
void TraceCtrl::InitPID()
{
	m_Setpoint = 0;
	m_Input = 0;
	m_Output = 0;
	// 比例系数 放大
	//m_Kp = 0.024;
	// 积分系数 静差
	//m_Ki = 0.01; 
	// 微分系数 防超调
	//m_Kd = 0.001;  
	m_PID.Init(&m_Input, &m_Output, &m_Setpoint, m_Kp, m_Ki, m_Kd, DIRECT);
	m_PID.InitLastInput(m_Input);
	m_PID.SetMode(AUTOMATIC);
	m_PID.SetOutputLimits(-1, 1);
}
void TraceCtrl::publishDiognostic()
{
#ifdef DIAGNOSTIC_PUB
	m_DiagPub.publish(m_DiagStat);
#endif
}
bool TraceCtrl::Init()
{
	InitPID();
	return true;
}
void TraceCtrl::Run()
{
	ros::NodeHandle n;
	ros::NodeHandle private_node("~");
	private_node.param<double>("fld", m_Lfd, 100);
	private_node.param<double>("keep_distance", m_KeepDistance, 50);
	private_node.param<int>("wall_position", m_WallPos, 1);
	private_node.param<double>("wheel_base", m_L, 2.5);
	private_node.param<double>("calc_control_period", m_ClacCtrlPeriod, 0.5);
	private_node.param<double>("keep_speed", m_KeepSpeed, 1);
	private_node.param<double>("min_keep_speed", m_MinKeepSpeed, 0.1);
	private_node.param<string>("frame_id", m_FrameID, "laser");
	private_node.param<int>("method",  m_Method, 1);
	// 速度滤波
	double lateral_accel_limit,  lowpass_gain_linear,  lowpass_gain_angular;
	private_node.param<bool>("is_twist_filter", m_IsFilter, "false");
	private_node.param<double>("lateral_accel_limit", lateral_accel_limit, 5.0);
	private_node.param<double>("lowpass_gain_linear", lowpass_gain_linear, 0.4);
	private_node.param<double>("lowpass_gain_angular", lowpass_gain_angular, 0.4);
	m_TwistFilter.Set( lateral_accel_limit,  lowpass_gain_linear,  lowpass_gain_angular);

	// pid
	private_node.param<double>("kp", m_Kp, 0.024);
	private_node.param<double>("ki", m_Ki, 0.01);
	private_node.param<double>("kd", m_Kd, 0.001);


	printf("fld = %lf, keep_distance = %lf, wall_pos = %d, wheel_base = %lf, period = %lf, speed = %lf, frame=%s, m_isfilter = %d\n",\
	m_Lfd, m_KeepDistance, m_WallPos, m_L, m_ClacCtrlPeriod, m_KeepSpeed, m_FrameID.c_str(), m_IsFilter ? 1 :0);

	if(Init())
	{
	// 发布跟踪结果
	m_TwistPub = n.advertise <geometry_msgs::Twist>("/cmd_vel", 10);
	// 订阅激光雷达/scan
	ros::Subscriber param_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TraceCtrl::LaserscanCallback,this);
	// 定时计算
	ros::Timer param_pub_timer = n.createTimer(ros::Duration(m_ClacCtrlPeriod), &TraceCtrl::TrackCalcCallback, this);
	// 发布Marker
	m_MakerPub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
	// 发布诊断信息
	m_DiagPub = n.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic_status", 10);
	// 发布曲线
	m_LinePub = n.advertise<nav_msgs::Path>("visualization_path", 10);
	m_ALinePub = n.advertise<nav_msgs::Path>("A_Line", 10);
	m_ABLinePub = n.advertise<nav_msgs::Path>("A_B_Line", 10);
	m_OBCirclePub = n.advertise<nav_msgs::Path>("O_B_Circle", 10);
	ros::spin();
	}
	return;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "track_node");
	TraceCtrl ctrl;
	ctrl.Run();
	return 0;
}
