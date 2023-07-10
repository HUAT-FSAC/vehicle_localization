/* *
 * 订阅/INS/ASENSING_INS消息;
 * 处理消息得到车辆位置和航线角等信息;
 * 发布话题/Carstate传递给cone_position处理锥筒坐标
 */

#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_Carstate.h"
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#define PI 3.14159265358979

void doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);
void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0,
                     double h0, double enu_xyz[3]);
// method defination goes to the header file

ros::Publisher INSpub;
ros::Publisher carPositionPub;

double enu_xyz[3];
double first_lat, first_lon, first_alt;
bool isFirstMsgReceived = false;

common_msgs::HUAT_Carstate Carstate;
common_msgs::HUAT_ASENSING my_ins;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "car_positon");
  ros::NodeHandle nh;

  // set up subscriber and publisher
  ros::Subscriber INSsub = nh.subscribe<common_msgs::HUAT_ASENSING>(
      "/INS/ASENSING_INS", 1, doINSMsg);
  ros::Rate rate(10);
  INSpub = nh.advertise<common_msgs::HUAT_Carstate>("/Carstate", 10);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// ref: https://ww2.mathworks.cn/help/map/ref/geodetic2enu.html
void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0,
                     double h0, double enu_xyz[3]) {
  // 定义一些常量，表示参考椭球体的参数
  double a, b, f, e_sq;
  a = 6378137;        // 长半轴，单位为米
  b = 6356752.3142;   // 短半轴，单位为米
  f = (a - b) / a;    // 扁率
  e_sq = f * (2 - f); // 第一偏心率平方

  // 计算站点（非原点）的ECEF坐标（地心地固坐标系）
  double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
  lamb = lat;                     // 纬度，单位为度
  phi = lon;                      // 经度，单位为度
  s = sin(lamb);                  // 纬度的正弦值
  N = a / sqrt(1 - e_sq * s * s); // 卯酉圈曲率半径

  sin_lambda = sin(lamb); // 纬度的正弦值
  cos_lambda = cos(lamb); // 纬度的余弦值
  sin_phi = sin(phi);     // 经度的正弦值
  cos_phi = cos(phi);     // 经度的余弦值

  x = (h + N) * cos_lambda * cos_phi;    // x坐标，单位为米
  y = (h + N) * cos_lambda * sin_phi;    // y坐标，单位为米
  z = (h + (1 - e_sq) * N) * sin_lambda; // z坐标，单位为米

  // 计算原点的ECEF坐标（地心地固坐标系）
  double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0,
      y0, z0;
  lamb0 = lat0;                      // 原点的纬度，单位为度
  phi0 = lon0;                       // 原点的经度，单位为度
  s0 = sin(lamb0);                   // 原点纬度的正弦值
  N0 = a / sqrt(1 - e_sq * s0 * s0); // 原点卯酉圈曲率半径

  sin_lambda0 = sin(lamb0); // 原点纬度的正弦值
  cos_lambda0 = cos(lamb0); // 原点纬度的余弦值
  sin_phi0 = sin(phi0);     // 原点经度的正弦值
  cos_phi0 = cos(phi0);     // 原点经度的余弦值

  x0 = (h0 + N0) * cos_lambda0 * cos_phi0;   // 原点x坐标，单位为米
  y0 = (h0 + N0) * cos_lambda0 * sin_phi0;   // 原点y坐标，单位为米
  z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0; // 原点z坐标，单位为米

  // 计算站点相对于原点的ECEF坐标差
  double xd, yd, zd, t;
  xd = x - x0;
  yd = y - y0;
  zd = z - z0;

  // 计算站点的ENU坐标（本地东北天坐标系）
  t = -cos_phi0 * xd - sin_phi0 * yd;

  enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd; // 东方向坐标，单位为米
  enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd; // 北方向坐标，单位为米
  enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd +
               sin_lambda0 * zd; // 天空方向坐标，单位为米

  // 更新车辆状态的x和y坐标，并打印出来
  Carstate.car_state.x = enu_xyz[0];
  Carstate.car_state.y = enu_xyz[1];
  std::cout << "车辆坐标 x= " << Carstate.car_state.x << std::endl;
  std::cout << "车辆坐标 y= " << Carstate.car_state.y << std::endl;
}

void doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msgs) {
  ROS_DEBUG("进入回调函数进行处理");

  // copy velocity values
  my_ins.east_velocity = msgs->east_velocity;
  my_ins.north_velocity = msgs->north_velocity;
  my_ins.ground_velocity = msgs->ground_velocity;

  // total velocity
  Carstate.V =
      sqrt(pow(my_ins.east_velocity, 2) + pow(my_ins.north_velocity, 2) +
           pow(my_ins.ground_velocity, 2));

  Carstate.car_state.theta =
      (msgs->azimuth) * PI / 180; // 后续需要考虑弧度还是角度

  if (!isFirstMsgReceived) {
    // initial msg circumstance
    first_lat = msgs->latitude;
    first_lon = msgs->longitude;
    first_alt = msgs->altitude;
    Carstate.car_state.x = 0;
    Carstate.car_state.y = 0;

    isFirstMsgReceived = true;
  } else {
    GeoDetic_TO_ENU((msgs->latitude) * PI / 180, (msgs->longitude) * PI / 180,
                    msgs->altitude, first_lat * PI / 180, first_lon * PI / 180,
                    first_alt, &enu_xyz[0]);
  }

  INSpub.publish(Carstate);
}
