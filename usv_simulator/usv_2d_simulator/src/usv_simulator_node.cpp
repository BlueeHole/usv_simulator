/*******************************************************************
 * @Descripttion   : 
 * @version        : 
 * @Author         : Murphy
 * @Date           : 2024-05-10 21:30
 * @LastEditTime   : 2024-05-12 13:12
*******************************************************************/
#include <nav_msgs/Odometry.h>
#include <util_msgs/SO2Command.h>
#include <Eigen/Geometry>
// #include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>
#include <usv_simulator/USV.h>

typedef struct _Control {
  double tau[3];
} Control;

typedef struct _Command {
  float tau[3];
  float k[3];
} Command;

typedef struct _Disturbance {
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;

static Command command;
static Disturbance disturbance;

void stateToOdomMsg(const USVSimulator::USV::State& state,
                    nav_msgs::Odometry& odom);
void quadToImuMsg(const USVSimulator::USV& quad, sensor_msgs::Imu& imu);

static Control getControl(const USVSimulator::USV& usv, const Command& cmd) {
  /* 四旋翼中，输入的控制量是电机转速，因此这里需要由力和力矩解算转速。
     但我的船模型方程里直接是力和力矩（没做物理仿真），因此不需要解算*/
  Control control = {cmd.tau[0], cmd.tau[1], cmd.tau[2]};
  return control;
}

static void cmd_callback(const util_msgs::SO2Command::ConstPtr& cmd) {
  command.tau[0] = cmd->tau.x;
  command.tau[1] = cmd->tau.y;
  command.tau[2] = cmd->tau.z;  // 力矩
  ROS_WARN("cmd_callback: tau.x = %f, tau.y = %f, tau.z = %f", command.tau[0],
           command.tau[1], command.tau[2]);
}

static void force_disturbance_callback(
    const geometry_msgs::Vector3::ConstPtr& f) {
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
}

static void moment_disturbance_callback(
    const geometry_msgs::Vector3::ConstPtr& m) {
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}

/* 仿真器：订阅cmd, force_disturbance, moment_disturbance，发布odom和imu
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "usv_simulator_so2");

  ros::NodeHandle n("~");

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  // 会被映射为 /so3_cmd
  ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback,
                                        ros::TransportHints().tcpNoDelay());
  ros::Subscriber f_sub =
      n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                  ros::TransportHints().tcpNoDelay());
  ros::Subscriber m_sub =
      n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                  ros::TransportHints().tcpNoDelay());

  USVSimulator::USV usv;
  double _init_x, _init_y, _init_yaw;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_yaw", _init_yaw, 0.0);

  Eigen::Vector3d pose = Eigen::Vector3d(_init_x, _init_y, _init_yaw);
  usv.setStatePose(pose);

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1 / odom_rate);

  std::string usv_name;
  n.param("usv_name", usv_name, std::string("usv"));

  USVSimulator::USV::State state = usv.getState();

  ros::Rate r(simulation_rate);
  const double dt = 1 / simulation_rate;

  Control control;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/world";
  odom_msg.child_frame_id = "/" + usv_name;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "/simulator";

  /*
  command.force[0] = 0;
  command.force[1] = 0;
  command.force[2] = quad.getMass()*quad.getGravity() + 0.1;
  command.qx = 0;
  command.qy = 0;
  command.qz = 0;
  command.qw = 1;
  command.kR[0] = 2;
  command.kR[1] = 2;
  command.kR[2] = 2;
  command.kOm[0] = 0.15;
  command.kOm[1] = 0.15;
  command.kOm[2] = 0.15;
  */

  ros::Time next_odom_pub_time = ros::Time::now();
  while (n.ok()) {
    ros::spinOnce();

    auto last = control;
    control = getControl(usv, command);
    for (int i = 0; i < 3; ++i) {
      //! @bug might have nan when the input is legal
      if (std::isnan(control.tau[i]))
        control.tau[i] = last.tau[i];
    }
		// ROS_WARN("while control: tau.x = %f, tau.y = %f, tau.z = %f", control.tau[0],
		// 			 control.tau[1], control.tau[2]);
    usv.setInput(control.tau[0], control.tau[1], control.tau[2]);
    usv.setExternalForce(disturbance.f);
    usv.setExternalMoment(disturbance.m);
    usv.step(dt);

    ros::Time tnow = ros::Time::now();

    if (tnow >= next_odom_pub_time) {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state = usv.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(usv, imu);
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu);
    }

    r.sleep();
  }

  return 0;
}

void stateToOdomMsg(const USVSimulator::USV::State& state,
                    nav_msgs::Odometry& odom) {
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = 0;

  auto angle = state.x(2);
  Eigen::Matrix3d rot;
  // clang-format off
  rot << cos(angle), -sin(angle), 0,
                     sin(angle), cos(angle),  0,
										 0,										0,  1;
  // clang-format on
  // 将旋转矩阵转换为四元数
  Eigen::Quaterniond q(rot);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = state.v(2);
}

void quadToImuMsg(const USVSimulator::USV& usv, sensor_msgs::Imu& imu)

{
  USVSimulator::USV::State state = usv.getState();
  auto angle = state.x(2);
  Eigen::Matrix3d rot;
  // clang-format off
  rot << cos(angle), -sin(angle), 0,
				 sin(angle), cos(angle),  0,
				0,										0,  1;
  // clang-format on
  // 将旋转矩阵转换为四元数
  Eigen::Quaterniond q(rot);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.v(0);
  imu.angular_velocity.y = state.v(1);
  imu.angular_velocity.z = 0;

  imu.linear_acceleration.x = usv.getAcc()[0];
  imu.linear_acceleration.y = usv.getAcc()[1];
  imu.linear_acceleration.z = 0;
}
