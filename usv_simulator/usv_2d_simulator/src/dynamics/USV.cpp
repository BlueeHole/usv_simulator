/*******************************************************************
 * @Descripttion   : 
 * @version        : 
 * @Author         : Murphy
 * @Date           : 2024-05-10 15:57
 * @LastEditTime   : 2024-05-11 19:01
*******************************************************************/
#include "usv_simulator/USV.h"
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>
#include "ode/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace USVSimulator {

USV::USV(void) {
  // Northern Clipper Boat Model
	double scale = 1e6;		//! 注意这里整体缩小了比例
  m11 = 5.3 * 1e6 / scale;
  m22 = 8.3 * 1e6 / scale;
  m33 = 3.7 * 1e9 / scale;
  d11 = 5.0 * 1e4 / scale;
  d22 = 2.7 * 1e5 / scale;
  d33 = 4.2 * 1e8 / scale;
  M << m11, 0, 0, 0, m22, 0, 0, 0, m33;
  D << d11, 0, 0, 0, d22, 0, 0, 0, d33;
  M_inv = M.inverse();
	max_tau[0] = 1e9 / scale;
	max_tau[1] = 1e9 / scale;
	max_tau[2] = 1e9 / scale;
	min_tau[0] = -1e9 / scale;
	min_tau[1] = -1e9 / scale;
	min_tau[2] = -1e9 / scale;

  state_.x = Eigen::Vector3d::Zero();
  state_.v = Eigen::Vector3d::Zero();
  state_.tau = Eigen::Vector3d::Zero();

  motor_time_constant_ = 1.0 / 30;

  external_force_.setZero();

  updateInternalState();

  input_ = Eigen::Vector3d::Zero();
}

void USV::step(double dt) {
  auto save = internal_state_;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  for (int i = 0; i < 9; ++i) {
    if (std::isnan(internal_state_[i])) {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 9; ++j) {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++) {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.tau(i) = internal_state_[6 + i];
  }

  updateInternalState();
}

void USV::operator()(const USV::InternalState& x, USV::InternalState& dxdt,
                     const double /* t */) {
  State cur_state;
  for (int i = 0; i < 3; i++) {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.tau(i) = x[6 + i];
  }
  Eigen::Vector3d x_dot, v_dot, tau_dot;
  Eigen::Matrix3d J_phi;
  J_phi << cos(cur_state.x(2)), -sin(cur_state.x(2)), 0, sin(cur_state.x(2)),
      cos(cur_state.x(2)), 0, 0, 0, 1;
  x_dot = J_phi * cur_state.v;
  // 水面无人艇运动控制及集群协调规划方法研究_秦梓荷
  Eigen::Matrix3d C;
  // clang-format off
  C << 0, 0, -m22 * cur_state.v(1), 
       0, 0, m11 * cur_state.v(0),
      m22 * cur_state.v(1), -m11 * cur_state.v(0), 0;
  // clang-format on
  v_dot = M_inv * (-C * cur_state.v - D * cur_state.v + cur_state.tau +
                   external_force_);
  acc_ = v_dot;

  tau_dot = (input_ - cur_state.tau) / motor_time_constant_;
	ROS_INFO("input: %f %f %f", input_(0), input_(1), input_(2));

  for (int i = 0; i < 3; i++) {
    dxdt[0 + i] = x_dot(i);
    dxdt[3 + i] = v_dot(i);
    dxdt[6 + i] = tau_dot(i, 0);
  }

  for (int i = 0; i < 9; ++i) {
    if (std::isnan(dxdt[i])) {
      dxdt[i] = 0;
      std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

void USV::setInput(double u1, double u2, double u3) {
  input_(0) = u1;
  input_(1) = u2;
  input_(2) = u3;
  for (int i = 0; i < 3; i++) {
    if (std::isnan(input_(i))) {
      input_(i) = (max_tau[i] + min_tau[i]) / 2;
      std::cout << "NAN input ";
    }
    if (input_(i) > max_tau[i])
      input_(i) = max_tau[i];
    else if (input_(i) < min_tau[i])
      input_(i) = min_tau[i];
  }
}

const USV::State& USV::getState(void) const {
  return state_;
}
void USV::setState(const USV::State& state) {
  state_.x = state.x;
  state_.v = state.v;
  state_.tau = state.tau;

  updateInternalState();
}

void USV::setStatePose(const Eigen::Vector3d& Pose) {
  state_.x = Pose;

  updateInternalState();
}

double USV::getMotorTimeConstant(void) const {
  return motor_time_constant_;
}
void USV::setMotorTimeConstant(double k) {
  if (k <= 0) {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }
  motor_time_constant_ = k;
}

const Eigen::Vector3d& USV::getExternalForce(void) const {
  return external_force_;
}
void USV::setExternalForce(const Eigen::Vector3d& force) {
  external_force_ = force;
}

const Eigen::Vector3d& USV::getExternalMoment(void) const {
  return external_moment_;
}
void USV::setExternalMoment(const Eigen::Vector3d& moment) {
  external_moment_ = moment;
}

void USV::updateInternalState(void) {
  for (int i = 0; i < 3; i++) {
    internal_state_[0 + i] = state_.x(i);
    internal_state_[3 + i] = state_.v(i);
    internal_state_[6 + i] = state_.tau(i);
  }
}

Eigen::Vector3d USV::getAcc() const {
  return acc_;
}
}  // namespace USVSimulator
