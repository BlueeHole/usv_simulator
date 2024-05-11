/*******************************************************************
 * @Descripttion   : 
 * @version        : 
 * @Author         : Murphy
 * @Date           : 2024-05-10 15:57
 * @LastEditTime   : 2024-05-10 21:25
*******************************************************************/
#ifndef __USV_SIMULATOR_USV_H__
#define __USV_SIMULATOR_USV_H__

#include <Eigen/Core>
#include <boost/array.hpp>

namespace USVSimulator
{

class USV
{
public:
  struct State
  {
    Eigen::Vector3d x;	// [x, y, phi], 				world frame
    Eigen::Vector3d v;	// [u, v, r], 					boat frame
    Eigen::Vector3d tau;	// [tau_u, tau_v, tau_r]		boat frame
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  USV();

  const USV::State& getState(void) const;

  void setState(const USV::State& state);

  void setStatePose(const Eigen::Vector3d& Pose);

  const Eigen::Vector3d& getExternalForce(void) const;
  void setExternalForce(const Eigen::Vector3d& force);

  const Eigen::Vector3d& getExternalMoment(void) const;
  void setExternalMoment(const Eigen::Vector3d& moment);

  double getMotorTimeConstant(void) const;
  void setMotorTimeConstant(double k);

  void setInput(double u1, double u2, double u3);

  // Runs the actual dynamics simulation with a time step of dt
  void step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 9> InternalState;
  void operator()(const USV::InternalState& x,
                  USV::InternalState&       dxdt, const double /* t */);

  Eigen::Vector3d getAcc() const;

private:
  void updateInternalState(void);


  Eigen::Matrix3d M;
  Eigen::Matrix3d M_inv;
  Eigen::Matrix3d D;
  double          m11;
  double          m22;
  double          m33;
  double          d11;
  double          d22;
  double          d33;
  double		  max_tau[3], min_tau[3];
  
  double          motor_time_constant_; // unit: sec

  USV::State state_;

  Eigen::Vector3d acc_;

  Eigen::Vector3d  input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  InternalState internal_state_;
};
}
#endif
