#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include <Eigen/Dense>

typedef struct traj_state {
  double t;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d angular_speed;
  Eigen::Vector3d acc;
  Eigen::Vector3d a_rot;
  Eigen::Matrix<double, 4, 1> motor;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} traj_state;

namespace Trajectory {
template <class T>
class Path {
 public:
  Path();

  T generate(double time);

 private:
};

class circle_generator {
 public:
  circle_generator(double radius, double speed, double dt)
      : radius_(radius), speed_(speed), dt_(dt) {};

  Eigen::Vector3d pos(double t);

  Eigen::Vector3d vel(double t);

  Eigen::Vector3d theta(double t);

  Eigen::Vector3d omega(double t);

  Eigen::Vector3d thrust(double t);

  Eigen::Matrix<double, 4, 1> inputfm(double t);

  Eigen::Matrix<double, 4, 1> input(double t);

 private:
  double radius_;
  double speed_;
  double dt_;

  Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81);
};

class cir_conacc_generator {
 public:
  cir_conacc_generator(double radius, double max_speed, double acc, double dt)
      : radius_(radius), max_speed_(max_speed), acc_(acc), dt_(dt) {};

  double angle(double t);

  Eigen::Vector3d pos(double t);

  Eigen::Vector3d vel(double t);

  Eigen::Vector3d theta(double t);

  Eigen::Vector3d omega(double t);

  Eigen::Vector3d thrust(double t);

  Eigen::Matrix<double, 4, 1> inputfm(double t);

  Eigen::Matrix<double, 4, 1> input(double t);

 private:
  double radius_;
  double max_speed_;
  double acc_;
  double dt_;

  Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81);
};

}  // namespace Trajectory

#endif