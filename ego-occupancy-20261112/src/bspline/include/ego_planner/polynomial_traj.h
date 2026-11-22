#pragma once
#include <Eigen/Core>
namespace ego_planner {
/// 单段五次多项式，用归一化时间减少不同持续时间导致的病态系数。
class PolynomialTraj {
 public:
  static PolynomialTraj Quintic(const Eigen::Vector3d& p0,const Eigen::Vector3d& v0,
    const Eigen::Vector3d& a0,const Eigen::Vector3d& p1,const Eigen::Vector3d& v1,
    const Eigen::Vector3d& a1,double duration);
  Eigen::Vector3d evaluate(double t,int derivative_order=0) const;
  double duration() const { return duration_; }
 private:
  PolynomialTraj(Eigen::Matrix<double,3,6> coefficients,double duration);
  Eigen::Matrix<double,3,6> coefficients_;
  double duration_;
};
}
