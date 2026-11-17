#include <ego_planner/polynomial_traj.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace ego_planner {
PolynomialTraj::PolynomialTraj(Eigen::Matrix<double,3,6> coefficients,double duration)
    : coefficients_(std::move(coefficients)),duration_(duration) {
  if(!coefficients_.allFinite()) throw std::overflow_error("polynomial coefficients overflow");
}
PolynomialTraj PolynomialTraj::Quintic(
    const Eigen::Vector3d& p0,const Eigen::Vector3d& v0,const Eigen::Vector3d& a0,
    const Eigen::Vector3d& p1,const Eigen::Vector3d& v1,const Eigen::Vector3d& a1,double t) {
  if(!p0.allFinite() || !v0.allFinite() || !a0.allFinite() ||
     !p1.allFinite() || !v1.allFinite() || !a1.allFinite() || !std::isfinite(t) || t<=0)
    throw std::invalid_argument("invalid quintic boundary or duration");
  Eigen::Matrix<double,3,6> c;
  c.col(0)=p0;c.col(1)=v0*t;c.col(2)=a0*(.5*t*t);
  const Eigen::Vector3d dp=p1-c.col(0)-c.col(1)-c.col(2);
  const Eigen::Vector3d dv=v1*t-c.col(1)-2*c.col(2);
  const Eigen::Vector3d da=a1*(t*t)-2*c.col(2);
  c.col(3)=10*dp-4*dv+.5*da;
  c.col(4)=-15*dp+7*dv-da;
  c.col(5)=6*dp-3*dv+.5*da;
  return PolynomialTraj(std::move(c),t);
}
Eigen::Vector3d PolynomialTraj::evaluate(double t,int order) const {
  if(!std::isfinite(t) || order<0 || order>3) throw std::invalid_argument("invalid polynomial evaluation");
  const double u=std::clamp(t,0.0,duration_)/duration_;
  Eigen::Vector3d result=Eigen::Vector3d::Zero();
  for(int i=5;i>=order;--i) {
    double factor=1;
    for(int j=0;j<order;++j) factor*=i-j;
    result=result*u+factor*coefficients_.col(i);
  }
  result/=std::pow(duration_,order);
  if(!result.allFinite()) throw std::overflow_error("polynomial evaluation overflow");
  return result;
}
}  // namespace ego_planner
