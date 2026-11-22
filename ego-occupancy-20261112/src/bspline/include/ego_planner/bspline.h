#pragma once
#include <Eigen/Core>
#include <array>
#include <utility>
#include <vector>
namespace ego_planner {
/// 控制点始终为3×N；evaluate参数为有效区间起点之后的秒数。
class Bspline {
 public:
  Bspline(Eigen::MatrixXd points, int degree, double interval);
  Bspline(Eigen::MatrixXd points, int degree, Eigen::VectorXd knots);
  Eigen::Vector3d evaluate(double t) const;
  Bspline derivative() const;
  Bspline scaledTime(double factor) const;
  std::pair<double, double> normBounds() const;  // 保守的三维速度/加速度范数上界。
  double duration() const;
  int degree() const { return degree_; }
  const Eigen::MatrixXd& controlPoints() const { return points_; }
  const Eigen::VectorXd& knots() const { return knots_; }
  /// >0仅表示由均匀构造器生成；显式节点不凭近似相等推断均匀性。
  double knownUniformInterval() const { return uniform_interval_; }
  /// 原EGO的最小二乘参数化；边界顺序v_start,v_end,a_start,a_end。
  static Bspline Parameterize(const std::vector<Eigen::Vector3d>& samples,
                              const std::array<Eigen::Vector3d, 4>& boundary, double dt);

 private:
  Eigen::MatrixXd points_;
  int degree_;
  Eigen::VectorXd knots_;
  double uniform_interval_ = 0.0;
  void validate() const;
};
}  // namespace ego_planner
