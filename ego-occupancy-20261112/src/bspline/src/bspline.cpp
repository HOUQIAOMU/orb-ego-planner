// Adapted from EGO-Planner uniform_bspline.cpp, GPL-3.0; see planning/SOURCE.md.
#include <ego_planner/bspline.h>

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ego_planner {
Bspline::Bspline(Eigen::MatrixXd points, int degree, double interval)
    : points_(std::move(points)), degree_(degree), uniform_interval_(interval) {
  if (degree < 0 || degree > 3 || points_.rows() != 3 || points_.cols() <= degree ||
      points_.cols() > 4096 || !std::isfinite(interval) || interval <= 0)
    throw std::invalid_argument("invalid uniform spline geometry");
  knots_.resize(points_.cols() + degree + 1);
  for (Eigen::Index i = 0; i < knots_.size(); ++i) knots_[i] = (i - degree) * interval;
  validate();
}
Bspline::Bspline(Eigen::MatrixXd points, int degree, Eigen::VectorXd knots)
    : points_(std::move(points)), degree_(degree), knots_(std::move(knots)) {
  validate();
}
void Bspline::validate() const {
  if (degree_ < 0 || degree_ > 3 || points_.rows() != 3 || points_.cols() <= degree_ ||
      points_.cols() > 4096 || !points_.allFinite() ||
      knots_.size() != points_.cols() + degree_ + 1 || !knots_.allFinite())
    throw std::invalid_argument("invalid spline dimensions or nonfinite values");
  for (Eigen::Index i = 1; i < knots_.size(); ++i)
    if (knots_[i] < knots_[i - 1]) throw std::invalid_argument("knots not monotonic");
  const double span = knots_[points_.cols()] - knots_[degree_];
  if (!std::isfinite(span) || span <= 0) throw std::invalid_argument("empty spline time domain");
  if (degree_ > 0)
    for (Eigen::Index i = 0; i + 1 < points_.cols(); ++i)
      if (knots_[i + degree_ + 1] <= knots_[i + 1])
        throw std::invalid_argument("zero derivative knot span");
}
double Bspline::duration() const { return knots_[points_.cols()] - knots_[degree_]; }
Eigen::Vector3d Bspline::evaluate(double t) const {
  if (!std::isfinite(t)) throw std::invalid_argument("nonfinite spline time");
  const double u = knots_[degree_] + std::clamp(t, 0.0, duration());
  Eigen::Index k = degree_;
  while (k + 1 < points_.cols() && knots_[k + 1] <= u) ++k;
  std::array<Eigen::Vector3d, 4> work;
  for (int i = 0; i <= degree_; ++i) work[i] = points_.col(k - degree_ + i);
  // De Boor仅在局部degree+1个控制点上运算；不把控制点当作插值点。
  for (int level = 1; level <= degree_; ++level)
    for (int i = degree_; i >= level; --i) {
      const auto left = i + k - degree_;
      const double denominator = knots_[i + 1 + k - level] - knots_[left];
      const double alpha = denominator > 0 ? (u - knots_[left]) / denominator : 0.0;
      work[i] = (1 - alpha) * work[i - 1] + alpha * work[i];
    }
  if (!work[degree_].allFinite()) throw std::overflow_error("spline evaluation overflow");
  return work[degree_];
}
Bspline Bspline::derivative() const {
  if (degree_ == 0) {
    Eigen::VectorXd knots(2);
    knots << knots_[0], knots_[points_.cols()];
    return Bspline(Eigen::MatrixXd::Zero(3, 1), 0, std::move(knots));
  }
  Eigen::MatrixXd points(3, points_.cols() - 1);
  for (Eigen::Index i = 0; i < points.cols(); ++i)
    points.col(i) =
        degree_ * (points_.col(i + 1) - points_.col(i)) / (knots_[i + degree_ + 1] - knots_[i + 1]);
  return Bspline(std::move(points), degree_ - 1, knots_.segment(1, knots_.size() - 2).eval());
}
Bspline Bspline::scaledTime(double factor) const {
  if (!std::isfinite(factor) || factor <= 0) throw std::invalid_argument("invalid time scale");
  if (uniform_interval_ > 0) return Bspline(points_, degree_, uniform_interval_ * factor);
  auto knots = ((knots_.array() - knots_[degree_]) * factor + knots_[degree_]).matrix().eval();
  return Bspline(points_, degree_, std::move(knots));
}
std::pair<double, double> Bspline::normBounds() const {
  auto velocity = derivative(), acceleration = velocity.derivative();
  return {velocity.points_.colwise().norm().maxCoeff(),
          acceleration.points_.colwise().norm().maxCoeff()};
}
Bspline Bspline::Parameterize(const std::vector<Eigen::Vector3d>& samples,
                              const std::array<Eigen::Vector3d, 4>& boundary, double dt) {
  if (samples.size() < 4 || samples.size() > 256 || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(1 / (dt * dt)))
    throw std::invalid_argument("invalid parameterization size/time");
  for (const auto& p : samples)
    if (!p.allFinite()) throw std::invalid_argument("nonfinite sample");
  for (const auto& p : boundary)
    if (!p.allFinite()) throw std::invalid_argument("nonfinite boundary");
  const Eigen::Index count = static_cast<Eigen::Index>(samples.size());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(count + 4, count + 2);
  Eigen::MatrixXd b(count + 4, 3);
  for (Eigen::Index i = 0; i < count; ++i) {
    a.block<1, 3>(i, i) = Eigen::RowVector3d(1, 4, 1) / 6;
    b.row(i) = samples[i].transpose();
  }
  a.block<1, 3>(count, 0) = Eigen::RowVector3d(-1, 0, 1) / (2 * dt);
  a.block<1, 3>(count + 1, count - 1) = Eigen::RowVector3d(-1, 0, 1) / (2 * dt);
  a.block<1, 3>(count + 2, 0) = Eigen::RowVector3d(1, -2, 1) / (dt * dt);
  a.block<1, 3>(count + 3, count - 1) = Eigen::RowVector3d(1, -2, 1) / (dt * dt);
  for (int i = 0; i < 4; ++i) b.row(count + i) = boundary[i].transpose();
  // 保留原EGO的最小二乘语义；一般输入的端点不保证精确插值，后续仍需边界复核。
  auto qr = a.colPivHouseholderQr();
  if (qr.rank() != count + 2) throw std::runtime_error("rank deficient parameterization");
  Eigen::MatrixXd points = qr.solve(b).transpose();
  return Bspline(std::move(points), 3, dt);
}
}  // namespace ego_planner
