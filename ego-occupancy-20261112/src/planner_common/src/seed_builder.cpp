#include <ego_planner/seed_builder.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ego_planner {
namespace {
using Coefficients = Eigen::Matrix<double, 6, 1>;
Coefficients Coeff(double p0, double v0, double a0, double p1, double v1, double a1, double t) {
  Coefficients c;
  c[0] = p0;
  c[1] = v0 * t;
  c[2] = 0.5 * a0 * t * t;
  const double p = p1 - c[0] - c[1] - c[2];
  const double v = v1 * t - c[1] - 2 * c[2];
  const double a = a1 * t * t - 2 * c[2];
  c[3] = 10 * p - 4 * v + 0.5 * a;
  c[4] = -15 * p + 7 * v - a;
  c[5] = 6 * p - 3 * v + 0.5 * a;
  return c;
}
}  // namespace

ReferenceTrajectory ReferenceTrajectory::MinimumJerk(const PlanningRequest& r, double duration) {
  if (!std::isfinite(duration) || duration <= 0 || duration > 600 || r.reference_points.size() > 7)
    throw std::invalid_argument("invalid reference duration/count");
  std::vector<Vec3> positions{r.start.position};
  positions.insert(positions.end(), r.reference_points.begin(), r.reference_points.end());
  positions.push_back(r.goal.position);
  const int count = static_cast<int>(positions.size()) - 1;
  std::vector<double> times(count);
  double length = 0;
  for (int i = 0; i < count; ++i) {
    times[i] = (positions[i + 1] - positions[i]).norm();
    if (!std::isfinite(times[i]) || times[i] < 1e-5)
      throw std::invalid_argument("degenerate reference segment");
    length += times[i];
  }
  // 给短段留下最小时长，避免极小距离造成求解病态。
  for (auto& t : times) t = duration * (0.2 / count + 0.8 * t / length);
  std::vector<Vec3> velocities(count + 1, Vec3::Zero()), accelerations(count + 1, Vec3::Zero());
  velocities.front() = r.start.velocity;
  velocities.back() = r.goal.velocity;
  accelerations.front() = r.start.acceleration;
  accelerations.back() = r.goal.acceleration;
  const int variables = 2 * (count - 1);
  if (variables > 0) {
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(variables, variables);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(variables, 3);
    for (int s = 0; s < count; ++s) {
      Eigen::Matrix<double, 6, 6> q = Eigen::Matrix<double, 6, 6>::Zero();
      for (int i = 3; i < 6; ++i)
        for (int j = 3; j < 6; ++j)
          q(i, j) = static_cast<double>(i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2)) /
                    ((i + j - 5) * std::pow(times[s], 5));
      Eigen::MatrixXd d = Eigen::MatrixXd::Zero(6, variables);
      if (s > 0) {
        d.col(2 * (s - 1)) = Coeff(0, 1, 0, 0, 0, 0, times[s]);
        d.col(2 * (s - 1) + 1) = Coeff(0, 0, 1, 0, 0, 0, times[s]);
      }
      if (s + 1 < count) {
        d.col(2 * s) = Coeff(0, 0, 0, 0, 1, 0, times[s]);
        d.col(2 * s + 1) = Coeff(0, 0, 0, 0, 0, 1, times[s]);
      }
      Eigen::Matrix<double, 6, 3> base;
      for (int axis = 0; axis < 3; ++axis)
        base.col(axis) = Coeff(positions[s][axis], velocities[s][axis], accelerations[s][axis],
                               positions[s + 1][axis], velocities[s + 1][axis],
                               accelerations[s + 1][axis], times[s]);
      h.noalias() += d.transpose() * q * d;
      b.noalias() -= d.transpose() * q * base;
    }
    const auto decomposition = h.ldlt();
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
      throw std::runtime_error("minimum-jerk reference is singular");
    const Eigen::MatrixXd x = decomposition.solve(b);
    if (!x.allFinite() || (h * x - b).norm() > 1e-7 * std::max(1.0, b.norm()))
      throw std::runtime_error("minimum-jerk reference residual too large");
    for (int i = 1; i < count; ++i) {
      velocities[i] = x.row(2 * (i - 1));
      accelerations[i] = x.row(2 * (i - 1) + 1);
    }
  }
  ReferenceTrajectory result;
  for (int i = 0; i < count; ++i)
    result.segments_.push_back(
        PolynomialTraj::Quintic(positions[i], velocities[i], accelerations[i], positions[i + 1],
                                velocities[i + 1], accelerations[i + 1], times[i]));
  return result;
}

double ReferenceTrajectory::duration() const {
  double sum = 0;
  for (const auto& segment : segments_) sum += segment.duration();
  return sum;
}
Vec3 ReferenceTrajectory::evaluate(double time, int order) const {
  if (!std::isfinite(time) || segments_.empty())
    throw std::invalid_argument("invalid reference time");
  time = std::clamp(time, 0.0, duration());
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    if (time <= segments_[i].duration() || i + 1 == segments_.size())
      return segments_[i].evaluate(time, order);
    time -= segments_[i].duration();
  }
  throw std::logic_error("empty reference");
}

void enforceBoundary(Eigen::MatrixXd& q, double dt, const BoundaryState& s,
                     const BoundaryState& e) {
  if (q.rows() != 3 || q.cols() < 8 || !q.allFinite() || !std::isfinite(dt) || dt <= 0 ||
      !s.position.allFinite() || !s.velocity.allFinite() || !s.acceleration.allFinite() ||
      !e.position.allFinite() || !e.velocity.allFinite() || !e.acceleration.allFinite())
    throw std::invalid_argument("invalid exact spline boundary");
  // p=(q0+4q1+q2)/6, v=(q2-q0)/(2dt), a=(q0-2q1+q2)/dt²。
  q.col(0) = s.position - dt * s.velocity + dt * dt * s.acceleration / 3;
  q.col(1) = s.position - dt * dt * s.acceleration / 6;
  q.col(2) = s.position + dt * s.velocity + dt * dt * s.acceleration / 3;
  const auto n = q.cols();
  q.col(n - 3) = e.position - dt * e.velocity + dt * dt * e.acceleration / 3;
  q.col(n - 2) = e.position - dt * dt * e.acceleration / 6;
  q.col(n - 1) = e.position + dt * e.velocity + dt * dt * e.acceleration / 3;
  if (!q.allFinite()) throw std::overflow_error("spline boundary overflow");
}

double uniformInterval(const Bspline& spline) {
  if (spline.degree() != 3 || spline.controlPoints().cols() < 8)
    throw std::invalid_argument("planner requires cubic spline with >=8 control points");
  const double dt = spline.knownUniformInterval();
  if (!std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("planner requires constructor-known uniform knots");
  return dt;
}

Seed SeedBuilder::build(const PlanningRequest& r, const PlanningConfig& c, double scale) const {
  validateRequest(r, c);
  if (!std::isfinite(scale) || scale < 1 || scale > 100)
    throw std::invalid_argument("invalid seed duration scale");
  double length = 0;
  Vec3 previous = r.start.position;
  for (const auto& p : r.reference_points) {
    length += (p - previous).norm();
    previous = p;
  }
  length += (r.goal.position - previous).norm();
  const double duration = std::max(2.0, 1.8 * length / c.nominal_speed) * scale;
  if (duration > c.max_duration) throw std::invalid_argument("seed duration exceeds limit");
  auto reference = ReferenceTrajectory::MinimumJerk(r, duration);
  // 极小间距可令除法溢出为Inf；先在浮点域截断，再进行有界整数转换。
  const double requested_spans = std::ceil(length / c.control_point_spacing);
  const int spans = static_cast<int>(
      std::clamp(requested_spans, 5.0, static_cast<double>(c.max_control_points) - 3.0));
  const double dt = duration / spans;
  std::vector<Vec3> samples;
  for (int i = 0; i <= spans; ++i) samples.push_back(reference.evaluate(i * dt));
  const std::array<Vec3, 4> boundary = {r.start.velocity, r.goal.velocity, r.start.acceleration,
                                        r.goal.acceleration};
  auto initial = Bspline::Parameterize(samples, boundary, dt);
  Eigen::MatrixXd points = initial.controlPoints();
  enforceBoundary(points, dt, r.start, r.goal);
  return {std::move(reference), Bspline(std::move(points), 3, dt), dt};
}

}  // namespace ego_planner
