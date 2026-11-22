#include "online_planning_session.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include "planning_io.h"
#include "planning_state_adapter.h"

namespace ego_planner::app {
namespace {
std::string Json(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else
      out << c;
  }
  out << '"';
  return out.str();
}
void Stamp(std::ostream& out, const ego_occupancy::Metadata& m) {
  out << "{\"source\":[" << m.source.session << ',' << m.source.map << ',' << m.source.revision
      << "],\"epoch\":" << m.epoch << ",\"sequence\":" << m.sequence << '}';
}
std::vector<Vec3> Sample(const Bspline& curve) {
  std::vector<Vec3> points;
  points.reserve(101);
  for (int i = 0; i <= 100; ++i) points.push_back(curve.evaluate(curve.duration() * i / 100.));
  return points;
}
}  // namespace
OnlinePlanningSession::OnlinePlanningSession(ego_occupancy::MapReader reader, PlanningConfig config,
                                             const std::filesystem::path& directory, double start,
                                             double goal)
    : config_(std::move(config)), directory_(directory), start_m_(start), goal_m_(goal) {
  if (!std::filesystem::create_directory(directory_))
    throw std::invalid_argument("planning output must be new");
  events_.open(directory_ / "events.jsonl");
  events_.exceptions(std::ios::failbit | std::ios::badbit);
  runtime_ = std::make_unique<PlanningRuntime>(
      std::move(reader), config_, RuntimeOptions{}, [this](const RuntimeEvent& e) {
        try {
          recordEvent(e);
        } catch (const std::exception& error) {
          throw std::runtime_error("planning output " + directory_.string() + ": " + error.what());
        }
      });
}
OnlinePlanningSession::~OnlinePlanningSession() { stop(); }
void OnlinePlanningSession::update(const ego_pose_bridge::Record& pose, std::uint64_t id,
                                   Clock::time_point received, bool bound) {
  runtime_->updateState(cameraState(pose, id, received, bound));
}
bool OnlinePlanningSession::trigger() { return runtime_->triggerProbe(start_m_, goal_m_); }
void OnlinePlanningSession::clear() { runtime_->clear(); }
std::string OnlinePlanningSession::error() const { return runtime_->error(); }
void OnlinePlanningSession::stop() {
  if (runtime_) runtime_->stop();
}
ego_occupancy::PlanningLayer OnlinePlanningSession::layer() const {
  const auto v = runtime_->view();
  ego_occupancy::PlanningLayer layer;
  layer.has_task = v.has_task;
  layer.ready = v.validity == TrajectoryValidity::Ready;
  layer.status = std::string(toString(v.validity)) + " / " + toString(v.state);
  layer.reason = v.reason;
  layer.version = v.request_id;
  if (v.frozen_request) {
    layer.start = v.frozen_request->start.position;
    layer.goal = v.frozen_request->goal.position;
  }
  if (v.candidate) {
    layer.version = v.candidate->candidate_id;
    if (v.candidate->plan.initial_spline) layer.initial = Sample(*v.candidate->plan.initial_spline);
    if (v.candidate->plan.trajectory) layer.trajectory = Sample(*v.candidate->plan.trajectory);
    for (const auto& span : v.candidate->plan.collision_segments)
      layer.astar.push_back(span.astar_path);
  }
  return layer;
}
void OnlinePlanningSession::recordEvent(const RuntimeEvent& e) {
  const auto& v = e.view;
  events_ << std::setprecision(17) << "{\"steady_ns\":"
          << std::chrono::duration_cast<std::chrono::nanoseconds>(e.time.time_since_epoch()).count()
          << ",\"kind\":" << Json(e.kind) << ",\"validity\":" << Json(toString(v.validity))
          << ",\"state\":" << Json(toString(v.state)) << ",\"reason\":" << Json(v.reason)
          << ",\"goal_version\":" << (v.frozen_request ? v.frozen_request->goal_version : 0)
          << ",\"frozen_state_id\":" << v.frozen_state_id << ",\"state_age_ms\":" << v.state_age_ms
          << ",\"map_age_ms\":" << v.map_age_ms << ",\"pending_ms\":" << v.pending_ms
          << ",\"validation_ms\":" << v.last_validation_ms << ",\"observed_map\":";
  Stamp(events_, v.observed_map);
  events_ << ",\"request_id\":" << v.request_id << ",\"solves\":" << v.solve_count
          << ",\"revalidations\":" << v.revalidation_count
          << ",\"discarded\":" << v.discarded_count;
  if (v.candidate) {
    events_ << ",\"candidate_id\":" << v.candidate->candidate_id << ",\"planned_on\":";
    Stamp(events_, v.candidate->plan.planned_on);
    events_ << ",\"validated_on\":";
    Stamp(events_, v.candidate->validated_on);
  }
  events_ << "}\n";
  events_.flush();
  if (e.kind != "Revalidated")
    std::cout << "PLANNING_EVENT " << e.kind << " request=" << v.request_id << " "
              << toString(v.validity) << " " << v.reason << std::endl;
  if (e.kind != "Accepted" || !v.candidate) return;
  const auto& c = *v.candidate;
  auto directory = directory_ / ("request-" + std::to_string(c.request.request_id) + "-candidate-" +
                                 std::to_string(c.candidate_id));
  if (!std::filesystem::create_directory(directory))
    throw std::runtime_error("refusing planning artifact overwrite");
  savePlanningResult(directory, c.plan, c.request, config_, "OnlineStaticPreview");
  std::ofstream version(directory / "online-version.json");
  version.exceptions(std::ios::failbit | std::ios::badbit);
  version << "{\"mode\":\"HistoricalStaticPreviewNotExecutionPermission\",\"frozen_state_id\":"
          << c.frozen_state_id << ",\"candidate_id\":" << c.candidate_id << ",\"planned_on\":";
  Stamp(version, c.plan.planned_on);
  version << ",\"validated_on\":";
  Stamp(version, c.validated_on);
  version << ",\"latest_validation\":" << Json(toString(c.latest_validation.status))
          << ",\"validity_at_event\":" << Json(toString(v.validity)) << "}\n";
  version.close();
  std::ofstream request(directory / "request.yaml");
  request.exceptions(std::ios::failbit | std::ios::badbit);
  request << std::setprecision(17) << "request_id: " << c.request.request_id
          << "\ngoal_version: " << c.request.goal_version << '\n';
  const auto vector = [&](const char* name, const Vec3& p) {
    request << name << ": [" << p.x() << ", " << p.y() << ", " << p.z() << "]\n";
  };
  vector("start_position", c.request.start.position);
  vector("goal_position", c.request.goal.position);
  vector("start_velocity", c.request.start.velocity);
  vector("start_acceleration", c.request.start.acceleration);
  vector("goal_velocity", c.request.goal.velocity);
  vector("goal_acceleration", c.request.goal.acceleration);
  request.close();
  std::ofstream complete(directory / "artifact-complete.json");
  complete.exceptions(std::ios::failbit | std::ios::badbit);
  complete << "{\"complete\":true,\"historical_only\":true}\n";
  complete.close();
}
}  // namespace ego_planner::app
