#include "planning_io.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace ego_planner::app {
namespace {
std::string Trim(const std::string& text) {
  const auto first = text.find_first_not_of(" \t\r\n"), last = text.find_last_not_of(" \t\r\n");
  return first == std::string::npos ? "" : text.substr(first, last - first + 1);
}
std::map<std::string, std::string> ReadFlatYaml(const std::filesystem::path& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open configuration: " + path.string());
  if (std::filesystem::file_size(path) > 64 * 1024)
    throw std::invalid_argument("configuration exceeds 64 KiB");
  std::map<std::string, std::string> entries;
  std::string line;
  int number = 0;
  while (std::getline(stream, line)) {
    ++number;
    line = Trim(line.substr(0, line.find('#')));
    if (line.empty()) continue;
    const auto colon = line.find(':');
    if (colon == std::string::npos || colon != line.rfind(':'))
      throw std::invalid_argument("expected flat key: value at line " + std::to_string(number));
    const auto key = Trim(line.substr(0, colon)), value = Trim(line.substr(colon + 1));
    if (key.empty() || value.empty() || !entries.emplace(key, value).second)
      throw std::invalid_argument("empty or duplicate key at line " + std::to_string(number));
  }
  return entries;
}
Vec3 Vector(const std::string& text) {
  if (text.size() < 5 || text.front() != '[' || text.back() != ']')
    throw std::invalid_argument("expected vector [x,y,z]: " + text);
  std::istringstream stream(text.substr(1, text.size() - 2));
  Vec3 vector;
  for (int i = 0; i < 3; ++i) {
    std::string number;
    if (!std::getline(stream, number, ','))
      throw std::invalid_argument("expected exactly three vector entries");
    vector[i] = parseFiniteNumber(Trim(number));
  }
  std::string extra;
  if (std::getline(stream, extra, ',')) throw std::invalid_argument("too many vector entries");
  if (text[text.size() - 2] == ',') throw std::invalid_argument("trailing vector delimiter");
  return vector;
}
std::size_t Integer(const std::string& text) {
  const double value = parseFiniteNumber(text);
  if (value < 0 || value > 10'000'000 || std::floor(value) != value)
    throw std::invalid_argument("invalid bounded nonnegative integer: " + text);
  return static_cast<std::size_t>(value);
}
std::ofstream Output(const std::filesystem::path& path) {
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << std::setprecision(17);
  return stream;
}
void String(std::ostream& out, const std::string& text) {
  constexpr char hex[] = "0123456789abcdef";
  out << '"';
  for (unsigned char c : text) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u00" << hex[c >> 4] << hex[c & 15];
    else
      out << c;
  }
  out << '"';
}
void JsonVector(std::ostream& out, const Vec3& v) {
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
void Row(std::ostream& out, double t, const Vec3& p, const Vec3& v, const Vec3& a) {
  out << t;
  for (const auto* vector : {&p, &v, &a})
    for (int axis = 0; axis < 3; ++axis) out << ',' << (*vector)[axis];
  out << '\n';
}
void SaveSpline(const std::filesystem::path& path, const Bspline& spline) {
  auto out = Output(path);
  out << "{\"uniform_interval\":" << spline.knownUniformInterval()
      << ",\"degree\":" << spline.degree() << ",\"duration\":" << spline.duration()
      << ",\"control_points\":[";
  for (Eigen::Index i = 0; i < spline.controlPoints().cols(); ++i) {
    if (i) out << ',';
    JsonVector(out, spline.controlPoints().col(i));
  }
  out << "],\"knots\":[";
  for (Eigen::Index i = 0; i < spline.knots().size(); ++i) {
    if (i) out << ',';
    out << spline.knots()[i];
  }
  out << "]}\n";
  out.close();
}
}  // namespace

double parseFiniteNumber(const std::string& text) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
    throw std::invalid_argument("expected finite number: " + text);
  return value;
}
PlanningConfig loadPlanningConfig(const std::filesystem::path& path) {
  auto values = ReadFlatYaml(path);
  PlanningConfig c;
  std::map<std::string, double*> real = {{"nominal_speed", &c.nominal_speed},
                                         {"max_velocity", &c.max_velocity},
                                         {"max_acceleration", &c.max_acceleration},
                                         {"control_point_spacing", &c.control_point_spacing},
                                         {"roi_margin", &c.roi_margin},
                                         {"soft_clearance", &c.soft_clearance},
                                         {"smoothness_weight", &c.smoothness_weight},
                                         {"collision_weight", &c.collision_weight},
                                         {"feasibility_weight", &c.feasibility_weight},
                                         {"fitness_weight", &c.fitness_weight},
                                         {"boundary_tolerance", &c.boundary_tolerance},
                                         {"gate_tolerance", &c.gate_tolerance},
                                         {"max_duration", &c.max_duration}};
  for (const auto& [key, value] : values) {
    if (real.count(key))
      *real.at(key) = parseFiniteNumber(value);
    else if (key == "half_extent")
      c.half_extent = Vector(value);
    else if (key == "max_control_points")
      c.max_control_points = Integer(value);
    else if (key == "max_roi_voxels")
      c.max_roi_voxels = Integer(value);
    else if (key == "max_validation_visits")
      c.max_validation_visits = Integer(value);
    else if (key == "max_subdivision_depth")
      c.max_subdivision_depth = static_cast<int>(Integer(value));
    else if (key == "max_iterations")
      c.max_iterations = static_cast<int>(Integer(value));
    else if (key == "max_restarts")
      c.max_restarts = static_cast<int>(Integer(value));
    else if (key == "max_time_adjustments")
      c.max_time_adjustments = static_cast<int>(Integer(value));
    else if (key == "allow_roi_expansion" && (value == "true" || value == "false"))
      c.allow_roi_expansion = value == "true";
    else
      throw std::invalid_argument("unknown configuration key/value: " + key);
  }
  validateConfig(c);
  return c;
}
PlanningRequest loadPlanningRequest(const std::filesystem::path& path) {
  auto values = ReadFlatYaml(path);
  if (!values.count("start_position") || !values.count("goal_position"))
    throw std::invalid_argument("task requires start_position and goal_position");
  PlanningRequest r;
  Gate gate;
  int gate_keys = 0;
  std::map<int, Vec3> references;
  for (const auto& [key, value] : values) {
    if (key == "start_position")
      r.start.position = Vector(value);
    else if (key == "goal_position")
      r.goal.position = Vector(value);
    else if (key == "start_velocity")
      r.start.velocity = Vector(value);
    else if (key == "goal_velocity")
      r.goal.velocity = Vector(value);
    else if (key == "start_acceleration")
      r.start.acceleration = Vector(value);
    else if (key == "goal_acceleration")
      r.goal.acceleration = Vector(value);
    else if (key == "request_id")
      r.request_id = Integer(value);
    else if (key == "goal_version")
      r.goal_version = Integer(value);
    else if (key.size() == 11 && key.substr(0, 10) == "reference_" && key.back() >= '0' &&
             key.back() <= '6')
      references.emplace(key.back() - '0', Vector(value));
    else if (key == "gate_center") {
      gate.center = Vector(value);
      ++gate_keys;
    } else if (key == "gate_normal") {
      gate.normal = Vector(value);
      ++gate_keys;
    } else if (key == "gate_u") {
      gate.u = Vector(value);
      ++gate_keys;
    } else if (key == "gate_v") {
      gate.v = Vector(value);
      ++gate_keys;
    } else if (key == "gate_half_width") {
      gate.half_width = parseFiniteNumber(value);
      ++gate_keys;
    } else if (key == "gate_half_height") {
      gate.half_height = parseFiniteNumber(value);
      ++gate_keys;
    } else
      throw std::invalid_argument("unknown task key: " + key);
  }
  for (const auto& [index, point] : references) {
    if (index != static_cast<int>(r.reference_points.size()))
      throw std::invalid_argument("reference indices must start at 0 without gaps");
    r.reference_points.push_back(point);
  }
  if (gate_keys != 0 && gate_keys != 6)
    throw std::invalid_argument("gate requires all six gate fields");
  if (gate_keys) r.gate = gate;
  return r;
}

void savePlanningResult(const std::filesystem::path& directory, const PlanResult& r,
                        const PlanningRequest& request, const PlanningConfig& config,
                        const std::string& mode) {
  auto report = Output(directory / "planning-report.json");
  report << "{\"mode\":";
  String(report, mode);
  report << ",\"status\":";
  String(report, toString(r.status));
  report << ",\"reason\":";
  String(report, r.reason);
  report << ",\"request_id\":" << request.request_id << ",\"goal_version\":" << request.goal_version
         << ",\"source\":[" << r.planned_on.source.session << ',' << r.planned_on.source.map << ','
         << r.planned_on.source.revision << "],\"epoch\":" << r.planned_on.epoch
         << ",\"sequence\":" << r.planned_on.sequence << ",\"frame_id\":";
  String(report, r.planned_on.frame_id);
  report << ",\"optimizer_status\":";
  String(report, toString(r.optimizer_status));
  report << ",\"selected_candidate\":";
  String(report, toString(r.selected_candidate));
  report << ",\"iterations\":" << r.iterations
         << ",\"cost_scope\":\"last_optimizer_call\",\"cost_terms_raw\":{\"smoothness\":"
         << r.smoothness_cost << ",\"collision\":" << r.collision_cost
         << ",\"feasibility\":" << r.feasibility_cost << ",\"fitness\":" << r.fitness_cost << "}";
  report << ",\"solver_code\":" << r.solver_code << ",\"evaluations\":" << r.evaluations
         << ",\"astar_calls\":" << r.astar_calls << ",\"roi_attempts\":" << r.roi_attempts
         << ",\"restarts\":" << r.restarts << ",\"time_adjustments\":" << r.time_adjustments
         << ",\"initial_cost\":" << r.initial_cost << ",\"final_cost\":" << r.final_cost
         << ",\"roi_min\":";
  JsonVector(report, r.roi.minimum.cast<double>());
  report << ",\"roi_max_exclusive\":";
  JsonVector(report, r.roi.maximum_exclusive.cast<double>());
  report << ",\"limits\":{\"velocity\":" << config.max_velocity
         << ",\"acceleration\":" << config.max_acceleration << "}";
  report << ",\"half_extent\":";
  JsonVector(report, config.half_extent);
  const auto& t = r.timings;
  report << ",\"timings_ms\":{\"seed\":" << t.seed_ms << ",\"view\":" << t.view_ms
         << ",\"constraints\":" << t.constraints_ms << ",\"optimization\":" << t.optimization_ms
         << ",\"validation\":" << t.validation_ms << ",\"total\":" << t.total_ms << "}}\n";
  auto validation = Output(directory / "validation.json");
  validation << "{\"status\":";
  String(validation, toString(r.validation.status));
  validation << ",\"reason\":";
  String(validation, r.validation.reason);
  validation << ",\"boundary_error\":" << r.validation.boundary_error
             << ",\"speed_bound\":" << r.validation.speed_bound
             << ",\"acceleration_bound\":" << r.validation.acceleration_bound
             << ",\"visits\":" << r.validation.visits
             << ",\"collision_span\":" << r.validation.collision_span
             << ",\"collision_begin\":" << r.validation.collision_begin
             << ",\"collision_end\":" << r.validation.collision_end
             << ",\"gate_crossings\":" << r.validation.gate_crossings << "}\n";
  auto constraints = Output(directory / "constraints.json");
  constraints << '[';
  for (std::size_t i = 0; i < r.constraints.size(); ++i) {
    const auto& proxy = r.constraints[i];
    if (i) constraints << ',';
    constraints << "{\"control_index\":" << proxy.control_index << ",\"base_point\":";
    JsonVector(constraints, proxy.base_point);
    constraints << ",\"direction\":";
    JsonVector(constraints, proxy.direction);
    constraints << '}';
  }
  constraints << "]\n";
  std::filesystem::create_directory(directory / "astar-paths");
  auto segments = Output(directory / "collision-segments.json");
  segments << '[';
  for (std::size_t i = 0; i < r.collision_segments.size(); ++i) {
    const auto& segment = r.collision_segments[i];
    if (i) segments << ',';
    segments << "{\"begin\":" << segment.begin_time << ",\"end\":" << segment.end_time << "}";
    auto path = Output(directory / "astar-paths" / (std::to_string(i) + ".csv"));
    path << "x,y,z\n";
    for (const auto& p : segment.astar_path) path << p.x() << ',' << p.y() << ',' << p.z() << '\n';
    path.close();
  }
  segments << "]\n";
  if (r.initial_spline) SaveSpline(directory / "initial-bspline.json", *r.initial_spline);
  if (r.trajectory) SaveSpline(directory / "bspline.json", *r.trajectory);
  auto reference = Output(directory / "reference.csv"),
       trajectory = Output(directory / "trajectory.csv");
  constexpr auto header = "t,x,y,z,vx,vy,vz,ax,ay,az\n";
  reference << header;
  trajectory << header;
  if (r.reference)
    for (int i = 0; i <= 400; ++i) {
      const double time = r.reference->duration() * i / 400;
      Row(reference, time, r.reference->evaluate(time), r.reference->evaluate(time, 1),
          r.reference->evaluate(time, 2));
    }
  if (r.trajectory) {
    const auto velocity = r.trajectory->derivative(), acceleration = velocity.derivative();
    for (int i = 0; i <= 400; ++i) {
      const double time = r.trajectory->duration() * i / 400;
      Row(trajectory, time, r.trajectory->evaluate(time), velocity.evaluate(time),
          acceleration.evaluate(time));
    }
  }
  report.close();
  validation.close();
  constraints.close();
  segments.close();
  reference.close();
  trajectory.close();
}
}  // namespace ego_planner::app
