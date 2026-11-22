#include <ego_occupancy/io.h>
#include <ego_planner/astar.h>
#include <ego_planner/bspline.h>
#include <ego_planner/polynomial_traj.h>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using ego_planner::Vec3;
double ParseNumber(const char* text) {
  std::size_t used = 0;
  const std::string value(text);
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result))
    throw std::invalid_argument("invalid finite numeric argument: " + value);
  return result;
}
std::ofstream Output(const std::filesystem::path& path) {
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << std::setprecision(17);
  return stream;
}
void JsonString(std::ostream& stream, const std::string& value) {
  constexpr char hex[] = "0123456789abcdef";
  stream << '"';
  for (unsigned char character : value) {
    if (character == '"' || character == '\\') stream << '\\' << character;
    else if (character < 32)
      stream << "\\u00" << hex[character >> 4] << hex[character & 15];
    else stream << character;
  }
  stream << '"';
}
void SaveMathExample(const std::filesystem::path& directory) {
  const Vec3 zero = Vec3::Zero();
  const auto polynomial = ego_planner::PolynomialTraj::Quintic(
      zero, zero, zero, Vec3(1, .5, .2), zero, zero, 4.0);
  std::vector<Vec3> samples;
  for (int i = 0; i <= 8; ++i) samples.push_back(polynomial.evaluate(i * .5));
  const std::array<Vec3, 4> boundary = {zero, zero, zero, zero};
  const auto spline = ego_planner::Bspline::Parameterize(samples, boundary, .5);
  const auto velocity = spline.derivative();
  const auto acceleration = velocity.derivative();
  auto csv = Output(directory / "math.csv");
  csv << "t,poly_x,poly_y,poly_z,poly_speed,poly_acceleration,"
         "spline_x,spline_y,spline_z,spline_speed,spline_acceleration\n";
  // 这是数学库独立示例，不将拟合曲线当作已经避障的A*后处理结果。
  for (int i = 0; i <= 200; ++i) {
    const double t = i * .02;
    const Vec3 p = polynomial.evaluate(t), q = spline.evaluate(t);
    csv << t << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << polynomial.evaluate(t, 1).norm() << ',' << polynomial.evaluate(t, 2).norm()
        << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
        << velocity.evaluate(t).norm() << ',' << acceleration.evaluate(t).norm() << '\n';
  }
}
void Usage() {
  std::cout << "ego-astar-probe MAP.egovox OUTPUT_DIR sx sy sz gx gy gz "
               "[half_extent_m=0.1] [budget_ms=200]\n"
               "Offline snapshot validation only; output directory must not exist.\n"
               "Exit codes: 0 success, 2 input/IO/view failure, 3 search rejected.\n";
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { Usage(); return 0; }
  try {
    if (argc < 9 || argc > 11) { Usage(); return 2; }
    const Vec3 start(ParseNumber(argv[3]), ParseNumber(argv[4]), ParseNumber(argv[5]));
    const Vec3 goal(ParseNumber(argv[6]), ParseNumber(argv[7]), ParseNumber(argv[8]));
    const double half_extent = argc >= 10 ? ParseNumber(argv[9]) : .1;
    const double budget_ms = argc >= 11 ? ParseNumber(argv[10]) : 200.0;
    if (half_extent < 0 || budget_ms <= 0 || budget_ms > 60'000)
      throw std::invalid_argument("half extent must be nonnegative; budget in (0,60000] ms");
    const std::filesystem::path directory(argv[2]);
    if (std::filesystem::exists(directory))
      throw std::runtime_error("refusing to overwrite output directory: " + directory.string());
    const auto snapshot = ego_occupancy::loadSnapshot(argv[1]);
    const auto began = ego_planner::Clock::now();
    const auto duration = std::chrono::duration_cast<ego_planner::Clock::duration>(
        std::chrono::duration<double, std::milli>(budget_ms));
    const ego_planner::WorkBudget budget{began + duration, nullptr};
    const ego_planner::PlanningMapView view(snapshot,
        {ego_planner::Index::Zero(), snapshot->dimensions()}, Vec3::Constant(half_extent), budget);
    const auto prepared = ego_planner::Clock::now();
    const auto result = ego_planner::Astar{}.search(view, start, goal, {}, budget);
    const double prepare_ms = std::chrono::duration<double, std::milli>(prepared - began).count();
    const double total_ms = std::chrono::duration<double, std::milli>(
        ego_planner::Clock::now() - began).count();
    if (!std::filesystem::create_directory(directory))
      throw std::runtime_error("cannot create output directory");
    auto csv = Output(directory / "path.csv");
    csv << "x,y,z\n";
    for (const Vec3& point : result.path)
      csv << point.x() << ',' << point.y() << ',' << point.z() << '\n';
    csv.close();
    const auto& metadata = result.map_metadata;
    auto report = Output(directory / "report.json");
    report << "{\n  \"mode\":\"OfflineValidation\",\n  \"status\":\""
           << ego_planner::toString(result.status) << "\",\n"
           << "  \"source\":[" << metadata.source.session << ',' << metadata.source.map
           << ',' << metadata.source.revision << "],\n"
           << "  \"epoch\":" << metadata.epoch << ",\n  \"sequence\":" << metadata.sequence
           << ",\n  \"frame_id\":";
    JsonString(report, metadata.frame_id);
    report << ",\n  \"resolution_m\":" << snapshot->resolution()
           << ",\n  \"half_extent_m\":" << half_extent
           << ",\n  \"roi_voxels\":" << view.voxelCount()
           << ",\n  \"expanded\":" << result.expanded
           << ",\n  \"queue_peak\":" << result.queue_peak
           << ",\n  \"prepare_ms\":" << prepare_ms
           << ",\n  \"astar_ms\":" << result.elapsed_ms
           << ",\n  \"total_ms\":" << total_ms
           << ",\n  \"budget_ms\":" << budget_ms
           << ",\n  \"length_m\":" << result.length_m
           << ",\n  \"path_points\":" << result.path.size()
           << ",\n  \"trajectory_optimized\":false\n}\n";
    report.close();
    SaveMathExample(directory);
    std::cout << ego_planner::toString(result.status) << " points=" << result.path.size()
              << " length_m=" << result.length_m << " prepare_ms=" << prepare_ms
              << " astar_ms=" << result.elapsed_ms << " total_ms=" << total_ms << '\n';
    return result.status == ego_planner::SearchStatus::Success ? 0 : 3;
  } catch (const std::exception& error) {
    std::cerr << "ego-astar-probe: " << error.what() << '\n';
    return 2;
  }
}
