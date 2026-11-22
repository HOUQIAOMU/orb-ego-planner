#include <ego_occupancy/io.h>

#include <iostream>

#include "planning_io.h"

namespace {
void Usage() {
  std::cout << "ego-plan-snapshot MAP.egovox TASK.yaml CONFIG.yaml OUTPUT_DIR [budget_ms=5000]\n"
            << "Offline frozen-snapshot validation. Output directory must not exist.\n"
            << "Exit: 0 validated trajectory; 2 input/IO error; 3 planning rejected.\n";
}
}  // namespace
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    Usage();
    return 0;
  }
  try {
    if (argc < 5 || argc > 6) {
      Usage();
      return 2;
    }
    const double budget_ms = argc == 6 ? ego_planner::app::parseFiniteNumber(argv[5]) : 5000;
    if (budget_ms <= 0 || budget_ms > 60'000)
      throw std::invalid_argument("budget must be in (0,60000] ms");
    const std::filesystem::path output(argv[4]);
    if (std::filesystem::exists(output))
      throw std::runtime_error("refusing to overwrite existing output: " + output.string());
    const auto config = ego_planner::app::loadPlanningConfig(argv[3]);
    const auto request = ego_planner::app::loadPlanningRequest(argv[2]);
    ego_planner::validateRequest(request, config);
    const auto snapshot = ego_occupancy::loadSnapshot(argv[1]);
    const auto duration = std::chrono::duration_cast<ego_planner::Clock::duration>(
        std::chrono::duration<double, std::milli>(budget_ms));
    const auto result = ego_planner::PlannerCore{}.plan(
        snapshot, request, config, {ego_planner::Clock::now() + duration, nullptr});
    if (!std::filesystem::create_directory(output))
      throw std::runtime_error("cannot create exclusive output directory");
    std::filesystem::copy_file(argv[2], output / "request.yaml");
    std::filesystem::copy_file(argv[3], output / "planning.yaml");
    ego_planner::app::savePlanningResult(output, result, request, config);
    std::cout << ego_planner::toString(result.status) << ": " << result.reason << "\n"
              << "total=" << result.timings.total_ms << " ms, A*=" << result.astar_calls
              << ", evaluations=" << result.evaluations
              << ", validation=" << ego_planner::toString(result.validation.status)
              << "\nreport=" << (output / "planning-report.json") << '\n';
    return result.status == ego_planner::PlanStatus::Success ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << "ego-plan-snapshot: " << e.what() << '\n';
    return 2;
  }
}
