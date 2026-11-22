#pragma once

#include <ego_planner/planner_core.h>

#include <filesystem>

namespace ego_planner::app {

/// 读取明确限定的平面YAML子集：key: scalar 或 key: [x,y,z]；未知/重复键拒绝。
PlanningConfig loadPlanningConfig(const std::filesystem::path& path);
PlanningRequest loadPlanningRequest(const std::filesystem::path& path);
double parseFiniteNumber(const std::string& text);
void savePlanningResult(const std::filesystem::path& directory, const PlanResult& result,
                        const PlanningRequest& request, const PlanningConfig& config,
                        const std::string& mode = "OfflineFrozenSnapshot");

}  // namespace ego_planner::app
