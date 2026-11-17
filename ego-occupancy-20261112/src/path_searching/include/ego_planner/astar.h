#pragma once
#include <ego_planner/planning_map_view.h>
#include <limits>
#include <vector>
namespace ego_planner {
enum class SearchStatus {
  Success, InvalidInput, StartBlocked, GoalBlocked, NoPath,
  Timeout, Cancelled, ResourceLimit
};
const char* toString(SearchStatus status);
struct SearchOptions {
  std::size_t max_nodes=4'000'000;
  std::size_t max_expansions=4'000'000;
  std::size_t max_queue_entries=4'000'000;
};
struct SearchResult {
  SearchStatus status=SearchStatus::InvalidInput;
  std::vector<Vec3> path;
  double length_m=0.0;
  std::size_t expanded=0;
  std::size_t queue_peak=0;
  double elapsed_ms=0.0;
  ego_occupancy::Metadata map_metadata;
};
class Astar {
 public:
  /// 无跨任务可变状态；失败结果的path始终为空。起终点不会被自动平移。
  SearchResult search(const PlanningMapView& view,const Vec3& start,const Vec3& goal,
                      SearchOptions options={},WorkBudget budget={}) const;
};
}
