#include <ego_planner/astar.h>
#include <algorithm>
#include <cmath>
#include <new>
#include <queue>

namespace ego_planner {
const char* toString(SearchStatus status) {
  switch(status) {
    case SearchStatus::Success:return "Success";
    case SearchStatus::InvalidInput:return "InvalidInput";
    case SearchStatus::StartBlocked:return "StartBlocked";
    case SearchStatus::GoalBlocked:return "GoalBlocked";
    case SearchStatus::NoPath:return "NoPath";
    case SearchStatus::Timeout:return "Timeout";
    case SearchStatus::Cancelled:return "Cancelled";
    case SearchStatus::ResourceLimit:return "ResourceLimit";
  }
  return "InvalidInput";
}
SearchResult Astar::search(const PlanningMapView& view,const Vec3& start,const Vec3& goal,
                          SearchOptions options,WorkBudget budget) const {
  const auto began=Clock::now();
  SearchResult result;
  result.map_metadata=view.snapshot().metadata();
  auto finish=[&](SearchStatus status) {
    result.status=status;
    if(status!=SearchStatus::Success) { result.path.clear();result.length_m=0; }
    result.elapsed_ms=std::chrono::duration<double,std::milli>(Clock::now()-began).count();
    return result;
  };
  auto interrupted=[&]() {
    return budget.status()==WorkStatus::Cancelled?SearchStatus::Cancelled:SearchStatus::Timeout;
  };

  if(budget.status()!=WorkStatus::Ready) {
    return finish(interrupted());
  }
  if(!start.allFinite() || !goal.allFinite()) {
    return finish(SearchStatus::InvalidInput);
  }
  const auto begin_index=view.snapshot().worldToIndex(start);
  const auto end_index=view.snapshot().worldToIndex(goal);
  
  if(!begin_index || !end_index) return finish(SearchStatus::InvalidInput);
  const Roi& roi=view.roi();
  auto inside=[&](const Index& p) {
    return (p.array()>=roi.minimum.array()).all() && (p.array()<roi.maximum_exclusive.array()).all();
  };
  if(!inside(*begin_index) || !inside(*end_index)) return finish(SearchStatus::InvalidInput);
  if(!view.pointFree(start)) return finish(SearchStatus::StartBlocked);
  if(!view.pointFree(goal)) return finish(SearchStatus::GoalBlocked);
  if(view.voxelCount()>options.max_nodes || options.max_expansions==0 || options.max_queue_entries==0)
    return finish(SearchStatus::ResourceLimit);
  try {
    const Index dims=roi.maximum_exclusive-roi.minimum;
    const std::size_t nx=static_cast<std::size_t>(dims.x()),ny=static_cast<std::size_t>(dims.y());
    auto id=[&](const Index& p) {
      const Index q=p-roi.minimum;
      return static_cast<std::size_t>(q.x())+nx*(static_cast<std::size_t>(q.y())+ny*q.z());
    };
    auto index=[&](std::size_t value) -> Index {
      return roi.minimum+Index(static_cast<int>(value%nx),static_cast<int>((value/nx)%ny),
                               static_cast<int>(value/(nx*ny)));
    };
    const double resolution=view.snapshot().resolution();
    auto heuristic=[&](const Index& p) {
      return (p.cast<double>()-end_index->cast<double>()).norm()*resolution;
    };
    const auto begin=id(*begin_index),end=id(*end_index),count=view.voxelCount();
    const auto none=std::numeric_limits<std::size_t>::max();
    std::vector<double> distance(count,std::numeric_limits<double>::infinity());
    std::vector<std::size_t> parent(count,none);
    std::vector<std::uint8_t> closed(count,0);
    struct Entry { double f,g; std::size_t id; };
    struct Greater {
      bool operator()(const Entry& a,const Entry& b) const {
        if(a.f!=b.f) return a.f>b.f;
        if(a.g!=b.g) return a.g>b.g;
        return a.id>b.id;
      }
    };
    std::priority_queue<Entry,std::vector<Entry>,Greater> queue;
    distance[begin]=0;queue.push({heuristic(*begin_index),0,begin});result.queue_peak=1;
    while(!queue.empty()) {
      if(budget.status()!=WorkStatus::Ready) return finish(interrupted());
      const Entry entry=queue.top();queue.pop();
      // 队列项不可变。更优路径另插一项，弹出时丢弃旧记录，避免修改堆内节点破坏顺序。
      if(entry.g!=distance[entry.id] || closed[entry.id]) continue;
      if(result.expanded>=options.max_expansions) return finish(SearchStatus::ResourceLimit);
      ++result.expanded;
      const Index p=index(entry.id);
      if(entry.id==end) {
        std::vector<Index> reverse;
        for(std::size_t node=end;;node=parent[node]) {
          reverse.push_back(index(node));
          if(node==begin) break;
          if(parent[node]==none || reverse.size()>count) return finish(SearchStatus::InvalidInput);
        }
        auto append=[&](const Vec3& point) {
          if(result.path.empty() || (result.path.back()-point).squaredNorm()>0)
            result.path.push_back(point);
        };
        append(start);
        // 同一格中的真实端点直接连接；整格配置空间已验证，不必绕到格中心。
        if(begin!=end) for(auto it=reverse.rbegin();it!=reverse.rend();++it)
          append(view.snapshot().indexToWorld(*it));
        append(goal);
        for(std::size_t i=1;i<result.path.size();++i)
          result.length_m+=(result.path[i]-result.path[i-1]).norm();
        if(budget.status()!=WorkStatus::Ready) return finish(interrupted());
        return finish(SearchStatus::Success);
      }
      closed[entry.id]=1;
      for(int z=-1;z<=1;++z) for(int y=-1;y<=1;++y) for(int x=-1;x<=1;++x) {
        if(x==0 && y==0 && z==0) continue;
        // p属于地图有效索引，向外一步仍在int范围内；view统一处理ROI外阻挡。
        const Index q=p+Index(x,y,z);
        if(!view.edgeFree(p,q)) continue;
        const auto neighbor=id(q);
        if(closed[neighbor]) continue;
        const double candidate=entry.g+std::sqrt(double(x*x+y*y+z*z))*resolution;
        if(candidate>=distance[neighbor]) continue;
        if(queue.size()>=options.max_queue_entries) return finish(SearchStatus::ResourceLimit);
        distance[neighbor]=candidate;parent[neighbor]=entry.id;
        queue.push({candidate+heuristic(q),candidate,neighbor});
        result.queue_peak=std::max(result.queue_peak,queue.size());
      }
    }
    return finish(SearchStatus::NoPath);
  } catch(const std::bad_alloc&) {
    return finish(SearchStatus::ResourceLimit);
  }
}
}  // namespace ego_planner
