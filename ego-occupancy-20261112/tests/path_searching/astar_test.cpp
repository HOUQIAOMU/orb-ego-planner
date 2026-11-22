#include "test_support.h"
#include <ego_planner/astar.h>
#include <queue>
#include <cmath>
#include <limits>
using namespace ego_planner;
using namespace planning_test;
using ego_occupancy::VoxelState;
namespace {
WorkBudget LongBudget(){return {Clock::now()+std::chrono::seconds(5),nullptr};}
Vec3 Point(const PlanningMapView& v,Index i){return v.snapshot().indexToWorld(i);}
void CheckEdges(const PlanningMapView& v,const SearchResult& result){
 CHECK(!result.path.empty());
 for(std::size_t i=1;i<result.path.size();++i){
  auto a=v.snapshot().worldToIndex(result.path[i-1]);auto b=v.snapshot().worldToIndex(result.path[i]);
  CHECK(a && b && v.edgeFree(*a,*b));
 }
}
double ReferenceDijkstra(const PlanningMapView& view,Index start,Index goal){
 const Index dims=view.roi().maximum_exclusive-view.roi().minimum;
 auto id=[&](Index p){return p.x()+dims.x()*(p.y()+dims.y()*p.z());};
 std::vector<double> distance(static_cast<std::size_t>(dims.prod()),std::numeric_limits<double>::infinity());
 using Item=std::pair<double,int>;
 std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
 distance[id(start)]=0;queue.emplace(0,id(start));
 while(!queue.empty()){
  auto [d,flat]=queue.top();queue.pop();
  if(d!=distance[flat])continue;
  Index p(flat%dims.x(),(flat/dims.x())%dims.y(),flat/(dims.x()*dims.y()));
  if(p==goal)return d;
  for(int z=-1;z<=1;++z)for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x){
   Index q=p+Index(x,y,z);
   // 参考实现直接逐轴组合检查raw格，不依赖被测A*队列或edgeFree。
   bool free=true;
   for(int mask=0;mask<8;++mask){
    Index touched=p;
    for(int a=0;a<3;++a)if(mask&(1<<a))touched[a]=q[a];
    if(view.snapshot().query(touched).state!=VoxelState::Free)free=false;
   }
   if(!free || q==p)continue;
   double candidate=d+std::sqrt(double(x*x+y*y+z*z))*.05;
   if(candidate<distance[id(q)]){distance[id(q)]=candidate;queue.emplace(candidate,id(q));}
  }
 }
 return std::numeric_limits<double>::infinity();
}
}
int main(){
 try{
  Astar astar;
  auto map=FreeMap({11,11,11});
  PlanningMapView view(map,{{0,0,0},{11,11,11}},Vec3::Zero());
  auto start=Point(view,{1,1,1}),goal=Point(view,{8,8,8});
  auto result=astar.search(view,start,goal,{},LongBudget());
  CHECK(result.status==SearchStatus::Success);CheckEdges(view,result);
  CHECK((result.path.front()-start).norm()==0 && (result.path.back()-goal).norm()==0);
  CHECK(std::abs(result.length_m-7*.05*std::sqrt(3.0))<1e-10);
  auto same=astar.search(view,start,start,{},LongBudget());
  CHECK(same.status==SearchStatus::Success && same.path.size()==1 && same.length_m==0);
  auto offset=astar.search(view,start+Vec3(.009,.003,0),goal+Vec3(-.008,0,0),{},LongBudget());
  CHECK(offset.status==SearchStatus::Success);
  CHECK((offset.path.front()-(start+Vec3(.009,.003,0))).norm()==0);
  CHECK((offset.path.back()-(goal+Vec3(-.008,0,0))).norm()==0);
  auto fixture=[](bool closed,bool unknown){
   return MakeMap({11,11,11},.05,Vec3(-.275,-.275,-.275),[=](Index p){
    if(p.x()!=5)return VoxelState::Free;
    if(!closed && p.y()>=4 && p.y()<=6 && p.z()>=4 && p.z()<=6)
      return unknown?VoxelState::Unknown:VoxelState::Free;
    return VoxelState::Occupied;
   });
  };
  PlanningMapView gate(fixture(false,false),{{0,0,0},{11,11,11}},Vec3::Zero());
  auto through=astar.search(gate,Point(gate,{1,2,5}),Point(gate,{9,2,5}),{},LongBudget());
  CHECK(through.status==SearchStatus::Success);CheckEdges(gate,through);
  CHECK(std::abs(through.length_m-ReferenceDijkstra(gate,{1,2,5},{9,2,5}))<1e-10);
  PlanningMapView shut(fixture(true,false),{{0,0,0},{11,11,11}},Vec3::Zero());
  auto no_path=astar.search(shut,Point(shut,{1,2,5}),Point(shut,{9,2,5}),{},LongBudget());
  CHECK(no_path.status==SearchStatus::NoPath && no_path.path.empty());
  PlanningMapView unknown(fixture(false,true),{{0,0,0},{11,11,11}},Vec3::Zero());
  CHECK(astar.search(unknown,Point(unknown,{1,2,5}),Point(unknown,{9,2,5}),{},LongBudget()).status==SearchStatus::NoPath);
  auto narrow_map=MakeMap({21,11,11},.05,Vec3::Zero(),[](Index p){
    if(p.x()!=10 || (p.y()>=4 && p.y()<=6 && p.z()>=4 && p.z()<=6)) return VoxelState::Free;
    return VoxelState::Occupied;
  });
  PlanningMapView narrow(narrow_map,{{0,0,0},{21,11,11}},Vec3(.1,.1,.1));
  CHECK(astar.search(narrow,Point(narrow,{4,5,5}),Point(narrow,{16,5,5}),{},LongBudget()).status==SearchStatus::NoPath);
  CHECK(astar.search(shut,Point(shut,{5,5,5}),Point(shut,{8,5,5}),{},LongBudget()).status==SearchStatus::StartBlocked);
  CHECK(astar.search(shut,Point(shut,{2,5,5}),Point(shut,{5,5,5}),{},LongBudget()).status==SearchStatus::GoalBlocked);
  CHECK(astar.search(view,Vec3(-100,0,0),goal).status==SearchStatus::InvalidInput);
  CHECK(astar.search(view,Vec3(std::numeric_limits<double>::quiet_NaN(),0,0),goal).status==SearchStatus::InvalidInput);
  CHECK(astar.search(view,start,goal,{}, {Clock::now(),nullptr}).status==SearchStatus::Timeout);
  std::atomic<bool> stop{true};
  CHECK(astar.search(view,start,goal,{}, {Clock::now()+std::chrono::seconds(1),&stop}).status==SearchStatus::Cancelled);
  CHECK(astar.search(view,start,goal,{10,100,100},LongBudget()).status==SearchStatus::ResourceLimit);
  CHECK(astar.search(view,start,goal,{2000,1,1000},LongBudget()).status==SearchStatus::ResourceLimit);
  CHECK(astar.search(view,start,goal,{2000,2000,1},LongBudget()).status==SearchStatus::ResourceLimit);
  CHECK(astar.search(view,start,goal,{},LongBudget()).status==SearchStatus::Success); // 失败不能污染下一轮。
  auto corner=MakeMap({2,2,1},.05,Vec3::Zero(),[](Index p){
    return (p==Index(0,0,0)||p==Index(1,1,0))?VoxelState::Free:VoxelState::Occupied;});
  PlanningMapView diagonal(corner,{{0,0,0},{2,2,1}},Vec3::Zero());
  CHECK(astar.search(diagonal,Point(diagonal,{0,0,0}),Point(diagonal,{1,1,0}),{},LongBudget()).status==SearchStatus::NoPath);
  // 外廓与非Free闭合体素仅接触面/棱/角时，也必须拒绝。
  for(auto state:{VoxelState::Unknown,VoxelState::Occupied})
    for(const Index& obstacle:{Index(1,5,5),Index(1,3,5),Index(1,3,3)}) {
      auto touching=MakeMap({12,12,12},.125,Vec3::Zero(),[=](Index p){
        return p==obstacle?state:VoxelState::Free;});
      PlanningMapView contact(touching,{{0,0,0},{12,12,12}},Vec3(.25,.25,.25));
      CHECK(astar.search(contact,Vec3(.5,.75,.75),Vec3(.875,.875,.875),{},
                         LongBudget()).status==SearchStatus::StartBlocked);
    }
  auto boundary_map=MakeMap({12,12,12},.125,Vec3::Zero(),[](Index){return VoxelState::Free;});
  PlanningMapView boundary(boundary_map,{{0,0,0},{12,12,12}},Vec3(.25,.25,.25));
  CHECK(astar.search(boundary,Vec3(.25,.75,.75),Vec3(.875,.875,.875),{},
                     LongBudget()).status==SearchStatus::StartBlocked);
  auto point_map=MakeMap({12,12,12},.125,Vec3::Zero(),[](Index p){
    return p==Index(3,6,6)?VoxelState::Unknown:VoxelState::Free;});
  PlanningMapView point_contact(point_map,{{0,0,0},{12,12,12}},Vec3::Zero());
  CHECK(astar.search(point_contact,Vec3(.5,.75,.75),Vec3(.875,.8125,.8125),{},
                     LongBudget()).status==SearchStatus::StartBlocked);
  std::cout<<"PASS A*: gate/closed/unknown/corners, exact endpoints, Dijkstra, repeated search, bounded failure\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
