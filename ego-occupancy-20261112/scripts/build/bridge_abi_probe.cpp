#include "pose_bridge.h"
#include <cstddef>
#include <iostream>
#include <type_traits>
using namespace ego_pose_bridge;
static_assert(std::is_standard_layout<Record>::value,"Record must remain standard layout");
#define FIELD(name,type) static_assert(std::is_same<decltype(Record::name),type>::value,"Record field type changed: " #name)
FIELD(captured,bool);FIELD(valid,bool);FIELD(tracking_state,int);
FIELD(session,std::uint64_t);FIELD(map_token,std::uint64_t);FIELD(revision,std::uint64_t);FIELD(frame_id,std::uint64_t);FIELD(timestamp_sec,double);
using PoseArray=std::array<float,16>;FIELD(T_world_camera,PoseArray);
static_assert(std::is_same<decltype(&Initialize),void(*)(std::uint64_t)>::value,"Initialize ABI");
static_assert(std::is_same<decltype(&SetObserver),void(*)(Observer,void*)>::value,"Observer ABI");
static_assert(std::is_same<decltype(&BeginFrame),void(*)(double)>::value,"BeginFrame ABI");
static_assert(std::is_same<decltype(&Capture),void(*)(std::uint64_t,double,const void*,const float*,int,bool)>::value,"Capture ABI");
static_assert(std::is_same<decltype(&Last),Record(*)()>::value,"Last ABI");
static_assert(std::is_same<decltype(&IsCurrent),bool(*)(const Record&)>::value,"IsCurrent ABI");
static_assert(std::is_same<decltype(&Revision),std::uint64_t(*)()>::value,"Revision ABI");
int main(){
    Record r;int called=0;{CaptureGuard guard([&]{++called;});}
    std::cout<<sizeof(Record)<<' '<<alignof(Record)<<' '<<sizeof(CorrectionScope)<<' '<<alignof(CorrectionScope)<<' '<<sizeof(CaptureGuard)<<' '<<alignof(CaptureGuard)<<'\n';
#define OFFSET(name) std::cout<<offsetof(Record,name)<<' ';
    OFFSET(captured);OFFSET(valid);OFFSET(tracking_state);OFFSET(session);OFFSET(map_token);OFFSET(revision);OFFSET(frame_id);OFFSET(timestamp_sec);OFFSET(T_world_camera);
    std::cout<<'\n'<<r.captured<<' '<<r.valid<<' '<<r.tracking_state<<' '<<r.session<<' '<<r.map_token<<' '<<r.revision<<' '<<r.frame_id<<' '<<r.timestamp_sec;
    for(float value:r.T_world_camera)std::cout<<' '<<value;
    std::cout<<'\n'<<called<<'\n';return called==1?0:1;
}
