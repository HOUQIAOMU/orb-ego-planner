#pragma once
#include <ego_occupancy/io.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + " " + #x); } while (false)
namespace planning_test {
template<class F> void Reject(F f) {
  bool caught = false;
  try { f(); } catch (const std::exception&) { caught = true; }
  CHECK(caught);
}
/// 测试夹具只写公开.egovox格式，再走真实加载器；不开放生产类的可写体素接口。
inline std::shared_ptr<const ego_occupancy::MapSnapshot> MakeMap(
    Eigen::Vector3i dimensions, double resolution, Eigen::Vector3d origin,
    const std::function<ego_occupancy::VoxelState(Eigen::Vector3i)>& classify,
    bool inflated = false) {
  std::vector<std::uint8_t> bytes;
  auto integer = [&](std::uint64_t value, int width) {
    for (int i=0; i<width; ++i) bytes.push_back(static_cast<std::uint8_t>(value>>(8*i)));
  };
  auto real = [&](double value) { std::uint64_t bits; std::memcpy(&bits,&value,8); integer(bits,8); };
  for (char c : std::string("EGOOCC01")) bytes.push_back(c);
  integer(1,4);
  for(int k=0;k<3;++k) integer(dimensions[k],4);
  real(resolution);
  for(int k=0;k<3;++k) real(origin[k]);
  for(std::uint64_t value : {7ULL,8ULL,9ULL,2ULL,3ULL,4ULL}) integer(value,8);
  real(1.0); integer(7,4);
  for(char c:std::string("fixture")) bytes.push_back(c);
  const std::size_t count=static_cast<std::size_t>(dimensions.prod());
  integer(count,8);
  // 与公开格式一致：z为连续轴，随后y和x（见map.cpp的Address）。
  for(int x=0;x<dimensions.x();++x) for(int y=0;y<dimensions.y();++y)
    for(int z=0;z<dimensions.z();++z)
      bytes.push_back(static_cast<std::uint8_t>(classify({x,y,z})));
  bytes.insert(bytes.end(),count,inflated?1:0);
  std::uint64_t hash=14695981039346656037ULL;
  for(auto byte:bytes) { hash^=byte; hash*=1099511628211ULL; }
  integer(hash,8);
  const auto path=std::filesystem::temp_directory_path()/("planning-fixture-"+std::to_string(getpid())+".egovox");
  { std::ofstream out(path,std::ios::binary); out.exceptions(std::ios::failbit|std::ios::badbit);
    out.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size())); }
  try { auto map=ego_occupancy::loadSnapshot(path); std::filesystem::remove(path); return map; }
  catch(...) { std::filesystem::remove(path); throw; }
}
inline auto FreeMap(Eigen::Vector3i n={9,9,9}, double resolution=0.05) {
  return MakeMap(n,resolution,{-0.225,-0.225,-0.225},
    [](auto){return ego_occupancy::VoxelState::Free;});
}
} // namespace planning_test
