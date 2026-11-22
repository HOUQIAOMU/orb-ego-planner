#include <unistd.h>

#include <fstream>
#include <thread>

#include "online_planning_session.h"
#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
int main() {
  try {
    ego_pose_bridge::Initialize(8765);
    static int identity = 0;
    float t[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    ego_pose_bridge::BeginFrame(1);
    ego_pose_bridge::Capture(1, 1, &identity, t, 2, true);
    auto record = ego_pose_bridge::Last();
    ego_occupancy::Config mc;
    mc.size = {3, 3, 3};
    mc.resolution = .1;
    mc.max_age_sec = 3;
    mc.inflation.setZero();
    ego_occupancy::OccupancyMapper map(mc);
    ego_occupancy::SourceStamp source{record.session, record.map_token, record.revision};
    map.reset(source, Vec3::Zero());
    for (int i = 1; i <= 3; ++i) {
      ego_occupancy::RayObservation o;
      o.source = source;
      o.frame_index = i;
      o.timestamp_sec = i;
      o.received = Clock::now();
      for (int x = -4; x <= 4; ++x)
        for (int y = -4; y <= 4; ++y) o.rays.push_back({Vec3(x * .1, y * .1, 1.3), false});
      CHECK(map.integrate(o).published);
    }
    PlanningConfig cfg;
    cfg.half_extent.setZero();
    cfg.roi_margin = .2;
    auto root =
        std::filesystem::temp_directory_path() / ("ego-online-session-" + std::to_string(getpid()));
    app::OnlinePlanningSession session(map.reader(), cfg, root, .5, .9);
    session.update(record, 7, Clock::now(), true);
    CHECK(session.trigger());
    auto deadline = Clock::now() + std::chrono::seconds(3);
    while (!session.layer().ready && Clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    CHECK(session.layer().ready);
    CHECK(session.layer().trajectory.size() > 2);
    session.stop();
    bool csv = false, mode = false;
    for (const auto& e : std::filesystem::recursive_directory_iterator(root)) {
      if (e.path().filename() == "trajectory.csv") {
        std::ifstream f(e.path());
        std::string s;
        int lines = 0;
        while (std::getline(f, s)) ++lines;
        csv = lines > 2;
      }
      if (e.path().filename() == "planning-report.json") {
        std::ifstream f(e.path());
        std::string s((std::istreambuf_iterator<char>(f)), {});
        mode = s.find("OnlineStaticPreview") != std::string::npos;
      }
    }
    CHECK(csv && mode);
    CHECK(std::filesystem::file_size(root / "events.jsonl") > 0);
    CHECK(std::filesystem::exists(root / "request-1-candidate-1/artifact-complete.json"));
    const auto faultroot = root.string() + "-io-fault";
    app::OnlinePlanningSession broken(map.reader(), cfg, faultroot, .5, .9);
    std::filesystem::create_directory(std::filesystem::path(faultroot) / "request-1-candidate-1");
    broken.update(record, 9, Clock::now(), true);
    CHECK(broken.trigger());
    const auto failed_deadline = Clock::now() + std::chrono::seconds(3);
    while (broken.error().empty() && Clock::now() < failed_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    CHECK(broken.error().find("refusing planning artifact overwrite") != std::string::npos);
    CHECK(!broken.layer().ready);
    CHECK(!broken.trigger());
    broken.stop();
    std::filesystem::remove_all(faultroot);
    std::filesystem::remove_all(root);
    std::cout << "PASS online adapter, static layer and historical trajectory files\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
