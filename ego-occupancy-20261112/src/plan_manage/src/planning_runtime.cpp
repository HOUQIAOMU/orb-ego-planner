#include <ego_planner/planning_runtime.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace ego_planner {
namespace occ = ego_occupancy;
namespace {
bool SameEpoch(const occ::Metadata& a, const occ::Metadata& b) {
  return a.source == b.source && a.epoch == b.epoch;
}
bool SameVersion(const occ::Metadata& a, const occ::Metadata& b) {
  return SameEpoch(a, b) && a.sequence == b.sequence;
}
bool Fresh(const RuntimeState& state, Clock::time_point now, double age) {
  const double elapsed = std::chrono::duration<double>(now - state.received).count();
  return state.valid && elapsed >= 0 && elapsed <= age;
}
Roi CurveRoi(const occ::MapSnapshot& map, const Bspline& curve, const PlanningConfig& cfg) {
  Vec3 low = curve.controlPoints().rowwise().minCoeff() - cfg.half_extent -
             Vec3::Constant(2 * map.resolution());
  Vec3 high = curve.controlPoints().rowwise().maxCoeff() + cfg.half_extent +
              Vec3::Constant(2 * map.resolution());
  Roi roi;
  for (int axis = 0; axis < 3; ++axis) {
    // 先在浮点域限幅，避免异常几何导致浮点转int溢出。
    roi.minimum[axis] =
        static_cast<int>(std::clamp(std::floor((low[axis] - map.origin()[axis]) / map.resolution()),
                                    0., double(map.dimensions()[axis])));
    roi.maximum_exclusive[axis] = static_cast<int>(
        std::clamp(std::ceil((high[axis] - map.origin()[axis]) / map.resolution()) + 1., 0.,
                   double(map.dimensions()[axis])));
  }
  return roi;
}
}  // namespace
const char* toString(TrajectoryValidity v) {
  switch (v) {
    case TrajectoryValidity::Invalid:
      return "INVALID";
    case TrajectoryValidity::PendingValidation:
      return "PENDING";
    case TrajectoryValidity::Ready:
      return "READY";
  }
  return "INVALID";
}
struct PlanningRuntime::Impl {
  occ::MapReader reader;
  PlanningConfig config;
  RuntimeOptions options;
  EventSink sink;
  std::string fatal_error;
  mutable std::mutex mutex;
  std::condition_variable wake;
  RuntimeState state;
  RuntimeView output;
  PlanningFsm fsm;
  occ::Metadata current_map;
  bool have_map = false;
  std::optional<PlanningRequest> task;
  occ::Metadata task_map;
  std::uint64_t frozen_id = 0, generation = 0, next_request = 0, next_candidate = 0;
  std::shared_ptr<std::atomic<bool>> cancellation = std::make_shared<std::atomic<bool>>(false);
  std::atomic<bool> finish{false};
  std::thread worker;
  Clock::time_point next_solve{}, next_validation{}, pending_since{};
  bool needs_replan = false;
  occ::Metadata attempted_on;
  std::uint64_t attempted_generation = 0, saved_candidate = 0;

  Impl(occ::MapReader r, PlanningConfig c, RuntimeOptions o, EventSink s)
      : reader(std::move(r)), config(std::move(c)), options(o), sink(std::move(s)) {
    validateConfig(config);
    if (!std::isfinite(o.max_state_age_sec) || o.max_state_age_sec <= 0 || o.budget_ms < 1 ||
        o.budget_ms > 10000 || o.retry_interval_ms < 1)
      throw std::invalid_argument("invalid runtime limits");
    worker = std::thread([this] { Run(); });
  }
  void Revoke(const std::string& reason) {  // mutex已持有；不等待worker才能撤销。
    cancellation->store(true);
    ++generation;
    task.reset();
    output.has_task = false;
    output.frozen_request.reset();
    output.candidate.reset();
    output.validity = TrajectoryValidity::Invalid;
    output.reason = reason;
    fsm.onEvent({EventType::TaskCleared, current_map});
    output.state = fsm.state();
  }
  bool SubmitLocked(PlanningRequest request, const occ::Metadata& map) {
    if (finish) return false;
    if (!Fresh(state, Clock::now(), options.max_state_age_sec) || state.source != map.source) {
      output.reason = "trigger rejected: fresh matching camera state required";
      return false;
    }
    if (!request.start.velocity.isZero() || !request.start.acceleration.isZero() ||
        !request.goal.velocity.isZero() || !request.goal.acceleration.isZero()) {
      output.reason = "static runtime requires zero endpoint velocity/acceleration";
      return false;
    }
    try {
      validateRequest(request, config);
    } catch (const std::exception& e) {
      output.reason = e.what();
      return false;
    }
    cancellation->store(true);
    cancellation = std::make_shared<std::atomic<bool>>(false);
    ++generation;
    request.request_id = ++next_request;
    request.goal_version = next_request;
    current_map = map;
    have_map = true;
    fsm.onEvent({EventType::MapAvailable, map});
    fsm.onEvent({EventType::StateValid, map});
    fsm.onEvent({EventType::TaskSet, map, request.request_id, request.goal_version});
    task = request;
    task_map = map;
    frozen_id = state.state_id;
    output.frozen_state_id = frozen_id;
    needs_replan = false;
    next_validation = {};
    pending_since = Clock::now();
    output.request_id = request.request_id;
    output.has_task = true;
    output.frozen_request = request;
    output.candidate.reset();
    output.state = fsm.state();
    output.validity = TrajectoryValidity::PendingValidation;
    output.reason = "static task queued; start is frozen";
    wake.notify_all();
    return true;
  }
  RuntimeView View() const {
    auto map = reader.latestForDisplay();
    const auto now = Clock::now();
    const bool usable = reader.isUsable(map, now);
    std::lock_guard<std::mutex> lock(mutex);
    RuntimeView result = output;
    result.state_age_ms = std::chrono::duration<double, std::milli>(now - state.received).count();
    if (map) {
      result.observed_map = map->metadata();
      result.map_age_ms =
          std::chrono::duration<double, std::milli>(now - map->metadata().received).count();
    }
    if (!fatal_error.empty()) {
      result.validity = TrajectoryValidity::Invalid;
      result.reason = "runtime failure: " + fatal_error;
      return result;
    }
    if (finish || !usable || !Fresh(state, now, options.max_state_age_sec) ||
        (map && state.source != map->metadata().source) ||
        (task && map && !SameEpoch(task_map, map->metadata()))) {
      result.validity = TrajectoryValidity::Invalid;
      result.reason =
          finish ? "runtime stopped"
                 : (!usable ? "map invalid/stale" : "camera state invalid/stale or source changed");
    } else if (result.candidate && result.candidate->plan.trajectory &&
               !SameVersion(result.candidate->validated_on, map->metadata())) {
      if (result.validity == TrajectoryValidity::Ready)
        result.reason = "new map requires curve revalidation";
      result.validity = TrajectoryValidity::PendingValidation;
      result.state = PlanningState::Validating;
    }
    if (result.validity == TrajectoryValidity::PendingValidation) {
      const auto since = output.validity == TrajectoryValidity::Ready && map
                             ? map->metadata().published
                             : pending_since;
      result.pending_ms =
          std::max(0., std::chrono::duration<double, std::milli>(now - since).count());
    }
    return result;
  }
  void Emit(const std::string& kind) {
    if (sink) sink({kind, View(), Clock::now()});
  }
  bool Current(std::uint64_t g, const occ::Metadata& map) const {
    return !finish && task && generation == g && SameEpoch(task_map, map) &&
           Fresh(state, Clock::now(), options.max_state_age_sec) && state.source == map.source;
  }
  void Step() {
    auto snapshot = reader.acquire();
    std::optional<PlanningRequest> request;
    std::shared_ptr<const RuntimeCandidate> candidate;
    std::shared_ptr<std::atomic<bool>> token;
    std::uint64_t g = 0, state_id = 0;
    bool solve = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!snapshot) {
        if (task) Revoke("map invalid/stale; trigger a new task after recovery");
        if (have_map) fsm.onEvent({EventType::MapInvalid, current_map});
        have_map = false;
        output.state = fsm.state();
        return;
      }
      const auto& meta = snapshot->metadata();
      if (have_map && !SameEpoch(current_map, meta))
        Revoke("map source/epoch changed; task cleared");
      current_map = meta;
      have_map = true;
      fsm.onEvent({EventType::MapAvailable, meta});
      if (!Fresh(state, Clock::now(), options.max_state_age_sec) || state.source != meta.source) {
        if (task) Revoke("camera invalid/stale; task cleared");
        fsm.onEvent({EventType::StateInvalid, meta});
        output.state = fsm.state();
        return;
      }
      fsm.onEvent({EventType::StateValid, meta});
      output.state = fsm.state();
      if (!task) return;
      request = task;
      candidate = output.candidate;
      token = cancellation;
      g = generation;
      state_id = frozen_id;
      if (candidate && candidate->plan.trajectory) {
        if (SameVersion(candidate->validated_on, meta)) return;
        if (Clock::now() < next_validation) return;  // 失败复核也合并/限流，不能每20ms重试。
        if (output.validity == TrajectoryValidity::Ready) pending_since = meta.published;
        output.state = PlanningState::Validating;
        output.validity = TrajectoryValidity::PendingValidation;
      } else {
        // 同一失败版本不忙循环；新图合并后按有界频率重新尝试。
        if (g == attempted_generation &&
            ((SameVersion(attempted_on, meta) && !needs_replan) || Clock::now() < next_solve))
          return;
        solve = true;
        needs_replan = false;
        attempted_generation = g;
        attempted_on = meta;
        next_solve = Clock::now() + std::chrono::milliseconds(options.retry_interval_ms);
        fsm.onEvent({EventType::SolveStarted, meta, request->request_id, request->goal_version});
        output.state = PlanningState::Planning;
        output.validity = TrajectoryValidity::PendingValidation;
        output.reason = "planning on one immutable snapshot";
        ++output.solve_count;
      }
    }
    const auto meta = snapshot->metadata();
    WorkBudget budget{Clock::now() + std::chrono::milliseconds(options.budget_ms), token.get()};
    if (solve) {
      auto result = PlannerCore{}.plan(snapshot, *request, config, budget);
      if (budget.status() != WorkStatus::Ready) {
        result.status =
            budget.status() == WorkStatus::Cancelled ? PlanStatus::Cancelled : PlanStatus::Timeout;
        result.reason = "budget exhausted before candidate commit";
        result.trajectory.reset();
      }
      snapshot.reset();  // 必须先释放S_n，下一步才能获取最新S_m；结果无地图租约。
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (!Current(g, meta) || token->load()) {
          ++output.discarded_count;
          return;
        }
        if (result.status != PlanStatus::Success || !result.trajectory) {
          fsm.onEvent({EventType::SolveFailed, meta, request->request_id, request->goal_version});
          output.state = PlanningState::Failed;
          output.validity = TrajectoryValidity::Invalid;
          output.reason = std::string(toString(result.status)) + ": " + result.reason;
        } else {
          auto value = std::make_shared<RuntimeCandidate>();
          value->candidate_id = ++next_candidate;
          value->frozen_state_id = state_id;
          value->request = *request;
          value->plan = std::move(result);
          // 分配/复制后仍须检查；候选不能越过整轮截止时间提交。
          if (budget.status() != WorkStatus::Ready) {
            fsm.onEvent({EventType::SolveFailed, meta, request->request_id, request->goal_version});
            output.state = PlanningState::Failed;
            output.validity = TrajectoryValidity::Invalid;
            output.reason = "Timeout: budget exhausted before candidate commit";
            return;
          }
          next_validation = {};
          output.candidate = value;
          output.validity = TrajectoryValidity::PendingValidation;
          output.reason = "candidate needs latest-map validation";
          fsm.onEvent(
              {EventType::SolveSucceeded, meta, request->request_id, request->goal_version});
          output.state = PlanningState::Validating;
        }
      }
      Emit("SolveFinished");
      return;
    }
    const auto validation_started = Clock::now();
    ValidationReport validation;
    try {
      PlanningMapView view(snapshot, CurveRoi(*snapshot, *candidate->plan.trajectory, config),
                           config.half_extent, budget, config.max_roi_voxels);
      validation = TrajectoryValidator{}.validate(view, *candidate->plan.trajectory, *request,
                                                  config, budget);
    } catch (const std::exception& e) {
      validation.status =
          budget.status() == WorkStatus::Ready
              ? ValidationStatus::Unverified
              : (budget.status() == WorkStatus::Cancelled ? ValidationStatus::Cancelled
                                                          : ValidationStatus::Timeout);
      validation.reason = e.what();
    }
    snapshot.reset();
    // IO与显示都只拿数值结果。提交时再看最新发布，不能把S_m永远当成“当前”。
    auto latest = reader.acquire();
    const bool same = latest && SameVersion(latest->metadata(), meta);
    latest.reset();
    // 大对象复制在状态锁外，复制时间仍计入本次复核预算。
    auto value = std::make_shared<RuntimeCandidate>(*candidate);
    value->validated_on = meta;
    value->latest_validation = validation;
    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!Current(g, meta) || token->load()) {
        ++output.discarded_count;
        return;
      }
      ++output.revalidation_count;
      output.last_validation_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - validation_started).count();
      // 最后一次算法检查之后仍可能被调度挂起；发布Ready前再检查一次。
      if (budget.status() != WorkStatus::Ready) {
        validation.status = budget.status() == WorkStatus::Cancelled ? ValidationStatus::Cancelled
                                                                     : ValidationStatus::Timeout;
        validation.reason = "budget exhausted before Ready commit";
      }
      if (validation.status == ValidationStatus::Valid) {
        next_validation = {};
        output.candidate = value;
        output.validity = same ? TrajectoryValidity::Ready : TrajectoryValidity::PendingValidation;
        output.reason = same ? "curve verified on latest map; static preview only"
                             : "map advanced during validation";
        fsm.onEvent(
            {EventType::ValidationPassed, meta, request->request_id, request->goal_version});
        output.state = same ? PlanningState::Ready : PlanningState::Validating;
        accepted = same && saved_candidate != value->candidate_id;
        if (accepted) saved_candidate = value->candidate_id;
      } else if (validation.status == ValidationStatus::Timeout ||
                 validation.status == ValidationStatus::Cancelled ||
                 validation.status == ValidationStatus::Unverified) {
        next_validation = Clock::now() + std::chrono::milliseconds(options.retry_interval_ms);
        output.validity = TrajectoryValidity::PendingValidation;
        output.reason =
            std::string(toString(validation.status)) + ": latest-map validation incomplete";
      } else {
        fsm.onEvent(
            {EventType::ValidationFailed, meta, request->request_id, request->goal_version});
        output.candidate.reset();
        output.validity = TrajectoryValidity::Invalid;
        output.state = PlanningState::ReplanPending;
        output.reason = std::string(toString(validation.status)) + ": replan required";
        needs_replan = true;  // 保留attempted_generation，所有求解仍服从同一限流。
      }
    }
    Emit(accepted ? "Accepted" : "Revalidated");
  }
  void Run() {
    try {
      std::string last_reason;
      while (!finish) {
        Step();
        auto v = View();
        if (v.reason != last_reason) {
          last_reason = v.reason;
          Emit("Status");
        }
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait_for(lock, std::chrono::milliseconds(20));
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex);
      fatal_error = e.what();
      Revoke(std::string("runtime failure: ") + fatal_error);
      output.state = PlanningState::Failed;
      finish = true;
    }
  }
  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      finish = true;
      cancellation->store(true);
    }
    wake.notify_all();
    if (worker.joinable()) worker.join();
  }
};
PlanningRuntime::PlanningRuntime(occ::MapReader r, PlanningConfig c, RuntimeOptions o,
                                 EventSink sink)
    : impl_(std::make_unique<Impl>(std::move(r), std::move(c), o, std::move(sink))) {}
PlanningRuntime::~PlanningRuntime() { stop(); }
void PlanningRuntime::updateState(const RuntimeState& value) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->finish) return;
  impl_->state = value;
  if (!value.valid || (impl_->task && value.source != impl_->task_map.source))
    impl_->Revoke("tracking/source invalid; task cleared");
  impl_->wake.notify_all();
}
bool PlanningRuntime::submit(PlanningRequest request) {
  auto map = impl_->reader.acquire();
  if (!map) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->SubmitLocked(std::move(request), map->metadata());
}
bool PlanningRuntime::triggerProbe(double start, double end) {
  if (!std::isfinite(start) || !std::isfinite(end) || start <= 0 || end <= start || end > 4.5)
    return false;
  auto map = impl_->reader.acquire();
  if (!map) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  PlanningRequest request;
  request.start.position = impl_->state.T_world_camera * Vec3(0, 0, start);
  request.goal.position = impl_->state.T_world_camera * Vec3(0, 0, end);
  return impl_->SubmitLocked(request, map->metadata());
}
void PlanningRuntime::clear() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->finish) return;
  impl_->Revoke("task cleared by user");
  impl_->wake.notify_all();
}
RuntimeView PlanningRuntime::view() const { return impl_->View(); }
std::string PlanningRuntime::error() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->fatal_error;
}
void PlanningRuntime::stop() {
  if (impl_) impl_->Stop();
}
}  // namespace ego_planner
