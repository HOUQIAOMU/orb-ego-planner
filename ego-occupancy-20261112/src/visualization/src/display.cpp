#include <ego_occupancy/display.h>
#include <pangolin/pangolin.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace ego_occupancy {
namespace {

constexpr const char* kWindowTitle = "EGO Occupancy - raw / inflated";

void DrawPolyline(const std::vector<Vec3>& points, const Rgb8& color, float width) {
  if (points.empty()) return;
  glLineWidth(width);
  glColor3ub(color[0], color[1], color[2]);
  glBegin(GL_LINE_STRIP);
  for (const auto& p : points) glVertex3dv(p.data());
  glEnd();
}
void DrawPlanning(const PlanningLayer& layer) {
  if (!layer.has_task) return;
  const Rgb8 unavailable{130, 130, 130};
  DrawPolyline(layer.initial, layer.ready ? Rgb8{180, 180, 180} : unavailable, 1);
  for (const auto& path : layer.astar)
    DrawPolyline(path, layer.ready ? Rgb8{255, 150, 20} : unavailable, 2);
  DrawPolyline(layer.trajectory, layer.ready ? Rgb8{0, 255, 255} : unavailable, 4);
  glPointSize(10);
  glBegin(GL_POINTS);
  glColor3ub(80, 255, 80);
  glVertex3dv(layer.start.data());
  glColor3ub(255, 80, 80);
  glVertex3dv(layer.goal.data());
  glEnd();
}

struct WindowCleanup {
  const char* title;
  ~WindowCleanup() { pangolin::DestroyWindow(title); }
};

std::string FormatXyz(const Vec3& point) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << point.x() << ", " << point.y() << ", "
         << point.z();
  return stream.str();
}

const char* TrackingStateName(int state) {
  switch (state) {
    case 0:
      return "NO IMAGES";
    case 1:
      return "INITIALIZING";
    case 2:
      return "OK";
    case 3:
      return "RECENTLY LOST";
    case 4:
      return "LOST";
    default:
      return "UNAVAILABLE";
  }
}

const char* ColorModeName(ColorMode mode) {
  switch (mode) {
    case ColorMode::CameraDepth:
      return "Camera Z depth [m]";
    case ColorMode::MapAxis:
      return "Map -Y [m], not gravity";
    case ColorMode::Flat:
      return "Flat color";
  }
  return "Unknown color mode";
}

double Milliseconds(TimePoint start, TimePoint end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// Legacy-compatible vertex arrays avoid a GL call for every map point/color.
void DrawPoints(const std::vector<Vec3>& points, const std::vector<Rgb8>* colors, Rgb8 uniform,
                float point_size) {
  if (points.empty()) {
    return;
  }
  if (points.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
      (colors && colors->size() != points.size())) {
    throw std::runtime_error("invalid display vertex/color array size");
  }
  static_assert(sizeof(Rgb8) == 3, "OpenGL expects packed RGB bytes");
  glPointSize(point_size);
  glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_DOUBLE, sizeof(Vec3), points.front().data());
  if (colors) {
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(Rgb8), colors->data());
  } else {
    glDisableClientState(GL_COLOR_ARRAY);
    glColor3ubv(uniform.data());
  }
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));
  glPopClientAttrib();
}

void DrawColorLegend(ColorRange range, bool reverse, bool available, const std::string& label,
                     int width, int height) {
  glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, width, 0, height, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  constexpr float left = 265;
  constexpr float bottom = 32;
  constexpr float bar_width = 240;
  glBegin(GL_QUADS);
  for (int i = 0; i < 64; ++i) {
    const auto color = available ? rainbowColor(i / 63.0, reverse) : kUnavailableColor;
    glColor3ubv(color.data());
    const float x = left + i * bar_width / 64;
    const float right = left + (i + 1) * bar_width / 64;
    glVertex2f(x, bottom);
    glVertex2f(right, bottom);
    glVertex2f(right, bottom + 12);
    glVertex2f(x, bottom + 12);
  }
  glEnd();
  // Pangolin glyph textures carry alpha; GlText expects the caller to blend it.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor3f(1, 1, 1);
  pangolin::GlFont::I().Text(label).Draw(left, bottom + 22);
  if (range.valid()) {
    std::ostringstream minimum;
    std::ostringstream maximum;
    minimum << std::fixed << std::setprecision(2) << range.minimum_m << " m";
    maximum << std::fixed << std::setprecision(2) << range.maximum_m << " m";
    pangolin::GlFont::I().Text(minimum.str()).Draw(left, bottom - 18);
    pangolin::GlFont::I().Text(maximum.str()).Draw(left + bar_width - 60, bottom - 18);
  }
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopAttrib();
}

void DrawCamera(const Eigen::Isometry3d& pose, const CameraModel& camera) {
  const auto frustum = cameraFrustum(pose, camera, 0.25);
  glDisable(GL_DEPTH_TEST);
  glLineWidth(3);
  glColor3f(1.0F, 0.85F, 0.1F);
  glBegin(GL_LINES);
  for (int i = 1; i <= 4; ++i) {
    glVertex3dv(frustum[0].data());
    glVertex3dv(frustum[i].data());
    glVertex3dv(frustum[i].data());
    glVertex3dv(frustum[i % 4 + 1].data());
  }
  for (int axis = 0; axis < 3; ++axis) {
    glColor3f(axis == 0, axis == 1, axis == 2);
    Vec3 delta = Vec3::Zero();
    delta[axis] = 0.35;
    const Vec3 tip = pose * delta;
    glVertex3dv(frustum[0].data());
    glVertex3dv(tip.data());
  }
  glEnd();
  glEnable(GL_DEPTH_TEST);
}

}  // namespace

struct OccupancyDisplay::Impl {
  MapReader reader;
  CameraModel calibration;
  CurrentFrameBuffer frames;
  std::filesystem::path screenshot;
  double max_pose_age = 0.0;
  DisplaySettings settings;
  std::thread thread;
  std::atomic<bool> finish{false};
  std::atomic<bool> done{false};
  mutable std::mutex mutex;
  std::string failure;

  Impl(MapReader map_reader, const CameraModel& camera,
       const std::filesystem::path& screenshot_path, double pose_age,
       const DisplaySettings& display_settings)
      : reader(std::move(map_reader)),
        calibration(camera),
        frames(camera),
        screenshot(screenshot_path),
        max_pose_age(pose_age),
        settings(display_settings) {
    thread = std::thread([this] { Run(); });
  }

  void Run() {
    try {
      constexpr int kWidth = 1280;
      constexpr int kHeight = 800;

      pangolin::CreateWindowAndBind(kWindowTitle, kWidth, kHeight);
      const WindowCleanup window{kWindowTitle};
      glEnable(GL_DEPTH_TEST);

      pangolin::OpenGlRenderState camera(
          pangolin::ProjectionMatrix(kWidth, kHeight, 650, 650, kWidth / 2, kHeight / 2, 0.1, 100),
          pangolin::ModelViewLookAt(0, -4, -5, 0, 0, 1, pangolin::AxisNegY));
      auto& view = pangolin::CreateDisplay()
                       .SetBounds(0.0, 1.0, pangolin::Attach::Pix(240), 1.0,
                                  -kWidth / static_cast<double>(kHeight))
                       .SetHandler(new pangolin::Handler3D(camera));
      auto& rgb_view = pangolin::CreateDisplay().SetBounds(
          0.0, 0.40, 0.66, 1.0, calibration.width / static_cast<double>(calibration.height));
      pangolin::CreatePanel("map").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(240));
      pangolin::Var<bool> plan_button("map.Plan static P", false, false);
      pangolin::Var<bool> clear_button("map.Clear plan C", false, false);
      pangolin::Var<std::string> plan_status("map.Plan status",
                                             settings.planning_layer ? "WAITING" : "DISABLED");
      pangolin::Var<std::string> plan_reason("map.Plan reason", "");
      if (settings.trigger_planning)
        pangolin::RegisterKeyPressCallback('p', settings.trigger_planning);
      if (settings.clear_planning) pangolin::RegisterKeyPressCallback('c', settings.clear_planning);
      pangolin::Var<bool> inflate("map.Show inflated", false, true);
      pangolin::Var<bool> slice("map.Show free slice", false, true);
      pangolin::Var<int> color_mode("map.Color mode", static_cast<int>(settings.color_mode), 0, 2);
      pangolin::Var<std::string> modes("map.Modes", "0 Depth / 1 Axis / 2 Flat");
      pangolin::Var<double> near_depth("map.Near m", settings.depth_range.minimum_m, 0.0, 10.0);
      pangolin::Var<double> far_depth("map.Far m", settings.depth_range.maximum_m, 0.0, 10.0);
      pangolin::Var<bool> reverse("map.Reverse colors", settings.reverse, true);
      pangolin::Var<double> point_size("map.Point pixels", 3.0, 1.0, 8.0);
      pangolin::Var<std::string> color_label("map.Color status", "WAITING");
      std::ofstream metrics;
      if (!settings.metrics_path.empty()) {
        metrics.open(settings.metrics_path);
        if (!metrics) {
          throw std::runtime_error("cannot open display metrics");
        }
        metrics << "elapsed_sec,frame,map_sequence,mode,points,prepare_ms,color_ms,"
                   "draw_submit_ms,finish_ms,map_usable,camera_valid\n";
      }
      const auto display_started = Clock::now();
      std::size_t rendered = 0;
      pangolin::Var<std::string> map_label("map.Map", "WAITING");
      pangolin::Var<std::string> pose_label("map.Camera", "WAITING");
      pangolin::Var<std::string> xyz_label("map.XYZ m", "--");
      pangolin::Var<std::string> tracking_label("map.Tracking", "WAITING");
      pangolin::Var<std::string> frame_label("map.RGB frame", "--");

      // Texture construction/destruction both happen inside the GL owning thread.
      {
        pangolin::GlTexture texture(calibration.width, calibration.height, GL_RGB, false, 0, GL_BGR,
                                    GL_UNSIGNED_BYTE);
        std::uint64_t epoch = ~0ULL;
        std::uint64_t sequence = ~0ULL;
        std::uint64_t uploaded = ~0ULL;
        std::uint64_t saved_sequence = ~0ULL;
        std::vector<Vec3> raw;
        std::vector<Rgb8> colors;
        ColorRange axis_range;
        bool saved_color_available = false;
        std::vector<Vec3> expanded;
        std::vector<Vec3> free_slice;
        std::uint64_t saved_plan_version = ~0ULL;
        bool saved_plan_ready = false;
        bool saved = false;
        bool has_image = false;

        while (!finish && !pangolin::ShouldQuit()) {
          if (pangolin::Pushed(plan_button) && settings.trigger_planning)
            settings.trigger_planning();
          if (pangolin::Pushed(clear_button) && settings.clear_planning) settings.clear_planning();
          const auto prepare_started = Clock::now();
          auto snapshot = reader.latestForDisplay();
          if (!snapshot) {
            raw.clear();
            expanded.clear();
            free_slice.clear();
            epoch = ~0ULL;
            sequence = ~0ULL;
          }
          if (snapshot &&
              (snapshot->metadata().epoch != epoch || snapshot->metadata().sequence != sequence)) {
            epoch = snapshot->metadata().epoch;
            sequence = snapshot->metadata().sequence;
            axis_range = {
                -snapshot->origin().y() - snapshot->dimensions().y() * snapshot->resolution(),
                -snapshot->origin().y()};
            raw.clear();
            expanded.clear();
            free_slice.clear();
            snapshot->forEach(Index::Zero(), snapshot->dimensions(),
                              [&raw, &expanded, &free_slice, &snapshot](
                                  const Index& index, const Vec3& point, VoxelQuery query) {
                                if (query.state == VoxelState::Occupied) {
                                  raw.push_back(point);
                                }
                                if (query.inflated) {
                                  expanded.push_back(point);
                                }
                                if (query.state == VoxelState::Free &&
                                    index.y() == snapshot->dimensions().y() / 2) {
                                  free_slice.push_back(point);
                                }
                              });
          }

          const auto prepare_finished = Clock::now();

          // Sample the frame/clock and source state after expensive voxel preparation.
          const auto sample = frames.sample();
          const auto& frame = sample.frame;
          const auto now = sample.sampled_at;
          const auto status = reader.status();
          const bool usable = reader.isUsable(snapshot, now);
          map_label = usable ? std::string("LIVE")
                             : (status.valid && status.accepting
                                    ? std::string("STALE: age limit")
                                    : std::string("INVALID: ") + status.reason);

          std::optional<Eigen::Isometry3d> pose;
          if (frame) {
            if (snapshot) {
              pose = cameraPoseForMap(*frame, *snapshot, status, now, max_pose_age);
            }
            const double age = std::chrono::duration<double>(now - frame->received).count();
            tracking_label = std::string(TrackingStateName(frame->pose.tracking_state));
            frame_label =
                std::to_string(frame->capture_index) + (age > max_pose_age ? " STALE" : "");
            if (pose) {
              pose_label = std::string("LIVE");
              xyz_label = FormatXyz(pose->translation());
            } else {
              xyz_label = std::string("--");
              if (!frame->pose_verified || !ego_pose_bridge::IsCurrent(frame->pose)) {
                pose_label = std::string("INVALID: tracking/source");
              } else if (age < 0 || age > max_pose_age) {
                pose_label = std::string("STALE: age limit");
              } else {
                pose_label = std::string("WAITING: matching map");
              }
            }

            if (uploaded != frame->capture_index) {
              cv::Mat image = frame->bgr.clone();
              cv::rectangle(image, cv::Rect(0, 0, image.cols, std::min(30, image.rows)),
                            cv::Scalar(0, 0, 0), cv::FILLED);
              const std::string title = "CURRENT FRAME " + std::to_string(frame->capture_index) +
                                        "  " + TrackingStateName(frame->pose.tracking_state);
              cv::putText(image, title, cv::Point(8, 21), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
              glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
              texture.Upload(image.data, GL_BGR, GL_UNSIGNED_BYTE);
              uploaded = frame->capture_index;
              has_image = true;
            }
          }
          snapshot.reset();
          const auto planning =
              settings.planning_layer ? settings.planning_layer() : PlanningLayer{};
          if (settings.planning_layer) {
            plan_status = planning.status;
            plan_reason = planning.reason;
          }
          const auto mode = static_cast<ColorMode>(std::clamp(static_cast<int>(color_mode), 0, 2));
          const ColorRange range =
              mode == ColorMode::MapAxis ? axis_range : ColorRange{near_depth, far_depth};
          const bool valid_range =
              range.valid() && (mode != ColorMode::CameraDepth || range.minimum_m >= 0);
          const bool color_available =
              usable && (mode == ColorMode::Flat ||
                         (valid_range && (mode != ColorMode::CameraDepth || pose)));
          const std::string color_name = ColorModeName(mode);
          if (!usable) {
            color_label = std::string("GRAY: map unavailable");
          } else if (mode != ColorMode::Flat && !valid_range) {
            color_label = std::string("GRAY: invalid color range");
          } else if (!color_available) {
            color_label = std::string("GRAY: camera unavailable");
          } else {
            color_label = color_name;
          }
          const auto color_started = Clock::now();
          colorizePoints(raw, mode, range, reverse, pose, usable, colors);
          const auto color_finished = Clock::now();

          glClearColor(0.08F, 0.09F, 0.11F, 1.0F);
          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
          view.Activate(camera);

          glLineWidth(2);
          glBegin(GL_LINES);
          glColor3f(1, 0, 0);
          glVertex3f(0, 0, 0);
          glVertex3f(0.5F, 0, 0);
          glColor3f(0, 1, 0);
          glVertex3f(0, 0, 0);
          glVertex3f(0, 0.5F, 0);
          glColor3f(0, 0, 1);
          glVertex3f(0, 0, 0);
          glVertex3f(0, 0, 0.5F);
          glEnd();

          DrawPoints(raw, &colors, kFlatColor, static_cast<float>(point_size));
          if (inflate) {
            const Rgb8 color = color_available ? Rgb8{255, 89, 51} : kUnavailableColor;
            DrawPoints(expanded, nullptr, color, 2);
          }
          if (slice) {
            const Rgb8 color = color_available ? Rgb8{77, 153, 77} : kUnavailableColor;
            DrawPoints(free_slice, nullptr, color, 2);
          }

          DrawPlanning(planning);
          if (pose) {
            DrawCamera(*pose, calibration);
          }
          if (has_image) {
            rgb_view.Activate();
            glDisable(GL_DEPTH_TEST);
            glColor3f(1, 1, 1);
            texture.RenderToViewportFlipY();
            glEnable(GL_DEPTH_TEST);
          }

          if (mode != ColorMode::Flat) {
            DrawColorLegend(range, reverse, color_available,
                            color_available ? color_name : std::string(color_label), kWidth,
                            kHeight);
          }
          const auto draw_finished = Clock::now();

          // Read back custom 3D/RGB content before swapping. The UI panel is
          // rendered by FinishFrame.
          if ((!raw.empty() || planning.has_task) && !screenshot.empty() &&
              (!saved || planning.version != saved_plan_version ||
               planning.ready != saved_plan_ready || color_available != saved_color_available ||
               (sequence % 10 == 0 && sequence != saved_sequence))) {
            cv::Mat image(kHeight, kWidth, CV_8UC3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, kWidth, kHeight, GL_RGB, GL_UNSIGNED_BYTE, image.data);
            cv::flip(image, image, 0);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            if (!cv::imwrite(screenshot.string(), image)) {
              throw std::runtime_error("cannot save occupancy screenshot");
            }
            saved_plan_version = planning.version;
            saved_plan_ready = planning.ready;
            saved = true;
            saved_sequence = sequence;
            saved_color_available = color_available;
          }

          const auto finish_started = Clock::now();
          pangolin::FinishFrame();
          const auto finish_finished = Clock::now();
          if (metrics.is_open()) {
            metrics << std::chrono::duration<double>(finish_finished - display_started).count()
                    << ',' << (frame ? frame->capture_index : 0) << ',' << sequence << ','
                    << static_cast<int>(mode) << ',' << raw.size() << ','
                    << Milliseconds(prepare_started, prepare_finished) << ','
                    << Milliseconds(color_started, color_finished) << ','
                    << Milliseconds(color_finished, draw_finished) << ','
                    << Milliseconds(finish_started, finish_finished) << ',' << usable << ','
                    << static_cast<bool>(pose) << '\n';
            if (++rendered % 10 == 0) {
              metrics.flush();
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex);
      failure = error.what();
    }
    done = true;
  }
};

OccupancyDisplay::OccupancyDisplay(MapReader reader, const CameraModel& camera,
                                   const std::filesystem::path& screenshot, double max_pose_age,
                                   const DisplaySettings& settings)
    : impl_(std::make_unique<Impl>(std::move(reader), camera, screenshot, max_pose_age, settings)) {
}

OccupancyDisplay::~OccupancyDisplay() { stop(); }

bool OccupancyDisplay::closed() const { return impl_->done; }

void OccupancyDisplay::stop() {
  impl_->finish = true;
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

std::string OccupancyDisplay::error() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->failure;
}

void OccupancyDisplay::publishCurrentFrame(const cv::Mat& bgr, const ego_pose_bridge::Record& pose,
                                           std::uint64_t capture_index, TimePoint received,
                                           bool verified) {
  impl_->frames.publish(bgr, pose, capture_index, received, verified);
}

}  // namespace ego_occupancy
