# Orbbec 60 Hz 实时建图与录制解耦实施计划

> **执行方式** 使用 `superpowers:executing-plans` 逐项实现和验证。每项完成本节验证后再开始下一项。

**目标** 在 RK3576 上以 SV1301S_U3 的 60 Hz 模式持续建图。实时处理优先使用最新帧，录制供问题复现，录制积压或录制失败时继续建图。

**架构** 相机帧对仍由现有 Orbbec 后端产生。`LiveSource` 把同一对图像分别交给实时分支和录制分支。实时分支只保留最新的待处理帧，录制分支用单独的队列和线程。ORB-SLAM3 仍按时间顺序处理拿到的帧，地图只融合定位有效的深度帧。

**技术范围** C++17、OpenCV、现有 ORB-SLAM3 位姿桥接、Orbbec SDK v1.10.37、CMake 和 CTest。

**需求依据** 已确认的方向见本文件的「行为约定与验收」。板端两次 60 Hz 运行均以 `recording queue overflow` 退出。一次运行已有 37 帧有效定位和 15 次融合，地图队列丢弃为 0。40 帧限量实测能结束并生成地图，持续运行仍需验收。

## 行为约定与验收

- 使用现有 `SV1301S_U3_RGBD_640x480_60fps_rectified.yaml`，保持 SDK v1 的深度对齐、内参、深度比例和现有 10 ms 配对规则。本计划针对 60 Hz。
- Orbbec 实时分支保留 **1 对待处理帧**。新帧覆盖待处理的旧帧，已经交给 ORB-SLAM3 的帧继续处理。送入 ORB 前检查本机帧龄，超过 **100 ms** 的帧跳过。帧龄从应用拿到 SDK 帧对时的单调时钟算起，用于约束应用内等待和预处理时间。相机曝光到 SDK 返回的延迟需要单独测量。
- Orbbec 默认按 **10 Hz** 选择要存的帧。新增实时模式参数 `--record-hz HZ`，`0` 关闭 RGB-D 图像录制，正数设置目标采样频率，取值范围 **0–90**。例如 `--record-hz 5` 用于少量复现记录。指定 60 仍遵循本次录制分支的尽力保留策略。
- Orbbec 录制队列最多 **8 对待写帧**。满时跳过本次录制并计数，实时分支照常接收。录制线程写入失败时停掉录制分支，诊断中保留原因。相机断流、建图错误或主输出目录不可用时，沿用对应失败处理。对 RealSense，未传新参数时维持原有实时队列及录制策略；显式传参时只启用新的录制策略。
- 保留已有回放格式、成对提交和写入失败回滚。关闭图像录制仍生成位姿、统计和地图产物，并保留相机标定及设备元数据。默认 10 Hz 和 100 ms 是本计划的首轮参数，用板端结果验证。
- 首轮板端验收使用同一相机、配置、场景和存储。默认录制及关闭录制各跑 **120 秒**，其中包含缓慢平移和转向。两轮都应在结束时生成可读地图，后半段仍有新的有效定位和地图更新，送入 ORB 的帧应用内帧龄不超过 100 ms。录制过载和写入失败由可控测试单独验证。

## 文件与职责

| 位置 | 改动 |
|---|---|
| `apps/support/include/ego_occupancy/options.h`、`apps/support/src/options.cpp` | 解析和验证 `--record-hz`，保留是否显式传入的信息。 |
| `src/sensor_io/src/live_source.cpp`、`src/sensor_io/include/live_source.h` | 实时和录制分发、录制错误隔离、停机顺序及分支统计。 |
| `src/sensor_io/include/live_dispatch.h` | 新增内部策略，复用已有 `BoundedQueue`，给录制限频和分支过载行为提供确定性的测试入口。 |
| `src/sensor_io/include/orb_rgbd_recording.h` | 在录制线程内为实际写出的帧计算有效深度比例，保留现有 PNG 和 CSV 格式。 |
| `apps/rgbd_map.cpp` | ORB 跟踪前的实时帧龄检查，以及日志、位姿 CSV 和汇总统计。 |
| `tests/integration/options_test.cpp`、`tests/sensor_io/live_dispatch_test.cpp`、`tests/sensor_io/live_source_test.cpp`、`tests/CMakeLists.txt` | 参数、队列溢出、录制故障及生命周期验证。 |
| `README.md` | 记录实际运行命令、参数及日志判读方法。 |

## 重点审查场景

- SDK 突发输入时，正在处理的一帧之外只有一帧待跟踪。长时间停顿后的过期帧在 ORB 入口被跳过。见任务二。
- 实时队列替换帧时，已经在录制队列里的整对图像仍有效。慢写盘只增加录制丢帧计数。见任务三。
- 录制中途出现写入错误时，已提交的 RGB、深度和清单仍匹配，之后还有有效实时帧。见任务三。
- 停止时先释放相机，再写完有限的录制积压。主动停止和相机断流都有明确的最终状态。见任务四。
- 关闭录制不会启动录制线程，回放调用和 RealSense 原有默认行为仍可用。见任务一、三和五。

## 任务一：确定录制参数

- [ ] 在 `tests/integration/options_test.cpp` 增加测试。实时接受 `--record-hz 0`、`--record-hz 10`，拒绝负数、`nan`、超过 90 和缺值；回放接受原有命令，拒绝带录制参数的命令；未传参保留未设置状态。
- [ ] 运行 `mapping_cli_contract`，确认新增用例在实现前失败。
- [ ] 在 `Options` 增加 `std::optional<double> record_hz`。复用现有有限数值校验，更新帮助文本。有效值由 `LiveSource` 决定，显式值优先，其次 Orbbec 取 10，RealSense 未传时用既有模式。相机的 `Camera.fps` 继续独立读取。
- [ ] 重跑 `mapping_cli_contract` 和 `online_options_contract`，均通过。

## 任务二：实时分支只交付新鲜帧

- [ ] 在 `tests/sensor_io/live_dispatch_test.cpp` 测试依次推入帧 1、2、3，在消费者取帧前积压，第一次取到的是帧 3，覆盖计数为 2。取走帧 3 后继续推入帧 4，消费者持有的帧 3 的数据保持有效。
- [ ] 用固定单调时钟值验证帧龄边界。送入 ORB 时帧龄为 100 ms 的帧可以通过；超过 100 ms 的帧计为过期并跳过。测试无需真实睡眠。
- [ ] 实现内部 `orb_live::LiveDispatch::publish(FramePtr)` 和 `next(FramePtr&, std::chrono::milliseconds)`，复用已有有界队列。Orbbec 队列容量取 1，RealSense 默认容量取 2。`LiveSource::next` 委托给分发对象。
- [ ] 实现内部判定 `orb_live::isFreshForTracking(const RGBDFrame&, std::chrono::steady_clock::time_point now)`。在 `apps/rgbd_map.cpp` 紧靠 `TrackRGBD` 调用前使用它，仅应用于 Orbbec 实时帧。过期帧不调用 ORB、不提交地图；继续取下一帧。现有 SLAM 跟踪状态、地图过期和相机超时仍负责报告连续无有效数据的情况。
- [ ] 注册并通过 `live_dispatch_contract`。在一次回放验收中核对回放帧不会被实时帧龄检查过滤。

## 任务三：录制限频并隔离错误

- [ ] 给 `live_dispatch_contract` 添加限频测试。用 1 秒内的 60 对规则采样，10 Hz 目标应选出 10 对；长时间无输入后只接收当前一对，不补写缺失时间槽；0 Hz 不接收。按首个有效帧的 `color_sec` 建立时间槽，跨入新槽时只选该槽的第一对。
- [ ] 添加过载测试。让录制消费者暂停，在 8 对等待写入后继续送入有效帧。超过容量的帧只增加录制丢弃计数，实时分支仍能不断拿到最新帧。测试实际被录制队列接受的帧即使在实时分支被覆盖，也还能用于录制。
- [ ] 为 `tests/sensor_io/live_source_test.cpp` 注入可控的假相机与写入故障。测试目标可单独编译 `live_source.cpp`，用测试端的 `openOrbbec` 提供帧，不需要 SDK 或相机。用临时输出目录模拟第二对图像提交失败，验证首对清单和两张图都还在，随后仍能收到实时帧。另测关闭录制时没有 PNG 和录制清单。
- [ ] 向 `LiveDispatch` 接入现有录制队列，先发布实时帧，再按频率尝试录制入队。复用只读共享帧数据。录制入队成功、策略跳过及队列满分别计数。保留 RealSense 未指定录制参数时的旧策略。
- [ ] `LiveSource::Impl` 仅在启用录制时构造写入器和启动线程。向 `SequenceWriter::write(const RGBDFrame&)` 数值计算位置移入当前的 `cv::countNonZero`，避免为只建图的帧扫描一次整张深度图，也避免录制线程修改实时分支共享的帧数据。
- [ ] 将录制故障与采集故障分别保存。Orbbec 录制初始化或写入失败时关闭录制队列、释放未写帧并停止接收新录制任务，保留一次明确告警。已提交文件仍按原有方式回放。主输出目录、位姿文件和地图文件的错误沿用对应处理。
- [ ] 注册并通过 `live_source_contract`。回放至少一段录制结果，确认现有加载器能读取原格式的 PNG 与关联清单。

## 任务四：统计与停机

- [ ] 在 `LiveStats` 和 `summary.yml` 中加入 `record_eligible`、`record_skipped_by_rate`、`record_queue_dropped`、`record_queue_high_water`、`recording_failure`。队列峰值在入队修改队列长度时更新，只有容量已满才计为 `record_queue_dropped`。保留 `captured`、`recorded`、`slam_queue_dropped` 和 `failure` 原字段。`failure` 仍用于导致建图失败的错误。
- [ ] 在 `poses.csv` 末尾追加 `slam_input_host_age_ms`，在实时进度和汇总里报告 `slam_stale_dropped`。每秒进度同时显示 `captured`、`verified`、`fused` 和录制丢弃数。让实时进度在只有过期帧或临时无帧时也能报告，避免跟踪停顿时只能等退出才看到数据。
- [ ] 用假相机验证正常退出顺序。请求停采 → 等采集线程结束 → 停 SDK 管线 → 关闭并排空录制队列。测试让写入器延后结束，确认 SDK 已收到停止调用。相机断流时，仍能从最终汇总和退出码识别真正的采集失败。
- [ ] 验证记录采样跳过、队列满、录制写入失败及实时帧过期可以分别判断。测试核对计数，没有写入的数据不会算作 `recorded`。

## 任务五：在 RK3576 验收

- [ ] 先在工作站跑不依赖相机的配置和测试。测试目标用假后端，与实际 Orbbec SDK 编译验证分开。

  ```bash
  cmake -S . -B build/live-policy-check \
    -DEGO_BUILD_RGBD=ON -DEGO_BUILD_ORB=OFF \
    -DEGO_BUILD_REALSENSE=OFF -DEGO_BUILD_ORBBEC=OFF \
    -DBUILD_TESTING=ON
  cmake --build build/live-policy-check --parallel 2
  ctest --test-dir build/live-policy-check --output-on-failure \
    -R 'mapping_cli_contract|online_options_contract|live_dispatch_contract|live_source_contract|host_pair_gate_contract'
  ```

- [ ] 在板端用已配置的 `build/release` 编译，确认链接仍指向 `/home/lckfb/OrbbecSDK_v1.10.37/SDK/lib`。使用同一脚本和相机 YAML 运行下列两轮，优先在相同的有纹理场景完成。板端脚本已经能把这些参数转发给程序。

  ```bash
  cd /home/lckfb/orb-slam3
  camera="$PWD/ego-occupancy-20261112/config/cameras/SV1301S_U3_RGBD_640x480_60fps_rectified.yaml"

  bash scripts/run_ego_occupancy.sh --camera-backend orbbec \
    --camera "$camera" --max-seconds 120 --no-viewer
  bash scripts/run_ego_occupancy.sh --camera-backend orbbec \
    --camera "$camera" --max-seconds 120 --no-viewer --record-hz 0
  ```

- [ ] 保存两轮的 `command.txt`、`run.log`、`summary.yml` 及 `poses.csv`。两轮都检查退出码为 0、`failure` 为空、`has_snapshot=1`，且后 60 秒仍有有效位姿和融合，同时记录 `slam_stale_dropped`。ORB 实际输入帧龄的最大值应在 100 ms 预算内，P95 作为后续板端比较基线。
- [ ] 检查默认录制的写入率、峰值队列长度和丢帧数。若后台录制经常丢帧，把它与关闭录制时的 SLAM 频率、帧龄以及 SDK 帧号跳变对比。关闭录制时 `recorded=0`，仍有相机元数据和可读地图。默认录制产物用现有 `--dataset` 至少做一次回放预检。
- [ ] 更新 README 的 Orbbec 运行示例，说明 `--record-hz 0`、默认 10 Hz、汇总计数和回放方式。只有当板端数据显示录制仍显著增加实时帧龄，才另做写入性能对照。当前交付保留回放格式；替换容器格式或引入并行编码可单独评估。

## 板端实施结果

RK3576 的日志与产物位于 `/home/lckfb/ego-60hz-verify.kxjd4Y`。板端远程验收使用静置相机。

| 场景 | 结果 |
|---|---|
| 60 Hz，默认抽样录制，120 秒 | 退出码 0，采集 5500 对、录制 1038 对、融合 802 次、录制丢弃 0。ORB 应用内输入帧龄 P95 为 29.08 ms，最大 65.15 ms。 |
| 60 Hz，尝试以 60 Hz 录制，30 秒压力测试 | 录制队列达到 8 对，溢出跳过 789 对。仍有 222 次融合、生成快照、退出码 0。 |
| 回放默认抽样录制的前 40 对 | 40 帧有效定位、25 次融合，退出码 0。录制和回放校验均通过。 |
| 关闭录制 | 多次因连续 5 秒没有通过配对的帧而提前退出。带诊断的运行确认 SDK 没有取帧超时，也没有不完整帧组；301 对因主机接收时差超过 10 ms 被拒绝，观测时差为 10.276–23.176 ms。移动场景及不间断 120 秒关闭录制仍待验证。 |

工作站 CTest 共 34 项通过。最终板端录制、队列、故障隔离和配置预检共 4 项通过。现有 10 ms 配对规则仍用于 60 Hz。
