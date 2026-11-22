# RK3576模块目录迁移与验收记录

日期：2026-11-16。只调整组织、构建和路径，不修改C++算法行为。

## 目录与兼容约定

当前模块与运行命令见[工程README](../README.md)。保留C++命名空间、公共头文件引用、CLI参数、文件格式和原CTest名称。
完整构建统一为`build/release/{bin,lib}`，离线构建为`build/planner/{bin,lib}`。父项目现有入口名称不变。

## 逐文件迁移清单

| 原位置 | 新位置 |
|---|---|
| `apps/live_source.cpp` | `src/sensor_io/src/live_source.cpp` |
| `apps/live_source.h` | `src/sensor_io/include/live_source.h` |
| `apps/online_planning_session.cpp` | `apps/support/src/online_planning_session.cpp` |
| `apps/online_planning_session.h` | `apps/support/include/online_planning_session.h` |
| `apps/planning_io.cpp` | `apps/support/src/planning_io.cpp` |
| `apps/planning_io.h` | `apps/support/include/planning_io.h` |
| `apps/planning_state_adapter.h` | `apps/support/include/planning_state_adapter.h` |
| `bridge/pose_bridge.cpp` | `src/orb_bridge/src/pose_bridge.cpp` |
| `bridge/pose_bridge.h` | `src/orb_bridge/include/pose_bridge.h` |
| `include/ego_occupancy/current_frame.h` | `src/sensor_io/include/ego_occupancy/current_frame.h` |
| `include/ego_occupancy/depth.h` | `src/sensor_io/include/ego_occupancy/depth.h` |
| `include/ego_occupancy/display.h` | `src/visualization/include/ego_occupancy/display.h` |
| `include/ego_occupancy/display_color.h` | `src/visualization/include/ego_occupancy/display_color.h` |
| `include/ego_occupancy/io.h` | `src/plan_env/include/ego_occupancy/io.h` |
| `include/ego_occupancy/map.h` | `src/plan_env/include/ego_occupancy/map.h` |
| `include/ego_occupancy/options.h` | `apps/support/include/ego_occupancy/options.h` |
| `include/ego_occupancy/planning_layer.h` | `src/visualization/include/ego_occupancy/planning_layer.h` |
| `include/ego_occupancy/worker.h` | `src/sensor_io/include/ego_occupancy/worker.h` |
| `planning/README.md` | `docs/planning-usage.md` |
| `planning/SOURCE.md` | `docs/planning-provenance.md` |
| `planning/include/ego_planner/astar.h` | `src/path_searching/include/ego_planner/astar.h` |
| `planning/include/ego_planner/bspline.h` | `src/bspline/include/ego_planner/bspline.h` |
| `planning/include/ego_planner/collision_constraints.h` | `src/bspline_opt/include/ego_planner/collision_constraints.h` |
| `planning/include/ego_planner/planner_core.h` | `src/plan_manage/include/ego_planner/planner_core.h` |
| `planning/include/ego_planner/planning_config.h` | `src/planner_common/include/ego_planner/planning_config.h` |
| `planning/include/ego_planner/planning_fsm.h` | `src/plan_manage/include/ego_planner/planning_fsm.h` |
| `planning/include/ego_planner/planning_map_view.h` | `src/path_searching/include/ego_planner/planning_map_view.h` |
| `planning/include/ego_planner/planning_request.h` | `src/planner_common/include/ego_planner/planning_request.h` |
| `planning/include/ego_planner/planning_runtime.h` | `src/plan_manage/include/ego_planner/planning_runtime.h` |
| `planning/include/ego_planner/planning_types.h` | `src/planner_common/include/ego_planner/planning_types.h` |
| `planning/include/ego_planner/polynomial_traj.h` | `src/bspline/include/ego_planner/polynomial_traj.h` |
| `planning/include/ego_planner/rebound_optimizer.h` | `src/bspline_opt/include/ego_planner/rebound_optimizer.h` |
| `planning/include/ego_planner/seed_builder.h` | `src/planner_common/include/ego_planner/seed_builder.h` |
| `planning/include/ego_planner/trajectory_validator.h` | `src/plan_manage/include/ego_planner/trajectory_validator.h` |
| `planning/src/astar.cpp` | `src/path_searching/src/astar.cpp` |
| `planning/src/bspline.cpp` | `src/bspline/src/bspline.cpp` |
| `planning/src/collision_constraints.cpp` | `src/bspline_opt/src/collision_constraints.cpp` |
| `planning/src/planner_core.cpp` | `src/plan_manage/src/planner_core.cpp` |
| `planning/src/planning_config.cpp` | `src/planner_common/src/planning_config.cpp` |
| `planning/src/planning_fsm.cpp` | `src/plan_manage/src/planning_fsm.cpp` |
| `planning/src/planning_map_view.cpp` | `src/path_searching/src/planning_map_view.cpp` |
| `planning/src/planning_runtime.cpp` | `src/plan_manage/src/planning_runtime.cpp` |
| `planning/src/polynomial_traj.cpp` | `src/bspline/src/polynomial_traj.cpp` |
| `planning/src/rebound_optimizer.cpp` | `src/bspline_opt/src/rebound_optimizer.cpp` |
| `planning/src/seed_builder.cpp` | `src/planner_common/src/seed_builder.cpp` |
| `planning/src/trajectory_validator.cpp` | `src/plan_manage/src/trajectory_validator.cpp` |
| `planning/tests/astar_test.cpp` | `tests/path_searching/astar_test.cpp` |
| `planning/tests/fixtures.py` | `tests/support/fixtures.py` |
| `planning/tests/fsm_test.cpp` | `tests/plan_manage/fsm_test.cpp` |
| `planning/tests/lbfgs_budget_test.cpp` | `tests/bspline_opt/lbfgs_budget_test.cpp` |
| `planning/tests/map_view_test.cpp` | `tests/path_searching/map_view_test.cpp` |
| `planning/tests/online_session_test.cpp` | `tests/integration/online_session_test.cpp` |
| `planning/tests/optimizer_test.cpp` | `tests/bspline_opt/optimizer_test.cpp` |
| `planning/tests/planner_cli_contract.py` | `tests/integration/planner_cli_contract.py` |
| `planning/tests/planner_test.cpp` | `tests/plan_manage/planner_test.cpp` |
| `planning/tests/probe_contract.py` | `tests/integration/probe_contract.py` |
| `planning/tests/retime_test.cpp` | `tests/plan_manage/retime_test.cpp` |
| `planning/tests/runtime_policy_test.cpp` | `tests/plan_manage/runtime_policy_test.cpp` |
| `planning/tests/runtime_test.cpp` | `tests/plan_manage/runtime_test.cpp` |
| `planning/tests/seed_test.cpp` | `tests/planner_common/seed_test.cpp` |
| `planning/tests/spline_math_test.cpp` | `tests/bspline/spline_math_test.cpp` |
| `planning/tests/test_support.h` | `tests/support/test_support.h` |
| `planning/tests/validator_test.cpp` | `tests/plan_manage/validator_test.cpp` |
| `planning/third_party/SOURCE.md` | `third_party/lbfgs/SOURCE.md` |
| `planning/third_party/lbfgs.hpp` | `third_party/lbfgs/lbfgs.hpp` |
| `planning_tasks/example_gate.yaml` | `config/tasks/example_gate.yaml` |
| `scripts/bridge_abi_probe.cpp` | `scripts/build/bridge_abi_probe.cpp` |
| `scripts/build_orb_occupancy_bridge.py` | `scripts/build/build_orb_occupancy_bridge.py` |
| `scripts/build_orb_stage_profile.py` | `scripts/build/build_orb_stage_profile.py` |
| `scripts/orb_live_common.h` | `src/sensor_io/include/orb_live_common.h` |
| `scripts/orb_rgbd_recording.h` | `src/sensor_io/include/orb_rgbd_recording.h` |
| `scripts/relink_orb_bridge.py` | `scripts/build/relink_orb_bridge.py` |
| `scripts/render_planning_report.py` | `scripts/validation/render_planning_report.py` |
| `scripts/render_trajectory_report.py` | `scripts/validation/render_trajectory_report.py` |
| `scripts/validate_ego_optimizer.py` | `scripts/validation/validate_ego_optimizer.py` |
| `scripts/verify_bridge_abi.py` | `scripts/build/verify_bridge_abi.py` |
| `src/current_frame.cpp` | `src/sensor_io/src/current_frame.cpp` |
| `src/depth.cpp` | `src/sensor_io/src/depth.cpp` |
| `src/display.cpp` | `src/visualization/src/display.cpp` |
| `src/display_color.cpp` | `src/visualization/src/display_color.cpp` |
| `src/io.cpp` | `src/plan_env/src/io.cpp` |
| `src/map.cpp` | `src/plan_env/src/map.cpp` |
| `src/options.cpp` | `apps/support/src/options.cpp` |
| `src/worker.cpp` | `src/sensor_io/src/worker.cpp` |
| `tests/app_contract.py` | `tests/integration/app_contract.py` |
| `tests/bridge_test.cpp` | `tests/orb_bridge/bridge_test.cpp` |
| `tests/current_frame_test.cpp` | `tests/sensor_io/current_frame_test.cpp` |
| `tests/depth_test.cpp` | `tests/sensor_io/depth_test.cpp` |
| `tests/display_color_benchmark.cpp` | `tests/visualization/display_color_benchmark.cpp` |
| `tests/display_color_test.cpp` | `tests/visualization/display_color_test.cpp` |
| `tests/display_depth_render_test.cpp` | `tests/visualization/display_depth_render_test.cpp` |
| `tests/display_planning_test.cpp` | `tests/visualization/display_planning_test.cpp` |
| `tests/display_render_test.cpp` | `tests/visualization/display_render_test.cpp` |
| `tests/display_startup_test.cpp` | `tests/visualization/display_startup_test.cpp` |
| `tests/io_test.cpp` | `tests/plan_env/io_test.cpp` |
| `tests/map_test.cpp` | `tests/plan_env/map_test.cpp` |
| `tests/online_options_test.cpp` | `tests/integration/online_options_test.cpp` |
| `tests/options_test.cpp` | `tests/integration/options_test.cpp` |
| `tests/test_relink.py` | `tests/orb_bridge/test_relink.py` |
| `tests/worker_test.cpp` | `tests/sensor_io/worker_test.cpp` |

## 开发问题与处理

| 问题 | 定位与解决 | 验证 |
|---|---|---|
| 旧构建目录约占项目绝大部分空间 | 新构建验收前保留，验收后仅按明确清单删除 | 目录体积与清理清单 |
| core中的.o仍是构建输入 | 保留core与原ORB对象；不按扩展名批量清理 | 原始对象、源码、历史清单哈希检查 |
| 重链接工具写死旧build及bridge路径 | 新增显式桥接库参数，更新头文件及工具路径，历史依据原文保留 | 先行失败测试后7项通过，真实重链接与ldd核对 |
| 公共采集头文件混在scripts中 | 移到sensor_io/include，C++内容保持不变，由目标导出include路径 | 完整应用构建与预检 |
| 规划夹具原位于planning/tests | 移到tests/support，通过CTest PYTHONPATH提供给CLI测试 | CLI与成组离线验收 |
| 显示测试默认返回跳过 | 需要EGO_TEST_DESKTOP=1及可用的实际X认证；用户明确授权本次临时认证，结束后删除 | 完整36项测试不得跳过 |
| build目录曾被Git跟踪 | 保留用户改动记录，仅移除旧build产物的跟踪，加入忽略规则 | 迁移前后Git清单 |
| build_orb_stage_profile.py名称像遗留工具 | 桥接构建仍导入其once、verify_baseline、sha，保留并注明实际依赖 | Python导入与桥接检查 |

## 验收记录

| 配置／流程 | 结果 |
|---|---|
| 迁移前完整Release实际桌面基线 | 36/36，无跳过 |
| 新布局完整Release与实际桌面 | 36/36，无跳过；原测试名称逐项相同 |
| 无相机／显示的离线规划Release | 25/25，无跳过 |
| 关闭规划及RGB-D的最小构建 | 6/6，无跳过 |
| RelWithDebInfo UBSan＋float-cast-overflow＋标准库断言 | 25/25，无跳过 |
| 桥接路径与拒绝陈旧输入的Python单元测试 | 7/7；修改前已复现不支持新路径 |
| 早期A*入口 | open成功、closed/unknown正确拒绝，图示生成 |
| 优化器成组入口 | 门、绕障、封闭、未知、偏置门、时间调整、真实地图共7项符合预期 |
| 全部C++源码／头文件 | 93个逐字节与板端迁移前一致 |
| 原始ORB核心／历史链接依据 | 保持原哈希；独立桥接核心按新路径重链接 |
| ELF与路径 | 无缺失依赖；不加载已删除构建目录中的库 |
| 清理后入口 | 实时／回放预检、真实地图加载、离线规划输出再次通过 |


## 证据位置

`/home/lckfb/orb-slam3/logs/layout-20261116`：基线哈希、Git改动、测试清单、迁移脚本、C++保留性证明、构建／测试日志、清理清单与进度记录。
原始采集、地图、轨迹均留在父项目data；新增验收数据使用新的目录，不覆盖旧结果。

## 实际桌面回放与实机

- 回放结果：`/home/lckfb/orb-slam3/data/results/layout-20261116-replay`，导出3组有效轨迹，地图重新加载成功。
- 成功实机结果：`/home/lckfb/orb-slam3/data/results/layout-20261116-live-02`，采集／记录486对、处理475帧、475帧定位有效、融合240次、来源失效0次、导出4组有效轨迹。地图重新加载成功。
- 用户实际目视确认：“都能看到，显示正常”，涵盖RGB、深度着色占据地图和青色规划曲线。
- 实机与单线程UBSan编译同时进行，SLAM队列丢弃9帧、同步配对拒绝2次，建图队列丢弃／建图拒绝／快照发布跳过均为0；不把本次冒烟当作性能基准。
- 首次实机朝向下出现跟踪失效及StartBlocked，没有输出有效轨迹。保留initial-live日志与原结果；用户调整为纹理充分、前方空旷并静止后，第二次成功。没有放宽Free、净空或新鲜度条件来制造成功。
- 第二次运行仍出现过0.5秒新鲜度门限触发的失效／任务撤销；目录整理不修复或改变该原有行为。
- 每组导出检查了完成标记、Valid验证结果及全部CSV数值有限性。此验收不代表移动起点连续规划或飞行验收。

## 清理结果

| 删除的构建目录 | 原占用 MiB |
|---|---:|
| `build-rollback-layout` | 7.92 |
| `build-online` | 19.49 |
| `build-planning` | 14.04 |
| `build-planning-asan` | 225.38 |
| `build-planning-off` | 4.02 |
| `build-planning-ubsan` | 701.06 |
| `build/checks` | 702.66 |

删除的可再生构建目录合计约1674.56 MiB；清理后项目约41.72 MiB。该统计不包含父项目日志及新增验收录制。
原data目录15939个文件逐项核对，均存在且内容哈希不变。
保留core/obj及原ORB构建对象，因为独立核心重链接仍使用它们。保留正式测试源码和正式Release／离线构建；删除检查构建、Python缓存和临时回退压缩包。
仅将旧build产物从Git索引移除，新增忽略规则；没有重置、暂存其他源码或提交用户尚未提交的源码改动。分支仍为codex/optimizer-20261115。

## 开发问题补充

| 问题 | 处理与证据 |
|---|---|
| 独立审查发现旧run_planner_foundations入口遗漏 | 修正构建目录、夹具、程序和绘图脚本四类路径；实际执行原入口并生成完整报告 |
| 显示用例默认跳过 | 查明EGO_TEST_DESKTOP开关，显式开启并使用用户单独授权的临时X认证；迁移前后实际桌面均36/36无跳过 |
| 试验场景不能支持静态探针 | 保留失败结果；用户调整场景后独立新目录补验，475/475定位有效及4组轨迹；未修改算法参数 |

## 最终状态

新模块结构、完整功能入口和数据格式已验证；旧构建删除后再次检查入口、地图读入和离线轨迹生成，未依赖旧目录。临时桌面认证已删除。
本次仅完成工程组织整理。原有0.5秒时效门限、固定起点在线静态预览和Unknown拒绝策略保持不变。
