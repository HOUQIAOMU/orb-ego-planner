# RK3576 占据地图：按当前相机深度着色

板端工程：`/home/lckfb/orb-slam3/ego-occupancy-20261112`。本次以板端当前源码为准原位开发，增加相机深度配色、色条、显示控制和耗时记录。

## 使用

```bash
sh /home/lckfb/orb-slam3/scripts/run_ego_occupancy.sh \
  --max-seconds 60 --stride 4 --display-color depth
```

`depth` 为默认模式。离线回放同样支持：

```bash
sh /home/lckfb/orb-slam3/scripts/run_ego_occupancy.sh \
  --dataset /完整/录制/rgbd --camera /完整/录制/rgbd/D435i.yaml \
  --max-frames 120 --max-seconds 0 --stride 4 --display-color depth
```

窗口控制：

| 控件 | 含义 |
| --- | --- |
| Color mode = 0 | 当前相机光学 Z 深度 |
| Color mode = 1 | 固定地图 −Y 坐标分层；不是重力高度 |
| Color mode = 2 | 单色显示 |
| Near m / Far m | 深度模式的近远色阶，默认 0.1～4.5 米 |
| Reverse colors | 反转近远配色 |
| Point pixels | 调整体素点大小，默认 3 像素 |
| Show inflated | 独立显示橙色膨胀层，便于区分实际障碍与碰撞余量 |
| Show free slice | 显示已有的空闲切片 |

也可用 `--display-color axis` 或 `--display-color flat` 选择启动模式。固定轴模式按当前网格世界坐标边界配色，Near/Far 控件仅影响深度模式。非法色阶范围会灰显并提示，不沿用旧颜色。

## 颜色的物理含义

对每个原始占据体素中心，用有效的 `T_world_camera` 计算：

```text
z_camera = R_world_camera.col(2) · (point_world − position_world_camera)
```

取光轴 Z 深度，默认红近、黄/绿居中、青/蓝远；正深度超出色阶范围时饱和。相机背后的体素（Z≤0）、非有限输入，以及缺少有效当前位姿时使用灰色。

位姿必须通过现有同帧验证、地图来源/epoch/修正版本和时效检查。地图失效时各地图层灰显；RGB 画面仍可显示。历史融合地图中的体素不一定当前可见，颜色不代表已经通过遮挡检测。

## 代码组织与开销控制

- `include/ego_occupancy/display_color.h`、`src/display_color.cpp`：独立 Eigen/标准库模块，定义模式、范围和 RGB 查表，处理坐标与异常输入。
- `include/ego_occupancy/display.h`、`src/display.cpp`：消费颜色结果，使用 OpenGL 顶点/颜色数组批量绘制，管理 GUI、图例与显示线程资源。
- `src/options.cpp`、`apps/rgbd_map.cpp`：启动模式校验和显示设置传递。
- `tests/display_color_test.cpp`：平移、旋转、光学深度、截断、反转、失效与备用模式契约。
- `tests/display_depth_render_test.cpp`：真实桌面上的体素颜色、位姿过期灰显与图例字形透明区域验证。
- `tests/display_color_benchmark.cpp`：可复现的颜色计算测量，包含实际地图和十万点压力样本。

复用已缓存占据点和颜色缓冲，显示最多跟随原有约 10 Hz 刷新；不为颜色增加一次完整体素遍历。建图算法、完整体素查询、序列化和 ORB 桥接保持原实现。原 ORB 与独立桥接 ORB 核心均通过开发前后哈希检查。

## 板端测量

以下为颜色计算本身，排除首次分配和查表初始化，不包含完整绘制或 SLAM：

| 点数 | 单色 P50 | 相机深度 P50 / P95 | RGB 缓冲大小 |
| --- | ---: | ---: | ---: |
| 实际保存地图的 6,828 点 | 0.017 ms | 0.185 / 0.208 ms | 20,484 字节 |
| 十万点 | 0.186 ms | 2.213 / 2.261 ms | 300,000 字节 |

同一段录制的两个 120 帧回放均为 120/120 位姿验证、60 次融合、建图队列淘汰 0；严格帧/来源/时间戳校验通过。有效显示期间：

| 指标 | 单色 | 相机深度 |
| --- | ---: | ---: |
| 占据点数 P50 | 3,542 | 3,437 |
| 颜色计算 P50 / P95 | 0.009 / 0.014 ms | 0.089 / 0.155 ms |
| 绘制提交 P50 | 0.479 ms | 0.656 ms |
| 地图准备 P50 | 30.484 ms | 30.101 ms |
| 地图年龄最大值 | 150.70 ms | 153.35 ms |

两次 SLAM 运行的姿态、地图和调度并不完全一致，不能用总延迟或 RSS 的差值证明显示的因果开销；独立颜色测量更适合评估新增计算。当前地图准备的完整网格遍历明显比着色耗时长，本次没有借机改动该算法路径。

每次启用显示会在数据目录生成 `display.csv`，记录点数、颜色模式、地图/相机有效性，以及地图准备、颜色计算、绘制提交、窗口交换耗时。`draw_submit_ms` 不等于 GPU 完整执行时间，也不包含截图 PNG 编码。

## 验证与注意事项

本次实际 ARM 桌面全部 14 项 CTest 已执行通过，包含原 RGB 原色、窗口异常清理等测试。实机初轮 20 秒测试保存 260/260 对 RGB-D，259/259 位姿有效、130 次融合、退出码 0；颜色计算 P50 0.097 ms，地图年龄最大 254.88 ms。完整地图重载为 2,048,000 体素：Unknown 1,959,552 / Free 84,111 / Occupied 4,337。

截图检查发现图例字体透明混合遗漏，已定位到 Pangolin GlText 需要调用方启用 alpha 混合；修复时同时保存/恢复相应 GL 状态。最终构建、测试和实机结果见文末交付记录。

`occupancy.png` 是周期采样：首次、颜色可用性变化或每十次地图发布时保存。窗口中的颜色实时更新，但仅相机运动或切换颜色参数不一定立即更新该 PNG；需要当前完整 UI 时，以实际窗口截图为准。这是本次审阅保留的一项低优先级改进。

当前验证是显示功能和数据链路验证，不包含新的开口尺寸、定位真值或飞行验收。

## 重建

本次显示改动可直接增量编译，不需要重新链接 ORB 核心：

```bash
cmake --build /home/lckfb/orb-slam3/ego-occupancy-20261112/build -j1
EGO_TEST_DESKTOP=1 DISPLAY=:0 XAUTHORITY=/home/lckfb/.Xauthority \
  ctest --test-dir /home/lckfb/orb-slam3/ego-occupancy-20261112/build --output-on-failure
```

完整 EGO 构建入口继续是 `/home/lckfb/orb-slam3/scripts/build_ego_occupancy.sh`。使用新机器或空构建缓存时，按该入口的依赖顺序执行。

## 源码与证据

- 板端构建、测试、测量及回退快照：`/home/lckfb/orb-slam3/logs/ego-depth-color-20261113/`。
- 新采集与回放：板端 `data/ego-depth-color-*/`；全部原始数据保留。
- 本地当前板端源码镜像：[current](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/current)。
- 本地紧凑证据：[evidence](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/evidence)。
- [实施进度与审阅处理](/home/jacobb/deep_camera/docs/superpowers/plans/2026-11-13-ego-depth-color-progress.md)。

## 最终交付记录

- 字形透明区域回归在移除混合修复时实际失败，恢复修复后最终 ARM 14/14 CTest 全部通过（6.71 秒，无图形跳过）。
- 最终版本再做 20 秒实机试验：260 对合格同步图像全部保存，259/259 处理帧位姿有效，130 次融合，建图和 SLAM 队列淘汰均为 0，退出码 0。
- 本轮有效显示约 3,049 个占据点（中位数），着色 P50 0.080 ms / P95 0.116 ms；地图年龄 P50 121.78 ms / P95 160.73 ms / 最大 236.86 ms，超过 500 ms 的融合更新为 0。
- 峰值 RSS 691.25 MiB，SDK 彩色/深度帧号缺口均为 0；完整录制、时间戳和来源校验通过，CSV 时间戳误差为 0。
- 最终完整地图重载：Unknown 1,973,591 / Free 71,009 / Occupied 3,400，共 2,048,000 体素；膨胀格 52,955。
- 原 ORB 和独立桥接核心哈希保持不变；试验后无本次 `ego-rgbd-map` 进程占用相机。
- [最终真实窗口截图](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/evidence/live-desktop-final.png)：Map/Camera 为 LIVE、Tracking 为 OK，可见深度颜色、米制色条、相机与 Current Frame。
- [最终测试输出](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/evidence/tests-final.log)、[性能分析](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/evidence/runtime-analysis.json)、[最终录制校验](/home/jacobb/deep_camera/logs/ego-depth-color-20261113/evidence/verify-live-final.json)。
