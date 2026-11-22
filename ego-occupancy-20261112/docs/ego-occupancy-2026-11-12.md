# 无 ROS EGO 占据地图实验入口

当前是建图实验，不含规划器、控制器或自动洞口识别。建图和未来规划在同一个 C++ 进程中；每次规划持有一份只读快照。单写者概率层与三槽查询快照均有大小限制。

## 文件和板端位置

- 独立库：`tools/ego-occupancy/`。核心只依赖 Eigen/标准库；关闭 RGB-D 选项后不需要 OpenCV、RealSense、Pangolin 或 ORB。
- 参数：`config/ego-occupancy.yaml`。
- 板端实验目录：`/home/lckfb/orb-slam3/tools/ego-occupancy-20261112/`，内部保持仓库相对布局，独立构建 `build/`，独立 ORB 库 `core/lib/libORB_SLAM3.so`。
- 原 `/home/lckfb/orb-slam3/ORB_SLAM3` 源码、核心库和原有实时/离线启动器不被覆盖。

## 启动与检查

在 RK3576 桌面终端：

```bash
sh /home/lckfb/orb-slam3/tools/ego-occupancy-20261112/scripts/run_ego_occupancy_rk.sh --check
sh /home/lckfb/orb-slam3/tools/ego-occupancy-20261112/scripts/run_ego_occupancy_rk.sh --max-seconds 60 --stride 4
```

启动后等待词典加载和预热，看到 `MAPPING_READY` 再缓慢移动相机。窗口是新的 `EGO Occupancy - raw / inflated`；青色为原始占据，黄色视锥为相机位置及朝向，相机旁的红/绿/蓝轴为光学 X右/Y下/Z前。左侧 Camera/XYZ m 显示当前相机状态和地图坐标；固定原点三色轴继续保留。右下角 CURRENT FRAME 显示实时 RGB、采集帧索引和跟踪状态。勾选 Show inflated 查看膨胀，Show free slice 查看中间 Y 切片空闲区域。Map 的 LIVE 才表示该快照当前可用于规划；失效或过期显示灰色及 STALE/INVALID，坐标重置等待新地图时清除旧点。这个 SLAM 坐标系不保证重力对齐。

目前显示的是体素中心点，初始视角比较远，可用鼠标拖动旋转、滚轮放大。空心区域不会被画成实体；中间 Y 切片也不一定经过实际开口。应同时检查原始占据边框、对应位置的体素查询和相机实物，不能只凭截图的黑色区域判断已经观测为空闲。部分遮挡后的未知区域默认不可通行。

按终端 Ctrl-C 停止，等待录制排空及 `MAPPING_FINISHED`。关闭占据窗口也会停止当前实验。退出码 0 表示有地图且应用没有报告运行错误，3 表示没有有效地图，2 表示输入拒绝，1 表示运行失败；不等同于地图精度或穿洞验收。

```bash
# 无窗口短测；max-frames 在 live 模式限制合格采集对数。
sh /home/lckfb/orb-slam3/tools/ego-occupancy-20261112/scripts/run_ego_occupancy_rk.sh \
  --no-viewer --max-frames 60

# 回放自己的对齐 RGB-D 目录；建议使用录制目录内的标定。
sh /home/lckfb/orb-slam3/tools/ego-occupancy-20261112/scripts/run_ego_occupancy_rk.sh \
  --dataset /完整/RGBD目录 --camera /完整/RGBD目录/D435i.yaml \
  --max-frames 120 --max-seconds 0
```

运行前自动预检，不打开相机或创建输出。实际启动会创建独立数据和日志目录并打印路径。默认相机与原 RGB-D 基线一致：D435i 042222071908、640×480@15、BGR8/Z16、emitter=0、无 IMU、预热 30 对。实际内参、零畸变、深度比例、对齐后标定、格式/分辨率/帧率和时间配对均检查。现阶段 ORB 入口只接受字符串 `File.version: "1.0"` 的 Rectified 配置；非零实机彩色畸变会拒绝，不能仅把 YAML 类型改名绕过校正。

## 地图和时序

示例参数 5 cm、8×4×8 m，共 2,048,000 格，X/Y/Z 按初始相机光学轴右/下/前解释；中心是本坐标 epoch 的第一份合格相机位置，随后不滚动。Z 前后各约 4m；最大射线 4.5m，仍会受网格边界裁剪。膨胀为每轴 0.10m 的量化盒模型，不能解释为已经验证的飞行净空。

原始概率用 double，单帧 hit/miss 汇总，未观测/证据不足为 Unknown。零/NaN/过近及不可靠的远深度跳过，不凭缺测清空空间；可靠有效背景可以沿射线累积 free 证据并撤销旧障碍。阈值和膨胀均可配置。

独立 ORB 桥接在跟踪地图锁下复制同帧位姿、状态和地图生命周期标识；新图/重置、回环、合并和 GBA 应用产生失效通知。普通局部 BA 不会每次清空地图，其几何重影仍须实验验证。一次跟踪丢失后的恢复也会重建，不能把多个 epoch 当作全局连续地图。

SLAM 帧队列 2、建图帧队列 2，均整包丢最旧；建图默认最多 10Hz，对 15Hz 输入按时间间隔采样时实际可能约 7.5Hz。图像和位姿始终绑定。录制队列 64，不静默挤掉已接收录制帧；溢出或磁盘不足报错停止。显示最多约 10Hz；实际还受渲染和地图遍历开销影响。规划默认阻塞 Unknown 和 Outside，持有的旧快照在失效/超时后不能作为有效规划输入。

采样步长默认仍为 2，建议下一轮 RK 手持实验显式传 `--stride 4`：每个轴每 4 像素取一点，在本次实测中明显降低融合负载。稀疏采样可能漏过细边框，不能将它视为已经确定的飞行配置。两次实验均保持 0.5 秒新鲜度阈值。

## 输出与体素读取

数据目录包含：

- `camera.yaml`、`map-config.yaml`：此次输入与融合配置。
- `poses.csv`：捕获索引、ORB 帧号、时间、跟踪状态、是否原子绑定并仍有效、地图 token/修正版本、跟踪耗时及在线位置。
- `mapping.csv`：实际建图帧的接收/发布结果、更新格数、融合/膨胀/发布耗时和应用接收至建图完成延迟。
- `summary.yml`：计数、错误、录制结果、源事件和进程内存峰值。
- `map.egovox`：带版本、坐标、源/epoch 元数据和校验和的完整三态/膨胀查询网格。
- `occupied.ply`、`inflated.ply`：仅供检查的体素中心；不能代替完整地图。
- `occupancy.png`：启用显示时的窗口采样。
- live 额外保存 `rgbd/` 中全部合格同步图像、关联、两路时间戳域/帧号及设备信息。PNG 无损级别 0，约 1.4GB/分钟，实际以录制大小为准。

`map.egovox` 保存查询状态，不保存概率历史，不支持从文件无损恢复融合。加载后的图像仅供离线查询，不被当作新鲜实时地图。

```bash
/home/lckfb/orb-slam3/tools/ego-occupancy-20261112/build/ego-map-inspect /完整/map.egovox
/home/lckfb/orb-slam3/tools/ego-occupancy-20261112/build/ego-map-inspect /完整/map.egovox 0.1 0.2 1.0
```

查询状态数字：0 Unknown、1 Free、2 Occupied、3 Outside。规划同进程调用示例：

```cpp
auto snapshot = worker.reader().acquire();
if (snapshot) {
    auto voxel = snapshot->query(Eigen::Vector3d(x, y, z));
    bool blocked = snapshot->isBlocked(Eigen::Vector3d(x, y, z));
    ego_occupancy::EgoGridMapAdapter ego_map(snapshot);
    int collision = ego_map.getInflateOccupancy(Eigen::Vector3d(x, y, z));
    // 本次计算只使用 snapshot；提交结果前再次检查 isUsable(snapshot)。
}
```

还支持索引查询、坐标转换及区域遍历。不开放可写数组。未来规划在结果提交时检查 epoch、新鲜度，并在最新有效地图复核碰撞；这些查询接口已提供，但尚未接入规划算法。

## 构建与验证命令

```bash
# 工作站核心与图像适配器
cmake -S tools/ego-occupancy -B build/ego-occupancy -DCMAKE_BUILD_TYPE=Release
cmake --build build/ego-occupancy -j2
ctest --test-dir build/ego-occupancy --output-on-failure

# 无图像/相机/SLAM 依赖的构建
cmake -S tools/ego-occupancy -B build/ego-occupancy-core-only \
  -DCMAKE_BUILD_TYPE=Release -DEGO_BUILD_RGBD=OFF
cmake --build build/ego-occupancy-core-only -j2
ctest --test-dir build/ego-occupancy-core-only --output-on-failure

# RK 工具目录内：先构建小库，再准备独立 ORB 核心，再启用应用
cmake -S tools/ego-occupancy -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j1
python3 scripts/build_orb_occupancy_bridge.py \
  --orb-root /home/lckfb/orb-slam3/ORB_SLAM3 --output core \
  --bridge-include tools/ego-occupancy/bridge --bridge-library build/libego_pose_bridge.so
cmake -S tools/ego-occupancy -B build -DCMAKE_BUILD_TYPE=Release \
  -DEGO_BUILD_ORB=ON -DEGO_BUILD_DISPLAY=ON \
  -DORB_SLAM3_ROOT=/home/lckfb/orb-slam3/ORB_SLAM3 \
  -DEGO_ORB_CORE="$PWD/core/lib/libORB_SLAM3.so"
cmake --build build -j1
ctest --test-dir build --output-on-failure
```

桥接构建必须从实际板端源码和已验证对象出发；`core/build-manifest.json` 记录依赖哈希、原库哈希和编译命令，`core/bridge.patch` 记录最小补丁。已有非空 core 目录拒绝覆盖，重建应选新的输出目录或有意识地管理自己的构建产物。不得覆盖原 ORB 目录，也不能把旧新公开类布局混用。原核心退出不完整的问题仍存在，实验进程先停止/join 自有线程、关闭自有输出后采用进程级退出。

本次 Tracking.cc 编译接近板端物理内存上限，临时启用已有 `/swapfile` 完成单任务编译，之后关闭；没有新建 swap、修改 fstab 或在采集期间依赖 swap。

## 验证记录

合成已知开口：512,000 体素，中心 Free/未膨胀/可通行，边框 Occupied；完整快照重新加载查询一致。

2026-11-12 最终板端 8/8 CTest 通过，工作站显示构建 7/7、不含图像依赖的核心构建 4/4 通过，原有 Python 回归 36/36 通过。覆盖射线/阈值/边界、障碍撤销、快照所有权/三槽耗尽/过期失效、文件损坏拒绝、深度投影、桥接生命周期、队列和显示启动失败。审阅发现的显示线程初始化竞态、小地图文件读取、预检参数验证及相机配置版本问题均修复并验证；另补了旧 weak_ptr 不能重新获取已复用存储的回归。

板端既有序列首轮回放：60/60 位姿验证、30 次融合、3,473 个占据体素，峰值 RSS 588.64 MiB。随后完成两次用户手持实验，场景是部分空心区域：

| 指标 | 60 秒，stride 2 | 30 秒，stride 4 |
| --- | ---: | ---: |
| 录制的同步 RGB-D 对 | 860/860 | 412/412 |
| 已验证位姿/处理帧 | 842/842 | 359/409 |
| 融合次数 | 416 | 181 |
| 融合耗时中位数 | 106.71 ms | 22.39 ms |
| 应用接收至建图完成，中位数 / P95 | 341.01 / 513.57 ms | 131.45 / 211.02 ms |
| 完成时已超过 500 ms 的更新 | 25/416 | 0/181 |
| 建图队列淘汰 | 10 | 0 |
| 峰值 RSS | 735.54 MiB | 704.15 MiB |

两次进程均正常退出，SDK 帧号缺口为 0。第一次全部处理帧跟踪有效，25 次更新完成时已过新鲜度阈值，支持“延迟导致灰色”的判断，但不能据此逐帧还原用户所见画面。第二次出现 45 帧 RECENTLY_LOST、3 帧 LOST、2 帧初始化，以及三次活动地图重置；这些帧没有被融合。地图 epoch 会失效并重建。两次移动轨迹不同，不能把跟踪丢失差异归因于采样步长。用户反馈边框不够明显、出现灰色，因此手持几何质量尚未验收。

板端数据位于 `/home/lckfb/orb-slam3/data/` 下：

- `ego-occupancy-replay-20261112-01`：首轮已有序列回放。
- `ego-occupancy-live-20261112-01`：stride 2，含全部录制帧。
- `ego-occupancy-live-20261112-02-stride4`：stride 4，含全部录制帧。
- `ego-occupancy-final-replay-20261112-01`：最终版本回放第一次录制的前 20 帧，20/20 位姿验证、10 次融合。

工作站 `logs/ego-occupancy-20261112/` 保存紧凑证据和完整查询网格，不复制全部原始录制。ARM 生成的两份手持地图已在 x86_64 重新加载，状态数量与 ARM 输出一致。原 ORB 核心库 SHA256 仍为 `cd0dbad1b2e906c147a7ce499ed1287236d66306f71d929adb218fd8fe68326b`；测试结束无占用相机的本次进程，临时 swap 已关闭。

最终校验脚本检查录制的所有 PNG 头、关联条目、时间戳/域、处理索引及每次融合对应的 ORB 帧号/地图来源：

```bash
python3 tools/ego-occupancy/tests/verify_run.py /完整/数据目录
# 回放数据需指定原始录制目录
python3 tools/ego-occupancy/tests/verify_run.py /完整/回放输出 --source-rgbd /完整/录制/rgbd
```

两次初始手持实验的 `poses.csv` 只输出 12 位有效数字，在 epoch 秒时间戳上造成最多约 5 ms 的文本舍入；内存中的帧绑定和二进制地图时间戳不受影响。最终版本改为 double 的 17 位精度，20 帧回放的严格校验通过，文本时间戳误差为 0。保留原始 CSV；校验这两份历史输出必须显式加 `--allow-legacy-rounded-timestamps`，结果 JSON 也记录此限制，不能将其称为严格时间戳无误差。

后续应在有纹理、光照稳定的场景中，用已知尺寸的部分开口重复慢速移动，测量边框偏差、重影、遮挡及重置前后行为。还需确定无人机含桨叶外廓、最小净空、误差预算和飞行速度。当前交付是可复现的占据地图实验基线与体素查询接口，尚未验证真实开口尺寸精度，未接入规划或完成穿洞飞行。


## 2026-11-12 相机位姿与 Current Frame 补充

ORB 位姿从独立桥接记录进入 MappingWorker，DepthProjector 使用完整 T_world_camera 转换深度端点；原显示器缺少的是相机和图像输入，并非融合没有使用位姿。现在跟踪循环每处理一帧，将同一个彩色图像、捕获索引、桥接记录和应用接收时间写入单槽显示邮箱。邮箱拥有 BGR 副本，覆盖最新帧，没有等待显示的队列。

相机图标和 XYZ 使用原始矩阵，不用固定原点或从点云猜测位置。显示前检查同帧验证结果、当前桥接修正版本、地图来源/epoch 及 0.5 秒位姿时效。跟踪失效时隐藏图标、清除 XYZ，但 CURRENT FRAME 继续显示 RGB；若输入停住，左侧 RGB frame 标记 STALE。Map 与 Camera 的状态分开，地图更新过慢不会被误写为跟踪丢失。

RGB 小窗使用已处理的当前彩色帧，带帧索引/跟踪状态文字；本版未叠加 ORB 特征点，也没有再次打开相机或启动 ORB 原 Viewer。当前地图坐标并非地理坐标或经过机体外参标定的无人机位置。


图形回归应在可访问的 RK 桌面显式启用；仅有 DISPLAY 环境变量不代表能连接 X11。工作站无可用桌面时，两项真实渲染测试返回77并记为跳过，其他8项正常运行。板端本次11/11均实际通过，包含颜色保持和截图失败后的窗口清理：

```bash
cd /home/lckfb/orb-slam3/tools/ego-occupancy-20261112
EGO_TEST_DESKTOP=1 DISPLAY=:0 XAUTHORITY=/home/lckfb/.Xauthority \
  ctest --test-dir build --output-on-failure
```

显示修正：体素遍历完成后才从邮箱同时取得最新帧与采样时间，再读取地图状态，避免旧时钟误拒新帧或延长旧位姿的有效期。RGB 渲染前恢复白色调制，避免被前面的蓝色坐标轴染色；截图失败也在显示线程销毁纹理和窗口。这两项图形缺陷均在实际 ARM 桌面先复现失败，再验证通过。


最终20秒实时测试：260/260同步对录制，255/255处理帧位姿验证，128次地图融合，建图队列淘汰0，退出码0，RSS峰值710.06MiB。应用接收至地图完成延迟中位155.77ms/P95 258.36ms/最大278.91ms，本轮无更新超过500ms。数据：`/home/lckfb/orb-slam3/data/ego-current-frame-live-20261112-01`；严格录制/帧与来源校验通过，时间戳文本误差0。之前180帧可视回放也全部验证、90次融合。

完整桌面截图的RGB帧98与poses.csv帧98对应：XYZ原值约(-0.1330,-0.0076,0.1055)m，界面显示(-0.13,-0.01,0.11)m，Map和Camera均LIVE、Tracking为OK。黄色视锥与固定原点轴分离，说明当前相机确实使用位姿链路。该截图窗口左缘略超出桌面，可拖动窗口调整位置；不影响体素或XYZ计算。

工作站证据：`logs/ego-current-frame-20261112/live/desktop-window.png`、`display-analysis.json`、`recording-verification.json`和完整`map.egovox`；构建、先失败后通过的图形测试及部署源哈希位于同目录的`build/`。板端与工作站31份工具源/头/CMake文件哈希一致。原ORB库哈希保持不变，测试进程已结束，未遗留相机占用。
