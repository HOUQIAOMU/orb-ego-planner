> 2026-11-16目录迁移说明：本文中的旧源码／构建路径为历史记录，当前入口与路径见[工程README](../README.md)，逐文件迁移见父项目 `logs/layout-20261116/file-moves.json`。

# RK3576 同进程在线静态规划实验

日期：2026-11-16。工程：`/home/lckfb/orb-slam3/ego-occupancy-20261112`。

## 1. 本轮范围与预期效果

沿用已验证的ORB-SLAM3、相机深度着色占据地图和P1–P3规划核心，接入P4静态在线预览。地图、当前相机和RGB画面持续更新；用户停稳后按P，固定本轮起终点并生成B样条。新地图先复核当前整条曲线，必要时再从冻结起点重算。

首轮任务为相机光轴前方0.5m至0.9m的短距离几何探针，速度和加速度边界均为零。起点不是相机原点，用于避开D435i近距离盲区；不将相机原点附近Unknown强行改为Free。具体Free范围不足时，规划必须拒绝。该探针不等于无人机机体当前状态，也不构成人工开口或飞行验收。

## 2. 板端入口

```bash
cd /home/lckfb/orb-slam3
bash scripts/build_ego_online_static.sh
bash scripts/run_ego_online_static.sh --max-seconds 120
```

新构建目录为`ego-occupancy-20261112/build-online`；原建图入口默认仍使用`build/ego-rgbd-map`。新入口继续复用既有相机互斥锁、记录流程、配置和结果目录规则。运行需要已登录且可访问的板端桌面。

先朝向纹理充分、前方至少约1.5m内没有近距离障碍的场景，缓慢补图后停稳。窗口中地图和相机状态有效后，点击`Plan static P`或在窗口按小写`p`触发；`Clear plan C`或小写`c`清除。相机可继续补图，但本任务起点不随手移动；改变起点需重新触发。跟踪失效或来源/epoch重置后，恢复定位也须重新触发。

可通过`--probe-start-m`、`--probe-goal-m`更改短段距离；`--planning-config`指定实验配置。范围以米计，不自动搜索或移动起点来掩盖阻挡。

## 3. 显示与数据语义

- 相机视锥：实时相机；绿色点：冻结起点；红色点：冻结终点。
- 青色曲线：读取时已通过当前最新地图复核的样条。
- 灰色曲线：待复核或失效的历史候选。参考初值为细灰线，存在绕障搜索时A*折线为橙色。
- `READY/PENDING/INVALID`是轨迹有效性；地图继续更新时允许出现短暂Pending，不能保留永久Ready标记。

每次运行的终端打印`DATA_DIR`。规划文件位于：

```text
DATA_DIR/
├── map.egovox                  # 采集结束时的地图，不冒充每条轨迹规划时的快照
├── poses.csv
├── occupancy.png
├── planning-config.yaml
└── planning/
    ├── events.jsonl
    └── request-N-candidate-M/
        ├── artifact-complete.json # 全部导出流close成功后生成
        ├── request.yaml
        ├── planning-report.json
        ├── online-version.json
        ├── bspline.json
        ├── trajectory.csv
        ├── reference.csv
        ├── validation.json
        └── ...                 # 初始样条、碰撞段和A*等现有诊断
```

`trajectory.csv`的列为`t,x,y,z,vx,vy,vz,ax,ay,az`；t是曲线内部相对时间，p/v/a使用同一SLAM局部地图坐标及米、秒。不能将t直接解释为实际执行墙钟，也不能把光学坐标轴的Z直接解释为重力高度。

`bspline.json`保存控制点、节点与均匀间隔；`online-version.json`保存冻结状态编号、候选编号、`planned_on`、`validated_on`和记录当时的有效性。`artifact-complete.json`表示本组文件已完整导出。文件是历史结果；文件存在不能证明当前仍Ready。后续失效及复核必须结合`events.jsonl`，实际使用仍应读取运行时门控。

## 4. 内部架构与地图共享

```text
D435i → ORB桥接 → MappingWorker → MapReader三槽快照
                 │                    │
                 └→纯数值相机状态     └→PlanningRuntime单worker
                     人工P触发             │
                     冻结探针任务          ├→PlannerCore：A* / 优化 / 完整验证
                                          ├→释放旧快照
                                          └→获取最新图，完整曲线复核
                                                │
                         不可变数值图层 ← 版本/状态/年龄门控
                               │                └→历史CSV/B样条/事件日志
                         既有显示线程
```

仅Mapper写占据概率；规划不维护第二张概率地图。A*、优化及同轮初检共享同一冻结快照；算法结果不保留快照租约。提交复核前释放旧图，再取最新图；显示只接收自有点数组，GL仍由原显示线程管理。普通地图更新不反复取消求解，但读取端立即撤销旧版本Ready资格；新任务或来源/状态失效取消旧任务，并拒收旧结果。

主要新增文件：`planning/include/ego_planner/planning_runtime.h`、`planning/src/planning_runtime.cpp`、`apps/online_planning_session.h/.cpp`、`apps/planning_state_adapter.h`、`include/ego_occupancy/planning_layer.h`。现有`rgbd_map.cpp`负责装配，`display.cpp`负责按钮及图层绘制。

## 5. 开发问题、解决方法与验收证据

### 5.1 开发中遇到的问题

| 问题/现象 | 定位与解决方法 | 验证 |
|---|---|---|
| 在线会话测试报`planning output must be new` | 新测试预建输出目录，与生产拒覆盖规则冲突；删除测试中的预建目录，保留拒覆盖行为 | `online_session_contract`通过；另注入同名候选目录，检查错误可见且任务撤销 |
| 图层未实现时，没有轨迹也没有规划截图 | 先写真实帧缓冲测试，复现缺绘制；显示只接自有数值点数组，增加Ready青线及Pending/Invalid灰线 | 原失败见display-red.log；真实桌面青线、灰线测试均通过 |
| 同图复核失败每20ms重试，或Unverified不断重跑优化 | 审查后板端复现350ms内17次复核、或9次求解；增加复核冷却，Unverified保留待证候选，确定碰撞才重算；所有重算继续受500ms限流 | 故障注入先5/5失败；修复后失败次数有界、新图恢复与取消测试通过 |
| IO异常只显示`runtime stopped`，未传到主程序 | 区分正常停止和故障停止；新增持久`error()`，保留目录/原错误，主循环与清理后均检查；地图仍有序保存，最终failure与退出码反映规划失败 | 注入`disk full sentinel`及候选目录冲突；错误原文可见、Invalid、拒绝新任务、退出清理通过 |
| 算法检查预算后发生延迟，仍可能提交Ready | 候选分配后、最终提交锁内重新检查WorkBudget | 1ms预算＋20ms延迟的故障注入先失败后通过；这是提交边界测试，不宣称真实validator自身漏检 |
| CTest的JUnit相对路径落入EGO内部logs | `--test-dir`改变结果相对位置；核对后迁入父项目logs，仅清理本次空目录；后续指定绝对输出路径 | 两份XML独立解析，Release 36项、UBSan 25项均无failure/error/skip |
| 测试中给Eigen TranslationPart直接赋initializer_list编译失败 | TranslationPart是block表达式，改用显式`Vec3`赋值 | 语法检查及板端完整编译通过 |

重要审查问题均先在RK复现再修复；不重复派发审查。故障注入测试明确替换同步算法返回/延迟，只验证调度边界；真实算法、真实MapReader和实际X渲染另有独立集成测试，不能互相替代。

### 5.2 自动回归

- 板端无显示Release：25/25通过，包括真实地图运行时、在线会话导出、五项调度故障注入及既有A*/优化/验证测试。
- 完整`build-online`：36/36通过，无跳过，包含实际X服务器下RGB、深度、青线/灰线、窗口清理及原建图回归。
- 关闭规划的配置6/6通过，独立占据库仍可用；原ORB核心及独立桥接核心SHA256保持不变。
- 最终清单发现板端A*有本轮开始后的括号/空白调整，保留该板端内容归档；重新核对当前树构建及回归，未用旧副本覆盖。
- C++新增/修改文件格式检查通过；统一入口shell语法检查通过。
- 优化UBSan：25/25通过，无跳过；RelWithDebInfo保留undefined、float-cast-overflow、不可恢复错误及标准库断言，生产预算没有放宽。ASan沿用此前板端环境限制，不计作通过。

[UBSan原始日志](../../logs/online-static-20261116/final-ubsan.log)、[Release原始日志](../../logs/online-static-20261116/final-release.log)、[审查失败复现](../../logs/online-static-20261116/review-red.log)、[修复后无显示全量](../../logs/online-static-20261116/review-green.log)、[独立审查记录](../../logs/online-static-20261116/review-record.md)。

### 5.3 60秒静止实机实验

结果目录：`/home/lckfb/orb-slam3/data/online-static-20261116-live-01`。运行正常退出，summary的failure为空；用户现场确认“地图、RGB和曲线都能清楚看到”。

| 项目 | 记录 |
|---|---|
| RGB-D/定位 | 采集并记录861对，SLAM处理860帧，860帧位姿均验证有效；同步配对拒绝2次，SDK帧号间隙及SLAM队列丢帧为0 |
| 建图 | 融合/发布430次；建图拒绝、队列丢弃、快照发布跳过均为0 |
| 本轮任务 | 冻结状态编号228，光轴前0.5→0.9m短段；零端点v/a |
| 求解/更新 | 仅求解1次；地图从第114版到430版，完整曲线复核317次，没有为每张新图重跑A*和优化 |
| 版本检查 | 所有318条Ready事件的validated_on与observed_map完全一致，source/epoch与planned_on一致；冻结状态编号保持不变 |
| 输出 | 401行有限值轨迹数据，曲线内部时间0→2秒；首尾p/v/a与请求误差小于1e-6；B样条、报告、版本、完成标记均存在 |
| 规划耗时 | 该短段单次PlannerCore 3.666ms；不是所有场景的性能保证 |
| 最新图复核 | 317次样本P50 0.245ms、P95 0.301ms、最大0.570ms；包含视图准备和验证/提交复制，不含历史文件导出 |
| 内存 | 整个相机＋SLAM＋建图＋显示＋规划进程峰值约677.9MiB |

关停时日志最终转Invalid是地图/状态不再新鲜后的正常撤销；保存轨迹仍是历史记录，不在进程退出后保留“在线有效”资格。本轮没有人为加入新障碍或制造跟踪丢失；这些分支来自自动回归，不能写成现场已验证。

[实机截图](../../logs/online-static-20261116/live-ready.png)、[实机统计与文件核验](../../logs/online-static-20261116/live-acceptance-summary.json)、[实机原始日志](../../logs/online-static-20261116/live-01.log)。RGB/深度原始采集保留在板端，未全量复制到本机。

## 6. 后续边界

本轮不实现运动状态估计、动态未来锚点、按墙钟执行轨迹、自动洞口识别或控制器。手持移动补图时，曲线仍从冻结位置出发；不应期待曲线像实际飞行轨迹一样自动跟随手的位置。连续运动重规划属于P5/P6。
