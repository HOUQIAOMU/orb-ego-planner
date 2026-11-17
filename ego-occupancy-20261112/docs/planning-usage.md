> 2026-11-16目录迁移说明：本文中的旧源码／构建路径为历史记录，当前入口与路径见[工程README](../README.md)，逐文件迁移见父项目 `logs/layout-20261116/file-moves.json`。

# 无 ROS EGO 规划模块

2026-11-16：在P1–P3基础上接入同进程在线静态自由空间探针：单规划worker、最新地图复核、人工触发、实时轨迹图层和历史轨迹数据。人工开口的在线任务输入、动态手持起点、控制器和飞行仍留待后续。见[在线实验与验收](../docs/online-static-experiment-2026-11-16.md)。

## 模块与数据流

```text
PlanningRequest(p/v/a, 人工参考点/门) + MapSnapshot
  → SeedBuilder(多段minimum-jerk + 精确三次B样条边界)
  → CollisionConstraintBuilder(完整曲线碰撞发现 + 局部A*)
  → ReboundOptimizer(固定base_point/direction代理 + L-BFGS)
  → 时间调整/精确边界恢复/refine
  → TrajectoryValidator(整条Bezier保守验证、3D范数、穿门)
  → PlanResult(曲线/诊断/地图版本，不持有地图快照)
```

原A*探针继续保留。算法核心使用C++17/Eigen/现有占据库，不依赖ROS、相机、ORB或OpenGL。
`PlanningFsm`目前用纯事件验证地图版本、任务替换与失效，不会自行启动在线线程；在线调度由新增`PlanningRuntime`负责。

## 构建与验收

在 `/home/lckfb/orb-slam3` 执行：

```bash
bash scripts/build_ego_planner.sh
bash scripts/run_ego_planner_validation.sh
```

第二条命令创建新的 `data/results/optimizer-<时间戳>/`，包含人工门、绕障、封闭、未知及已有真实地图案例；运行C++二进制后由Python生成图，不覆盖旧结果。

单独运行：

```bash
bash scripts/run_ego_plan_snapshot.sh \
  data/results/planner-foundations-20261114-final/fixtures/open.egovox \
  ego-occupancy-20261112/config/tasks/example_gate.yaml \
  ego-occupancy-20261112/config/planning.yaml \
  data/results/my-new-plan 5000
```

退出码：0为冻结地图上整条曲线通过；2为输入/IO错误；3为规划拒绝。输出目录必须不存在，父目录必须存在。
配置文件支持明确限定的平面YAML子集（`key: number`、`key: [x,y,z]`、布尔和注释），未知/重复键拒绝；不是完整YAML解析器。

## 内部只读调用

`PlannerCore::plan(shared_ptr<const MapSnapshot>, request, config, WorkBudget)`同步运行，所有尝试共用预算。
每次尝试固定快照/局部ROI，最多一次有界扩大。A*只处理碰撞段；Unknown、Outside和闭合接触均阻挡。
`PlanningMapView::halfExtent()`明确其配置空间外廓，验收配置必须一致，避免误用不同外廓。
结果的 `planned_on`带source/epoch/sequence，离线成功不代表对新地图持续有效。
在线使用前必须在最新地图复核，并在地图来源、跟踪或任务失效时撤销；这些调度职责属于P4。

## 安全性与适用边界

- 首尾p/v/a通过首尾各3个控制点精确施加，中间控制点参与优化；时间调整后重建运动边界。
- 只支持由均匀构造器生成并保留dt元数据的三次样条，至少8个控制点，数量和迭代均有上限；不按近似相等的显式节点推断均匀性。
- 曲线逐段转换Bezier，用凸包AABB与De Casteljau细分证明完整曲线Free；数值舍入、深度/访问/时间不足以证明时返回Unverified，不提交成功。
- 控制点与离散采样只用于代理构造和图示，不能代替完整曲线验证。末1/3和固定边界段均检查。
- 逐轴动力学软代价保留上游语义，最终速度/加速度使用三维导数凸包范数硬限制。
- 人工门要求唯一正向交点位于按实验外廓收缩的窗口内；切线、共面、反向和不确定交点拒绝。
- L-BFGS数值状态与轨迹验收分别记录。超时/取消不产生可用曲线；空约束和失败A*不混作成功。
- 外廓和v/a仅是手持实验参数；地图轴不自动等同重力坐标，未验证推力/倾角/制动或控制跟踪。

## 输出文件

`request.yaml`、`planning.yaml`、`reference.csv`、`initial-bspline.json`、`collision-segments.json`、
`astar-paths/*.csv`、`constraints.json`、`validation.json`、`planning-report.json`。
报告另记迭代数、四项原始代价及 `selected_candidate`（优化曲线或重新验证的合法初值回退），代价统计明确指向最后一次优化调用。
成功额外有 `bspline.json`、非空 `trajectory.csv(t,x,y,z,vx,vy,vz,ax,ay,az)`；拒绝时轨迹CSV仅表头。
图示脚本生成 `trajectory-report.png/.svg`，报告中的地图版本与参数用于复现。

来源与移植差异见 `SOURCE.md`，完整开发交付记录见 `../docs/optimizer-integration-2026-11-15.md`。
