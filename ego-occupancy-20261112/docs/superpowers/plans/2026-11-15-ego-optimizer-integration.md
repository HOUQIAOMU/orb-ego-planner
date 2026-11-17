# EGO Optimizer Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在RK3576交付P1–P3完整离线规划；通过后才考虑P4静态在线接入。

**Architecture:** 复用当前PlanningMapView/Astar/Bspline。单轮PlannerCore以同一不可变快照运行参考→碰撞段A*→代理约束→L-BFGS→独立验证；FSM与算法分离。

**Tech Stack:** C++17 / Eigen / CMake / 当前上游lbfgs.hpp / Python离线报告。

**Spec:** ../specs/2026-11-15-ego-optimizer-online-replanning-design.md

## Global Constraints

- 以板端当前源码为准，保留未提交修改和原ORB核心；板端-j1。
- 同进程、无ROS；控制器、动态手持P5/P6不在本次范围。
- Unknown/Outside/接触阻挡；完整曲线通过才有Ready资格。
- 固定均匀三次样条首尾各3控制点；边界p/v/a必须复核。
- A*、优化、重启、验证共用WorkBudget；每次尝试固定快照/ROI。
- 新增接口C++17，明确失败、有限值/资源检查，关键数学与安全边界附中文注释。

## Review Focus

1. 曲线末段、闭合接触和极薄障碍；P3测试完整曲线与保守细分。
2. 移动边界经时间调整不能改变所要求p/v/a；P1/P3非零边界测试。
3. 地图变更与旧结果迟到；P3纯FSM版本测试，P4若实施则真实worker测试。
4. 空约束与失败A*不得成为优化成功；P2封闭/退化/预算测试。
5. 穿门代价不等于穿门证明；P3反向/切线/窗口外/多次穿越测试。

## Task 1 / P1：参考与精确边界

Files: planning/include/ego_planner/{planning_config,planning_request,seed_builder}.h；planning/src/{planning_config,seed_builder}.cpp；planning/tests/seed_test.cpp；planning/CMakeLists.txt。

Interfaces: BoundaryState(p,v,a)；PlanningRequest(start,goal,reference_points,optional gate,request_id)；PlanningConfig；SeedBuilder::build(request,config,duration_scale)返回参考和均匀三次初值。

- [x] 先写静态/非零边界、多段C2、非法配置与零长度拒绝测试，板端编译确认缺失接口失败。
- [x] 实现有界多段最小jerk参考及精确首尾控制点；所有初值通过数值端点检查。
- [x] 板端cmake --build build-planning -j1；ctest -R planning_seed，期望全部通过。

## Task 2 / P2：碰撞约束与优化

Files: collision_constraints.h/.cpp；rebound_optimizer.h/.cpp；third_party/lbfgs.hpp与来源；tests/optimizer_test.cpp。

Interfaces: CollisionConstraintBuilder::build(view,spline,config,budget)；ReboundOptimizer::optimize(seed,constraints,config,budget,refine)。结构化状态、诊断与有界重启由PlannerCore组织。

- [x] 先写有限差分梯度、固定边界、绕障约束、封闭失败、取消/超时测试并观察失败。
- [x] 实现上游数学代价与固定约束LBFGS；共享预算覆盖评价和线搜索。
- [x] 板端ctest -R 'planning_(seed|optimizer)'，期望通过；保留原A*与数学回归。

## Task 3 / P3：完整验证、规划与FSM

Files: trajectory_validator.h/.cpp；planner_core.h/.cpp；planning_fsm.h/.cpp；tests/{validator,planner,fsm}_test.cpp。

Interfaces: Validator::validate(view,spline,request,config,budget)；PlannerCore::plan(snapshot,request,config,budget)；PlanningFsm事件输入/动作输出，不持地图或执行求解。

- [x] 写完整Bezier细分、末段薄壁、未知/接触、范数超限、门方向/窗口/切线、版本FSM测试并运行失败。
- [x] 实现完整保守验收、碰撞反馈、有界ROI扩大/重启、时间调整和精确边界恢复。
- [x] 板端新增测试及既有回归；失败不输出可用轨迹，数值状态和验收状态独立。

## Task 4 / P3：离线入口和验收材料

Files: apps/plan_snapshot.cpp；apps/planning_io.h/.cpp；config/planning.yaml；planning_tasks/example_gate.yaml；scripts/render_trajectory_report.py；父项目scripts/run_ego_plan_snapshot.sh；tests/planner_cli_contract.py。

- [x] 先写入口预检/输出拒覆盖/成功与失败报告契约，运行失败。
- [x] 实现有限配置解析、公开.egovox输入、轨迹/约束/报告文件与可视化；输出目录不覆盖。
- [x] 生成合成开口/绕障/封闭/未知案例与真实快照结果；保存完整命令和机器可读报告。

## Task 5 / P4：可选在线静态节点

本轮按用户授权选择P1–P3节点，P4未实施。仅当P1–P3完整通过才实施。新增runtime worker、版本门控及显示图层；人工触发冻结状态，MapReader新图优先复核，旧完成事件拒收。必须先测并发取消/三槽释放/重置，再接入现有rgbd_map。无法完整验收则不宣称P4完成，保留P1–P3可独立使用。

2026-11-16补充：已按用户选择完成在线静态自由空间节点，同进程显示和保存轨迹；见[当日实施与验收](../../online-static-experiment-2026-11-16.md)。上述“未实施”保留为11月15日历史记录。人工开口在线输入和连续运动仍未交付。

## Task 6：交付验证与独立审查

- [x] Release、UBSan、既有实际桌面测试；确认ORB核心哈希与原建图基线。
- [x] 一次独立全量代码审查，重要问题RED→GREEN修复并重跑相关回归。
- [x] 保存源码清单/命令/测试日志/图，更新docs交付说明与准确阶段状态。
