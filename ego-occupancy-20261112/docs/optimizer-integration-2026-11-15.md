# RK3576 EGO 优化器 P1–P3 开发与验收

> 后续（2026-11-16）：已完成在线静态自由空间探针、板侧轨迹显示和数据保存实测，见[在线实验记录](online-static-experiment-2026-11-16.md)。本文仍保留11月15日P1–P3的历史验收。

日期：2026-11-15。板端工程：`/home/lckfb/orb-slam3/ego-occupancy-20261112`。

本轮按用户授权交付 **P1–P3完整离线规划**：从已有占据地图快照生成经完整验证的B样条，附纯事件状态机。P4在线worker/实时轨迹图层、P5/P6手持连续运动重规划尚未接入。

## 1. 已实现链路和地图共享

```text
PlanningRequest（起终点p/v/a、人工参考点/门） + MapSnapshot S_n
  → SeedBuilder：多段minimum-jerk参考、最小二乘初值、精确首尾控制点
  → PlanningMapView：同一S_n、固定局部ROI、实验外廓配置空间
  → CollisionConstraintBuilder：完整曲线碰撞发现、局部A*、base/direction代理
  → ReboundOptimizer：固定约束L-BFGS、按需rebound/refine
  → 时间调整、重新施加精确运动边界
  → TrajectoryValidator：整条曲线、端点、3D速度/加速度、穿门约束
  → PlanResult：曲线、地图版本和诊断；不保留地图租约
```

同一轮A*、优化和初检共享一份只读视图；不维护第二张概率地图。每次尝试固定ROI，最多一次有界扩大，共用任务预算。Unknown、Occupied、Outside和闭合接触阻挡；不重复叠加原地图inflated层。

`PlannerCore::plan()`同步、可取消，无跨任务可变状态。`PlanningFsm`目前是纯事件库：等待地图/状态/任务、规划、验证、Ready与失败，返回动作，不执行耗时计算。

实际在线接入仍需要P4：MapReader新鲜快照、唯一规划worker、有界邮箱、释放旧快照后最新地图复核、结果版本/时效门控，以及显示图层。离线Success只对所记录的冻结地图有效。

## 2. 文件组织

| 文件/模块 | 职责 |
|---|---|
| planning/include/ego_planner/planning_request.h | p/v/a、人工门、参考点和请求版本 |
| planning_config.h / src/planning_config.cpp | 实验参数与资源/输入校验 |
| seed_builder.h/.cpp | 多段最小jerk参考、精确端点、均匀样条初值 |
| collision_constraints.h/.cpp | 碰撞分段、已有A*、代理约束及失败原因 |
| rebound_optimizer.h/.cpp | 四项代价/梯度、L-BFGS、refine与数值诊断 |
| trajectory_validator.h/.cpp | 完整Bezier曲线证书、边界、动力学和人工门 |
| planner_core.h/.cpp | 单轮流程、ROI、有界重试、重定时与候选选择 |
| planning_fsm.h/.cpp | 来源/epoch/请求绑定的纯事件状态机 |
| planning/third_party/ | 上游lbfgs.hpp、许可证依据与最小补丁说明 |
| apps/plan_snapshot.cpp / planning_io.h/.cpp | .egovox入口、限定YAML解析和报告IO |
| config/planning.yaml | 手持实验外廓、v/a、代价和资源限额 |
| planning_tasks/example_gate.yaml | 合成场景人工门示例 |
| planning/tests/ | 正式单元/集成/CLI回归 |
| scripts/render_trajectory_report.py | 只绘制已保存的C++结果，不计算规划 |
| scripts/validate_ego_optimizer.py | 七案例板端复现 |

保留原 `ego_planner_math`、`ego_planner_core` 和A*探针。新增代码采用C++17、Eigen和CMake，开启 `-Wall -Wextra -Wpedantic -Werror`，使用Google风格clang-format并在数学/失效边界添加中文注释。

## 3. 开发中遇到的问题、原因与解决方法

本节依据本轮板端失败日志、源码和独立审查记录补充。失败复现与修复后的验收分别列出；日志文件名中的`red/green`仅表示当时的执行阶段，是否通过以日志正文为准。以下源码路径均相对于本工程根目录。

复查入口：[独立审查记录](../../logs/optimizer-integration-20261115/final-review.md)、[最终Release日志](../../logs/optimizer-integration-20261115/final-tests.log)、[最终UBSan日志](../../logs/optimizer-integration-20261115/ubsan-final-tests.log)。这些是开发当时留存的验收记录，本次补充文档没有重新运行代码测试。

### 3.1 最小二乘拟合不能保证起终点的位置、速度和加速度

- **现象与风险：**上游`Parameterize`提供最小二乘初值，其端点p/v/a是拟合条件，不能直接作为严格满足运动边界的保证。随后若优化器还能移动这些控制点，误差会继续变化。
- **原因：**拟合残差与精确边界是不同约束；三次B样条的端点状态由相邻3个控制点共同决定，不能只固定首尾一个点。
- **解决：**在`planning/src/seed_builder.cpp`增加`enforceBoundary()`。以起点为例，令节点间隔为Δt：`q0=p−Δt·v+Δt²·a/3`、`q1=p−Δt²·a/6`、`q2=p+Δt·v+Δt²·a/3`；终点对称处理。优化时固定首尾各3列，只优化内部控制点，最终验收重新求值检查p/v/a。
- **相关澄清：**检查上游`minSnapTraj`的Q矩阵后确认其实际积分三阶导数平方，本轮多段参考按minimum-jerk命名；使用矩阵分解求解，避免显式求逆，并检查分段C2连续性。
- **验证：**`planning_seed`检查精确边界、多段参考和异常输入；`planning_retime`检查重定时后的非零运动边界。见最终Release日志对应测试。

### 3.2 浮点舍入使轨迹与体素公共面的接触被漏判

- **现象：**公共面接触用例求值为`y=1.2000000000000002`，早期验收返回`Valid`，但闭集碰撞约定要求接触也阻挡。
- **原因：**Bezier求值、坐标换算和栅格索引存在累计舍入；只调用一次`nextafter`扩大1 ULP不足以覆盖全部误差。后续又发现：细分时先遇到`Unverified`便退出，会掩盖兄弟区间中的确定碰撞。
- **解决：**在`planning/src/trajectory_validator.cpp`的包围盒查询中增加与数值尺度相关的舍入包络，并覆盖公共面两侧体素。无法证明完全Free时返回`Unverified`，拒绝提交；递归仍检查其他区间，发现真实碰撞时优先报告`Collision`。
- **验证：**[初始诊断](../../logs/optimizer-integration-20261115/p3-diagnostic.log)记录漏判；[中间分类失败](../../logs/optimizer-integration-20261115/contact-classification-red.log)保留后续问题。最终`planning_validator`同时覆盖接触拒绝和未知体素碰撞。`Unverified`表示未获得安全证明，不等同于已证明碰撞，也不能作为可用轨迹。

### 3.3 薄墙位于两个Free控制点之间，原代理约束不能推离障碍

- **现象：**起终点及相邻控制点都在Free中，曲线却穿过薄墙；优化后仍被完整曲线验收器拒绝。仅增加迭代次数不能解决。
- **原因：**控制点是否占据不能代表整段曲线是否碰撞；初版代理平面停在旧控制点附近，没有定位真实障碍边界，优化梯度缺少有效推离方向。
- **解决：**在`planning/src/collision_constraints.cpp`沿曲线寻找碰撞见证，利用见证处切平面与局部A*路径相交，沿绕障方向寻找Free边界，再将base/direction代理传播给该段的支撑控制点，推广上游EGO的短段处理。一次L-BFGS调用内代理保持不变，更新代理放在外层有界rebound重试中。
- **验证：**[薄墙约束失败复现](../../logs/optimizer-integration-20261115/thin-proxy-red.log)与[修复后两项测试](../../logs/optimizer-integration-20261115/thin-proxy-green.log)记录定位过程；最终`rebound`案例成功通过开口。曲线采样只用于发现约束，最终仍由完整Bezier验收决定；封闭墙或Unknown开口必须返回`NoPath`。

### 3.4 极小控制点间距导致Inf转int的未定义行为

- **现象：**普通ARM Release未报告错误，UBSan加入`float-cast-overflow`后报告`inf is outside the range of representable values of type 'int'`。
- **原因：**路径长度除以极小间距可溢出为Inf；在执行最大控制点数量限制之前就转为int，已经发生未定义行为。
- **解决：**`planning/src/seed_builder.cpp`先在浮点域按控制点数量上限截断，再执行有界整数转换，保留输入及资源上限检查。
- **验证：**[UBSan失败原文](../../logs/optimizer-integration-20261115/ubsan-spacing-red.log)定位到当时第143行；极小间距纳入`planning_seed`，最终UBSan全量通过。普通Release没有崩溃不能代替这类检查。

### 3.5 近似均匀节点使验收器证明了另一条曲线

- **现象：**独立审查对显式节点施加`8e-10`扰动，仍通过原`1e-9`均匀性容差；真实曲线已进入Occupied，旧验收却返回`Valid`。
- **原因：**验收器按理想均匀节点的固定系数转换Bezier，而曲线求值使用真实显式节点，二者不是同一个数学对象。放宽或收紧普通比较容差不能建立两者相等的保证。
- **解决：**`planning/src/bspline.cpp`仅由均匀构造器记录可信dt，`scaledTime()`保留并更新该元数据；`seed_builder.cpp::uniformInterval()`拒绝从显式节点近似推断均匀性。`trajectory_validator.cpp`从实际节点区间的端点和导数构造Bezier。JSON增加`uniform_interval`，供结果重建使用。
- **验证：**[审查失败复现](../../logs/optimizer-integration-20261115/review-red.log)中的`planning_validator`保留该反例；[修复后全量测试](../../logs/optimizer-integration-20261115/review-fix-tests.log)通过。当前规划接口要求可信均匀三次样条，不扩展为任意非均匀节点规划。

### 3.6 epoch切换、跟踪失效后仍可能沿用旧状态或旧目标

- **现象：**来源A的状态有效后切换到新epoch B，旧实现仍保留`state_valid`；新任务可能在没有B有效状态时触发求解。迟到的旧来源状态/任务也缺少过滤。
- **原因：**地图来源、状态和任务没有共同绑定source/epoch，重置只清除了部分标记。
- **解决：**`planning/src/planning_fsm.cpp`在来源/epoch切换及状态失效时撤销状态和任务；`StateValid`、`TaskSet`核对当前source/epoch。任务`request_id`在FSM生命周期内严格递增；重新取得合格状态后仍需新任务，旧事件不能重新启动求解。
- **验证：**[输入与FSM失败日志](../../logs/optimizer-integration-20261115/input-fsm-red.log)、审查失败日志分别记录跟踪失效和epoch反例；最终`planning_fsm`覆盖失效、迟到事件、取消及重新确认。这里只验证纯事件逻辑，P4还需验证真实线程中的消息顺序、状态年龄和结果门控。

### 3.7 平滑优化反而丢弃已经满足偏置人工门的初值

- **现象：**偏离起终点直线的人工门已有合法seed，优化器为降低平滑代价将曲线拉直，最终`GateNotSatisfied`，整轮被拒绝。
- **原因：**优化代价下降不等于满足任务约束；初版没有保留已独立验收通过的可行初值。
- **解决：**`planning/src/planner_core.cpp`先完整验证seed。若优化候选失败且仍有预算，则对seed再次完整验证；通过后以`selected_candidate=SeedFallback`返回。超时或取消一律不能走回退，报告不将回退初值标成优化结果。
- **验证：**审查失败日志中的`planning_planner`复现该问题；最终`offset-gate`案例明确返回`SeedFallback`并通过唯一正向穿门检查。该机制保留可行性，不代表求得更优轨迹。

### 3.8 直接缩放时间破坏非零起终点速度、加速度

- **问题与风险：**时间扩大r倍会将速度变为原来的`1/r`、加速度变为`1/r²`。静止端点不易暴露问题，非零运动边界会被改变。
- **解决：**`planning/src/planner_core.cpp`调整节点间隔后，按原请求重新构造首尾控制点，再执行refine和完整验收；保持重定时次数及整轮预算上限。
- **验证：**正式`planning_retime`直接调用`PlannerCore`，要求`time_adjustments>0`并检查真实端点p/v/a。交付的`retime`案例实际加时一次，边界最大误差约`1.11e-16`；不是仅对`scaledTime()`做孤立测试。

### 3.9 NaN输入和机体几何不一致可能绕过验收意图

- **现象：**加入非有限请求状态及地图视图/验收配置使用不同机体尺寸的反例后，旧验收未按预期返回`InvalidInput`。
- **原因：**不能依靠残差大小比较排除NaN；点机器人地图视图也不能作为较大机体的碰撞证明。验收函数作为公共入口，不能假定调用者已做全部检查。
- **解决：**`trajectory_validator.cpp`在计算残差前调用请求/配置校验，显式检查p/v/a等输入的有限性；`PlanningMapView`保存并提供`halfExtent()`，验收要求其与配置完全一致，否则拒绝。
- **验证：**[非有限输入失败](../../logs/optimizer-integration-20261115/input-fsm-red.log)和[几何不一致失败](../../logs/optimizer-integration-20261115/geometry-red.log)均保留；最终`planning_validator`覆盖两类输入。实验外廓仍是示例参数，真实无人机尺寸未确定。

### 3.10 上游L-BFGS取消回调覆盖不完整

- **现象：**用原始上游头文件执行“调用前已取消”的用例，目标函数仍被评价，`ctx.evaluations == 0`断言失败。
- **原因：**原More-Thuente线搜索已有逐次progress回调，但没有初次目标评价前和每轮外层迭代的完整取消检查。仅接上线搜索回调不能保证入口取消立即生效。
- **解决：**`planning/third_party/lbfgs.hpp`增加初次评价前、外层迭代的取消检查，并补上工作区分配失败的检查和部分分配清理；`rebound_optimizer.cpp`在目标函数、progress及调用前后共享同一`WorkBudget`，即使数值库停止分支报告收敛，也拒绝已超时/取消的候选。约束仅在外层重试间更新。
- **验证：**[原始上游反例](../../logs/optimizer-integration-20261115/lbfgs-upstream-red.log)与最终`planning_lbfgs_budget`对应。补丁来源与范围写入`planning/third_party/SOURCE.md`，没有修改或覆盖原ORB核心库。这里提供协作式取消，不承诺操作系统级硬实时抢占。

### 3.11 数值停止状态、轨迹可用性和诊断报告混在一起

- **现象：**早期JSON缺少`iterations`等字段，CLI契约测试抛出`KeyError`；A*已失败、没有执行曲线验收时，默认验收状态也容易误导排查。
- **原因：**“数值求解停止”“选中了哪条候选”“完整轨迹验证结论”是三个独立结果，单一成功标记无法解释失败阶段。
- **解决：**在`planner_core.h/.cpp`、`apps/planning_io.cpp`分别记录`optimizer_status`、`selected_candidate`和`validation`，增加迭代/评价次数及四项代价，`cost_scope`说明代价属于最后一次优化调用。未运行验收时使用`NotRun`。达到迭代上限的候选也必须完整通过后才能输出；超时/取消不输出可用曲线。
- **验证：**[报告字段失败](../../logs/optimizer-integration-20261115/report-fields-red.log)、[NotRun契约失败](../../logs/optimizer-integration-20261115/not-run-red.log)及最终`planning_cli_contract`记录闭环。失败CSV仅保留表头，防止下游误用残留轨迹。

### 3.12 严格编译检查发现误导性缩进

- **现象：**`apps/planning_io.cpp`早期将条件输出和后续`JsonVector`调用写在同一行，触发`-Werror=misleading-indentation`，构建中止。
- **解决：**拆分语句、明确条件作用范围，按Google风格clang-format整理新增代码；保持`-Wall -Wextra -Wpedantic -Werror`，没有通过关闭告警绕过。
- **验证：**[当时构建日志](../../logs/optimizer-integration-20261115/cli-green-build.log)正文保留编译失败；最终构建及CLI测试通过。该日志虽名为`green-build`，内容不能当作通过证据；GCC另外输出的ABI说明性note也不等同于编译错误。

### 3.13 真实桌面验收受到Xauthority失效影响

- **现象：**板端停在登录界面，旧用户Xauthority失效，使用旧认证不能连接真实X服务器，显示测试无法据此完成。
- **原因：**当前显示服务器认证与旧会话文件不匹配，是桌面环境问题，不能用跳过显示测试代替验证。
- **解决：**使用仅板端可读的私有临时认证文件连接当前真实X服务器，执行原显示回归后清理。未更改登录状态或X访问控制，也未将认证内容复制到工作站。
- **验证：**[真实桌面回归日志](../../logs/optimizer-integration-20261115/desktop-full.log)记录原14项全部通过、无跳过，包括深度绘制、RGB颜色和失败清理；不是仅运行无显示测试。

### 3.14 未优化UBSan构建触发既有耗时预算

- **现象：**首轮未优化Debug UBSan为14/16：既有`planning_map_view`约11.84秒超过10秒，`planning_probe_contract`触发约2秒预算。不能将该轮记作全量通过。
- **原因与处理：**未优化构建叠加检测开销，无法满足原验收预算。最终改用`RelWithDebInfo`的`-O2 -g`，保留`undefined`、`float-cast-overflow`、不可恢复错误及`_GLIBCXX_ASSERTIONS`，不放松生产规划预算。
- **验证：**[Debug首轮失败日志](../../logs/optimizer-integration-20261115/ubsan-full-tests.log)保留原文；最终UBSan17/17通过。该结论只适用于记录的优化检测配置，不声称未优化Debug也满足同样耗时要求。

### 3.15 三维报告自动缩放放大了浮点级Z噪声

- **现象与原因：**近似平面的轨迹存在浮点级Z差异，绘图库自动缩放会将极小变化放大为明显起伏，使图像与米制空间的实际比例不一致。
- **解决：**`scripts/render_trajectory_report.py`按真实ROI设定三轴范围和显示比例，并显示占据环境；报告只读取C++导出的结果，不另行计算路径。
- **验证：**七案例报告按统一米制范围绘制，原始`trajectory.csv`保留用于核对。图像用于辅助检查，轨迹可用性仍依据C++完整验收结果。

### 3.16 仍然保留的限制及下一阶段处理

| 项目 | 当前结论 | 后续处理 |
|---|---|---|
| ASan | 上一阶段记录的板端39位虚拟地址环境限制仍存在，本轮未计作通过 | 单独解决运行环境兼容性后再执行ASan；UBSan结果不能替代ASan |
| 在线更新与规划并发 | 当前仅验证冻结快照和纯FSM，没有P4实际worker、邮箱和最新图复核 | 接入唯一规划worker、预算/取消、版本与时效门控；旧快照结果须在最新地图复核 |
| 手持连续重规划 | 本轮离线结果不能证明手持运动时可连续输出合格轨迹 | 先接静止起点在线显示，再验证运动状态质量和未来锚点；不能假定人手按旧轨迹运动 |
| 无人机飞行安全 | 实验外廓、离线时延及人工门不构成飞行验收 | 确定真实尺寸、运动约束、坐标/状态连续性，再单独验收控制与飞行 |

## 4. 验收结果

- 最终Release：17/17通过，无跳过，见final-tests.log与release.xml。
- 规划关闭构建：5/5通过，公共构建/帮助脚本实际调用通过，ldd无缺失库且不链接ROS/相机/显示。
- 原建图/实际X渲染：14/14通过，无跳过，见desktop-full.log。
- 最终UBSan：17/17通过，无跳过；undefined + float-cast-overflow + 标准库断言，见ubsan-final-tests.log与ubsan.xml。
- 独立审查的三个重要问题均在RK先RED，再修复并运行全量GREEN；没有遗留次要审查问题。
- 原ORB核心、独立ORB桥接核心和真实输入地图哈希均保持不变。
- ASan沿用上一阶段已记录的板端39位虚拟地址环境限制，本轮不将其计为通过。

七个冻结地图案例：

| 案例 | 预期/实际行为 |
|---|---|
| gate | 人工门任务，整条优化样条通过且唯一正向穿门 |
| rebound | 初始参考穿墙，局部A*生成代理后优化样条通过开口 |
| closed | NoPath，不输出可用样条 |
| unknown | NoPath，未知开口不当作Free |
| offset-gate | 偏置人工门，合法初值重新验证后明确SeedFallback |
| retime | 非零运动边界，加时一次后完整通过 |
| real | 已保存真实图的0.4m已知Free短段通过 |

真实图端点选自先前已验证Free体积，不能当作当前相机位姿或真实洞口穿越验收。实验外廓是参数示例，没有替代未定的真实无人机尺寸。

每个案例在无编译任务时重复20次（共140次）。下表为C++整轮规划耗时，不含地图/报告IO、进程启动和绘图；RSS为整个子进程的峰值，包含IO。P95采用nearest-rank。

| 案例 | P50 ms | P95 ms | 最大 ms | 峰值RSS MiB |
|---|---:|---:|---:|---:|
| gate | 5.356 | 5.980 | 5.983 | 13.33 |
| rebound | 4.756 | 10.172 | 10.212 | 13.33 |
| closed | 46.744 | 52.721 | 75.846 | 13.33 |
| unknown | 47.133 | 49.890 | 71.399 | 13.33 |
| offset-gate | 1.858 | 2.260 | 4.047 | 13.33 |
| retime | 1.439 | 2.651 | 2.737 | 13.33 |
| real | 2.196 | 2.259 | 2.263 | 13.33 |

这些是固定小场景及已知Free短段的离线样本，不能替代P4活地图、相机状态年龄、复核排队与现场负载验收。各阶段原始分位数见结果目录benchmark.json。

[查看完整七案例图与报告](../../data/results/optimizer-20261115-acceptance-final/README.md)。

## 5. 回来后如何验收

在RK3576执行：

```bash
cd /home/lckfb/orb-slam3
bash scripts/build_ego_planner.sh
bash scripts/run_ego_planner_validation.sh
```

第一条配置、-j1构建、运行全部CTest；第二条创建新的data/results/optimizer-<时间戳>，运行七案例并绘图。新的结果目录不覆盖任何已有内容。

本次已保存结果：`/home/lckfb/orb-slam3/data/results/optimizer-20261115-acceptance-final/`。打开README或各案例trajectory-report.png；灰线为初值参考、橙线为局部A*、蓝线为已验收B样条。3D坐标范围使用实际ROI，避免放大浮点级Z噪声。

单份地图入口：

```bash
bash scripts/run_ego_plan_snapshot.sh \
  data/results/planner-foundations-20261114-final/fixtures/open.egovox \
  ego-occupancy-20261112/planning_tasks/example_gate.yaml \
  ego-occupancy-20261112/config/planning.yaml \
  data/results/my-new-plan 5000
```

退出码0=冻结图上验收通过，2=输入/IO错误，3=规划拒绝。输出目录必须不存在，父目录须存在。YAML支持明确限定的平面键值/向量子集，未知/重复键拒绝。

主要输出：request.yaml、planning.yaml、reference.csv、initial-bspline.json、astar-paths、constraints.json、validation.json、planning-report.json；成功额外有bspline.json及非空trajectory.csv(t,p,v,a)。失败轨迹CSV只有表头。

## 6. 保留与实施取舍

- 直接在板端当前目录开发，使用codex/optimizer-20261115分支、完整源码回退包和SHA256清单；不覆盖原未提交改动，不把混合历史擅自提交。代价是以清单/归档复现，而不是一个包含用户旧改动的提交。
- 用户授权P1–P3或P1–P4，本轮选择完整P1–P3节点；P4的在线快照复核、工作线程和实时轨迹显示明确保留到下一阶段。当前验收方式是离线图/数据，不是在线执行窗口。
- UBSan使用带优化及调试符号的检测构建；未优化Debug的超时原文保留，不能宣称Debug也满足相同时间预算。

回退包：`/home/lckfb/orb-slam3/logs/optimizer-integration-20261115/source-before.tar.gz`。命令、失败/通过日志、检测和源文件清单归入同一logs目录；本轮没有删除原数据或原代码。

后续P4应基于现有同步PlannerCore与纯FSM接入：停止时人工触发冻结起点，地图更新先复核、必要时重算；来源/跟踪失效撤销状态与目标。P5/P6再接合格运动状态与未来锚点，不能按旧轨迹假定人手正在执行。控制器、推力/倾角/制动及真实飞行验收继续单独开展。
