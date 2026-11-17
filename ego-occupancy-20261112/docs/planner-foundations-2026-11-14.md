# RK3576 独立 A* 与数学库开发及验收记录

日期：2026-11-14。板端地址：lckfb@172.20.10.5。

## 交付范围

在 /home/lckfb/orb-slam3/ego-occupancy-20261112 原地新增 planning/，直接在板端编译与验证。
原地开发是用户明确要求，未创建worktree、未提交用户仓库；修改前备份CMake与原核心哈希。
EGO_BUILD_PLANNER 默认OFF，新模块使用build-planning，原build保持运行基线。

- PlanningMapView：持有唯一不可变MapSnapshot，按ROI生成raw三态配置空间；
  Occupied/Unknown/Outside均阻挡，不重复叠加原inflated。
- Astar：26邻域、保守边连接检查、不平移端点、不可变优先队列记录、连续数组工作区、
  有界节点/队列/扩展数、单调时钟预算及取消，失败无残留路径。
- Bspline：3×N控制点、De Boor、导数、时间缩放、三维速度/加速度范数上界、
  原EGO最小二乘参数化。
- PolynomialTraj：归一化时间的单段五次初值；多段minimum-snap留待manager接入。
- apps/astar_probe.cpp：读取.egovox、保存路径/曲线/来源版本/耗时，模式明确为OfflineValidation。
- C++17、RAII与const接口，核心无ROS/SDK/图像/GUI依赖。新增目标启用
  -Wall -Wextra -Wpedantic -Werror；关键几何、时间与队列逻辑加入中文说明。

尚未接rebound优化、完整样条碰撞/穿门验收、在线FSM、运动起点、控制器或真实飞行。

## 最终测试

| 检查 | 结果 |
| --- | --- |
| Release无设备构建的CTest | 9/9通过（含4项新增规划契约与5项既有基础契约） |
| 原程序桌面回归 | 14/14通过，无跳过项，包含RGB/深度颜色和窗口清理 |
| UBSan与标准容器断言 | 新增功能4/4通过 |
| ASan | 无法运行；最小空main同样在初始化时失败，不能报告为通过 |
| 独立审查 | 1个Important已回归修复；1个Minor记录暂缓 |
| 原代码/应用/核心哈希 | 原map.cpp、io.cpp、原ego-rgbd-map、独立ORB核心与原ORB核心均不变 |
| 动态依赖 | 探针仅标准C++/C运行库；无ORB/OpenCV/RealSense/Pangolin/ROS |

ASan报错为sanitizer_allocator_primary64的地址空间预留检查失败；板端
CONFIG_ARM64_VA_BITS=39。没有修改内核或关闭安全检查来制造绿色结果。
UBSan不等同于地址/泄漏检查，当前不能声称ASan内存验证完成。

## 核心测试与问题修复

- 用独立Dijkstra对照A*代价；验证开放/封闭/Unknown开口、穿角、非中心端点、重复搜索、
  相同点、非法输入、超时/取消与资源上限。原地图文件经过真实加载器读取。
- 数学测试采用解析直线、五次边界、有限差分导数、时间缩放与三维范数，
  不以绘制出曲线作为数学正确性的依据。
- 夹具初始存储顺序误用了X连续；原map.cpp实际Z连续。不对称Unknown用例暴露问题，
  修正测试夹具后通过，原地图代码未改动。
- 编译器-Werror发现Eigen lambda返回惰性表达式引用临时Index，改为显式返回值类型。
- 当前160×80×160=2,048,000格，超过初版200万上限。增加实际布局回归RED后，
  将默认上限调整为有界400万；允许更大工作区，但没有无限分配。
- 独立审查复现：非中心端点的机体恰好接触Unknown面时漏判。
  新增Unknown/Occupied的面、棱、角，Outside接触及零外廓端点回归，先RED再修复。
  非零外廓按闭合候选体素生成掩码；整数半径额外覆盖接触格。零外廓的实际端点
  另外检查相邻格闭合接触。代价是更保守，临界尺寸开口可能拒绝。
- 暂缓Minor：特殊重复内部节点向量可求位置，但导数构造可能拒绝；
  本轮均匀节点链路不受影响，限制记录于planning/SOURCE.md。

## 板端实测呈现

合成地图40×30×20，分辨率5cm，实验盒半尺寸0.10m。以下为单次测量，非实时保证，
IO和绘图不计入地图准备加搜索预算。

| 场景 | 结果 | 准备ms | A* ms | 总ms | 路径长度m |
| --- | --- | ---: | ---: | ---: | ---: |
| 开口墙 | Success | 3.857 | 12.641 | 16.719 | 1.6985 |
| 封闭墙 | NoPath | 2.128 | 14.733 | 16.986 | — |
| 未知开口 | NoPath | 3.826 | 22.169 | 26.148 | — |
| 已有手持地图整图 | Success | 392.612 | 17.429 | 412.685 | 0.4000 |
| 同一手持地图局部ROI | Success | 1.254 | 0.173 | 1.428 | 0.4000 |

真实图为160×80×160格；ROI为12,789格。端点显式记录于real-map-input.json，
选自已知Free体积，既不是相机当前位姿验收，也不是物理洞口穿越验收。
整图准备约393ms，超过200ms起测预算；后续应按EGO任务/碰撞段选局部ROI，
不能每轮重复派生整图或据小场景宣称完成在线实时规划。

## 如何查看与复现

~~~bash
ssh lckfb@172.20.10.5
cd /home/lckfb/orb-slam3
./scripts/run_planner_foundations.sh
~~~

入口会创建新的data/results运行目录，不覆盖已有结果。它展示配置、-j1编译、CTest、
探针命令并保存图表。最终本轮结果目录：

~~~text
/home/lckfb/orb-slam3/data/results/planner-foundations-20261114-final/
  report.png / report.svg       三场景路径图与数学p/v/a曲线
  geometry-3d.png               占据墙与A*折线的三维图
  ctest.log / ctest.xml         机器可读及终端测试证据
  checks.json                  验收摘要及限制
  source-manifest.json          最终源文件SHA256
  open/ closed/ unknown/       各场景report.json、path.csv和math.csv
  real-map/                    真实保存图的短路径和报告
  real-map-input.json          真实地图来源哈希与明确端点
  real-map-roi.json            同一地图/端点的ROI验证
  dependencies.txt            ELF依赖
~~~

代码说明见planning/README.md与planning/SOURCE.md，构建/RED/GREEN/审查证据归入
/home/lckfb/orb-slam3/logs/planner-foundations-20261114/。
ROI验证源码roi-smoke.cpp也在该日志目录，便于复现，不散落项目根目录。

图中的A*折线不是飞行样条；数学曲线是独立示例，参数不是已验收飞行速度/加速度。
最小二乘拟合不保证任意输入精确满足全部端点条件，后续Manager/Validator必须重新检查。
