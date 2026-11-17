# ORB桥接构建辅助

日常构建使用父项目`scripts/build_ego_occupancy.sh`。

- `verify_bridge_abi.py`：比较当前桥接头与保留对象的原始ABI参考。
- `relink_orb_bridge.py`：校验原始对象和清单，显式指定桥接库后重新链接独立核心，不覆盖原ORB核心。
- `build_orb_occupancy_bridge.py`：从经过校验的ORB基线构建独立桥接核心，输出目录必须为空。
- `build_orb_stage_profile.py`：保留的历史辅助实现；桥接构建导入其中`once`、`verify_baseline`、`sha`，不能因文件名含profile就删除。其独立性能分析入口不属于当前支持功能。
- `bridge_abi_probe.cpp`：ABI比较程序，不属于应用入口。
