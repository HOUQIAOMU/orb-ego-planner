# CMake generated Testfile for 
# Source directory: /home/lckfb/orb-slam3/ego-occupancy-20261112
# Build directory: /home/lckfb/orb-slam3/ego-occupancy-20261112/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(display_color_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-display-color-test")
set_tests_properties(display_color_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;35;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(voxel_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-map-test")
set_tests_properties(voxel_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;49;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(snapshot_io_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-io-test")
set_tests_properties(snapshot_io_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;53;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(pose_bridge_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-bridge-test")
set_tests_properties(pose_bridge_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;57;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(display_startup_failure_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-display-startup-test")
set_tests_properties(display_startup_failure_contract PROPERTIES  TIMEOUT "15" _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;79;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(display_depth_render_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-display-depth-render-test")
set_tests_properties(display_depth_render_contract PROPERTIES  RUN_SERIAL "TRUE" SKIP_RETURN_CODE "77" TIMEOUT "15" _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;86;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(display_rgb_color_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-display-render-test" "color")
set_tests_properties(display_rgb_color_contract PROPERTIES  RUN_SERIAL "TRUE" SKIP_RETURN_CODE "77" TIMEOUT "15" _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;93;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(display_failure_cleanup_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-display-render-test" "cleanup")
set_tests_properties(display_failure_cleanup_contract PROPERTIES  RUN_SERIAL "TRUE" SKIP_RETURN_CODE "77" TIMEOUT "15" _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;95;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(app_preflight_contract "/usr/bin/python3" "/home/lckfb/orb-slam3/ego-occupancy-20261112/tests/app_contract.py" "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/ego-rgbd-map")
set_tests_properties(app_preflight_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;149;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(depth_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-depth-test")
set_tests_properties(depth_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;171;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(worker_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-worker-test")
set_tests_properties(worker_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;175;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(current_frame_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-current-frame-test")
set_tests_properties(current_frame_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;180;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(mapping_cli_contract "/home/lckfb/orb-slam3/ego-occupancy-20261112/build/occupancy-options-test")
set_tests_properties(mapping_cli_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;190;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
add_test(bridge_relocation_contract "/usr/bin/python3" "/home/lckfb/orb-slam3/ego-occupancy-20261112/tests/test_relink.py")
set_tests_properties(bridge_relocation_contract PROPERTIES  _BACKTRACE_TRIPLES "/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;194;add_test;/home/lckfb/orb-slam3/ego-occupancy-20261112/CMakeLists.txt;0;")
