# Clear all
cd autoware
rm -rf build/ install/ log/ src/

# Clean environment to avoid conflicts with old workspaces
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset ROS_PLUGIN_PATH

mkdir src

vcs import src < autoware.repos
vcs import src < tools.repos
vcs import src < repositories/autoware.repos
vcs import src < repositories/tools.repos
vcs import src < repositories/simulator.repos
vcs import src < repositories/autoware-nightly.repos

source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade -y
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
rosdep update

# build (no limit on CPU usage)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error --cmake-clean-cache 2>&1 | tee build.log

# build (limiting CPU usage)
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error 2>&1 | tee build.log
# build for autoware
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to \
  tier4_localization_launch autoware_default_adapi_universe autoware_adapi_adaptors \
  sample_sensor_kit_description sample_vehicle_description sample_sensor_kit_launch ndt_direct_measure \
  autoware_imu_corrector autoware_lanelet2_map_visualizer \
  2>&1 | tee build.log
# build (limiting CPU usage) for pilot-auto.x2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-up-to \
  tier4_localization_launch autoware_default_adapi_universe autoware_adapi_adaptors individual_params j6_gen2_description \
  tier4_external_api_msgs aip_x2_gen2_description aip_x2_gen2_launch \
  2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --packages-up-to \
  tier4_localization_launch autoware_default_adapi_universe autoware_adapi_adaptors individual_params j6_gen2_description \
  tier4_external_api_msgs imu_monitor autoware_cuda_pointcloud_preprocessor autoware_localization_rviz_plugin autoware_lanelet2_map_visualizer \
  2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-up-to \
  tier4_localization_launch default_ad_api ad_api_adaptors individual_params j6_gen1_description \
  2>&1 | tee build.log
# build (limiting CPU usage) for pilot-auto.xx1
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to \
  tier4_localization_launch autoware_default_adapi ad_api_adaptors individual_params jpntaxi_description \
  2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to \
  tier4_localization_launch autoware_stop_filter autoware_ekf_localizer autoware_twist2accel autoware_pose_instability_detector autoware_localization_error_monitor \
  tier4_sensing_launch pe_ars408_ros aip_xx1_launch individual_params autoware_launch \
  tier4_localization_rviz_plugin \
  tier4_system_launch tier4_vehicle_launch diagnostic_graph_aggregator \
  jpntaxi_launch jpntaxi_description aip_xx1_description \
  2>&1 | tee build.log
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to \
  tier4_localization_launch aip_xx1_launch individual_params autoware_launch \
  autoware_localization_rviz_plugin \
  jpntaxi_description aip_xx1_description \
  2>&1 | tee build.log

# build again (limiting CPU usage)
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error 2>&1 | tee -a build.log
# build replay localization packages
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to $(cat localization_packages.txt | grep -v '^#' | tr '\n' ' ') 2>&1 | tee build.log
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-up-to $(cat localization_packages.txt | grep -v '^#' | tr '\n' ' ') 2>&1 | tee build.log
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-select $(cat localization_packages.txt | grep -v '^#' | tr '\n' ' ') 2>&1 | tee build.log
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-select $(cat localization_packages.txt | grep -v '^#' | tr '\n' ' ') 2>&1 | tee build.log
# build again (no limit on CPU usage)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error 2>&1 | tee -a build.log

# Rebuild
vcs import src < autoware.repos
vcs import src < repositories/autoware.repos
vcs import src < repositories/tools.repos
vcs import src < repositories/simulator.repos
vcs import src < repositories/autoware-nightly.repos
vcs pull src

# 環境変数をクリーンアップ
unset AMENT_PREFIX_PATH
# 一般的に CMAKE_PREFIX_PATH には CMake のインストールディレクトリや、CMake がパッケージを探すためのパスを指定します。問題にはならないのでunsetしなくてOK
# unset CMAKE_PREFIX_PATH
# colcon でビルド後、setup.bash などを source するとCOLCON_PREFIX_PATHに値がセットされます。特に問題にはならないのでunsetしなくてOK
# unset COLCON_PREFIX_PATH

# 前回のビルドファイルを削除
rm -rf build/ install/ log/
# 依存関係を含めて再ビルド
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# re-build limiting CPU usage
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error 2>&1 | tee build.log
# MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --cmake-clean-cache 2>&1 | tee build.log

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to tier4_localization_launch aip_xx1_launch individual_params tier4_localization_rviz_plugin 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to tier4_localization_launch aip_xx1_launch individual_params autoware_localization_rviz_plugin 2>&1 | tee build.log


# build one package
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to autoware_ndt_scan_matcher 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to autoware_lidar_marker_localizer 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to autoware_unified_localization 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-up-to autoware_ekf_localizer 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-select autoware_ndt_scan_matcher 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --cmake-clean-cache --packages-select autoware_ekf_localizer 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 8 --continue-on-error --packages-select autoware_launch 2>&1 | tee build.log


colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to sample_vehicle_launch 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to aip_x2_gen2_launch 2>&1 | tee -a build.log

# run test
colcon test --packages-select autoware_ndt_scan_matcher 2>&1 | tee test.log
colcon test --packages-select autoware_lidar_marker_localizer 2>&1 | tee test.log
colcon test --packages-select autoware_ekf_localizer 2>&1 | tee test.log
colcon test --packages-select autoware_ekf_localizer --event-handlers console_direct+ 2>&1 | tee test.log
colcon test --packages-select autoware_unified_localization 2>&1 | tee test.log
# display test results
colcon test-result --all --verbose 2>&1 | tee -a test.log
# display test results of a specific package
colcon test-result --all --verbose --test-result-base build/autoware_ekf_localizer 2>&1 | tee -a test.log
colcon test-result --all --verbose --test-result-base build/autoware_ndt_scan_matcher 2>&1 | tee -a test.log
colcon test-result --all --verbose --test-result-base build/autoware_lidar_marker_localizer 2>&1 | tee -a test.log
colcon test-result --all --verbose --test-result-base build/autoware_unified_localization 2>&1 | tee -a test.log

# display test results for errors and failures
colcon test-result --verbose 2>&1 | tee -a test.log
