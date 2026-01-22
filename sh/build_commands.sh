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
vcs import src < autoware-nightly.repos

source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# build (no limit on CPU usage)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error 2>&1 | tee build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error --cmake-clean-cache 2>&1 | tee build.log

# build (limiting CPU usage)
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error 2>&1 | tee build.log

# build again (limiting CPU usage)
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error 2>&1 | tee -a build.log
# build again (no limit on CPU usage)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error 2>&1 | tee -a build.log

# Rebuild
vcs import src < autoware.repos
vcs import src < tools.repos
vcs import src < simulator.repos
vcs import src < autoware-nightly.repos
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
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
rosdep update

# re-build limiting CPU usage
MAKEFLAGS="-j16" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --cmake-clean-cache 2>&1 | tee build.log

# build one package
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-up-to autoware_ndt_scan_matcher 2>&1 | tee -a build.log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --continue-on-error --packages-select autoware_ndt_scan_matcher 2>&1 | tee -a build.log

# run test
colcon test --packages-select autoware_ndt_scan_matcher 2>&1 | tee test.log
colcon test-result --verbose 2>&1 | tee -a test.log
