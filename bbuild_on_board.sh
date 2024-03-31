#rm -rf build/ log/ install/
colcon build --symlink-install --packages-select rds_msgs
source install/setup.bash
colcon build --symlink-install --packages-ignore rds_sims
source install/setup.bash
