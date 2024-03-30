#rm -rf build/ log/ install/
colcon build --symlink-install --packages-ignore rds_hud rds_sims rds_control rds_gui
source install/setup.bash
