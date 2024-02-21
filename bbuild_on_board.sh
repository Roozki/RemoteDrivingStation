rm -rf build/ log/ install/
colcon build --packages-ignore rds_hud rds_sims rds_control rds_gui
source install/setup.bash
