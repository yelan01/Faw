source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash

////////////////開aw的///////////////////////

ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/nishishinjuku_autoware_map \
  data_path:=$HOME/autoware_data \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  vehicle_simulation:=true \
  rviz:=true \
  scenario_simulation:=false \
  is_simulation:=false


////////////////////////////////////////////////


/////////////////有新程式的時候 重新編譯用////////////////////
cd ~/autoware
colcon build --symlink-install --packages-select mpc_io_logger
source ~/autoware/install/setup.bash
////////////////////////////////////////////////




//////////MPC的
ros2 run mpc_io_logger trigger_logger_mpc --algo mpc --truncate --dump-path /tmp/mpc_log.jsonl

////pp的
ros2 run mpc_io_logger trigger_logger_pp --truncate --dump_path /tmp/pp_log.jsonl

