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




//////////MPC的檔案錄製
ros2 run mpc_io_logger trigger_logger_mpc --algo mpc --truncate --dump-path /tmp/mpc_log.jsonl

////pp的檔案錄製
ros2 run mpc_io_logger trigger_logger_pp --truncate --dump_path /tmp/pp_log.jsonl


//////mpc的權重 這兩個檔案要同時改
/home/ye/autoware/src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/lateral
/home/ye/autoware/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral

 //////然後那兩個該死的檔案改完之後計的重新編譯              
cd ~/autoware
source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_launch --symlink-install
source install/setup.bash


               
