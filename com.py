source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash

////////////////開aw的///////////////////////

source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/Shinjuku-Map/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit


////////////////////////////////////////////////

/////////////////看目前「起點 + 終點」///////////////
ros2 topic echo --once /planning/mission_planning/route
/////////////////////////////////////////////////
               
////////////////////////////確定那一種橫向控制器
ros2 param get /control/trajectory_follower/controller_node_exe lateral_controller_mode
////////////////////////////////////////////////////


               
/////////////////有新程式的時候 重新編譯用////////////////////
cd ~/autoware
colcon build --symlink-install --packages-select mpc_io_logger
source ~/autoware/install/setup.bash
////////////////////////////////////////////////




//////////MPC的檔案錄製
ros2 run mpc_io_logger trigger_logger_mpc --algo mpc --truncate --dump-path /tmp/mpc_log.jsonl     or  
               mpc_log --dump-path ~/autoware/src/Algorithm/logs/mpc_$(date +%Y%m%d_%H%M%S).jsonl --truncate
              ros2 run mpc_io_logger trigger_logger_mpc --algo mpc --truncate


////pp的檔案錄製
ros2 run mpc_io_logger trigger_logger_pp --truncate --dump_path /tmp/pp_log.jsonl   or   
               pp_log --dump_path ~/autoware/src/Algorithm/logs/pp_$(date +%Y%m%d_%H%M%S).jsonl --truncate
              ros2 run mpc_io_logger trigger_logger_pp --truncate --dump-path /tmp/pp_log.jsonl





//////mpc的權重 這兩個檔案要同時改
/home/ye/autoware/src/universe/autoware_universe/control/autoware_trajectory_follower_node/param/lateral
/home/ye/autoware/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral

27行    mpc_weight_steer_rate: 0.05                   //反正不能高於0.1 最低是0.0   0.1以上它就不打方向盤了 0.08 還是有一點點沒有很想打 所以改成0.05看看
28行    mpc_weight_steer_acc: 0.0001                  //不知道意義在哪 調了沒啥用但應該是這個直
37行    mpc_low_curvature_thresh_curvature: 0.02      //低曲率模式 也不知道在幹麻
35行    mpc_low_curvature_weight_steer_rate: 0.1      // 也不知道在幹麻

              
               
 //////然後那兩個該死的檔案改完之後計的重新編譯              
cd ~/autoware
source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_launch --symlink-install
source install/setup.bash










//////////////////  使用 gedit ~/.bashrc & 


              然後加入這些該死的玩意
# Created by `pipx` on 2026-01-18 20:51:24
export PATH="$PATH:/home/ye/.local/bin"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CCACHE_DIR="$HOME/.ccache"
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export PATH="$HOME/autoware/src/Algorithm/bin:$PATH"


# ===== Autoware environment helper =====
aw_env() {
  # ROS2 Humble
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  else
    echo "[aw_env] not found: /opt/ros/humble/setup.bash"
    return 1
  fi

  # Autoware (你的工作區)
  if [ -f "$HOME/autoware/install/setup.bash" ]; then
    source "$HOME/autoware/install/setup.bash"
  else
    echo "[aw_env] not found: $HOME/autoware/install/setup.bash"
    echo "[aw_env] 你如果 Autoware 不在 ~/autoware，請改成正確路徑"
    return 1
  fi

  return 0
}

# ===== Autoware planning_simulator shortcuts =====
alias aw_sim_mpc='aw_env && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/Shinjuku-Map/map data_path:=$HOME/autoware_data vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lateral_controller_mode:=mpc'


alias aw_sim_pp='aw_env && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/Shinjuku-Map/map data_path:=$HOME/autoware_data vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lateral_controller_mode:=pure_pursuit'

               
