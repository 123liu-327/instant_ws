
全流程:
```
<!-- sudo bash ~/ucar_ws/src/overclock.sh -->

roslaunch ucar_controller all_process.launch

rosrun ucar_controller process_5.5.1.py
rostopic pub /follow_begin std_msgs/String "data: 'YLeft'" 



rostopic pub /follow_begin std_msgs/String "data: 'Middle'" 
catkin_make -DCATKIN_WHITELIST_PACKAGES="flow_end" --force-cmake
```
#注意修改导航启动文件是在src/ucar_nav/launch/ucar_navigation_test.launch

#注意主程序可能随时更改，以README里实际运行的文件为准

#修改代价地图参数是在src/ucar_nav/launch/config/move_base/costmap_common_params.yaml

#注意建好图后进入/home/ucar/instant_ws/src/ucar_nav/maps/使用命令行
```
rosrun map_server map_saver -f 5.4.1
```
就可以完成保存


风扇：
```
<!-- sudo bash ~/ucar_ws/src/pwm_fan.sh -->
```