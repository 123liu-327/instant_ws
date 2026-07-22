# 方式 1: 和原始流程一致的入口（推荐）
roslaunch iden_controller cruise_navfn.launch
rosrun iden_controller process_navfn.py

roslaunch iden_controller cruise_navfn_v2_wide.launch
rosrun iden_controller cruise_demo.py

 roslaunch iden_controller cruise_navfn_success1.launch
 rosrun iden_controller process_navfn_success1.py

roslaunch iden_controller cruise_navfn_success2.launch
 rosrun iden_controller process_navfn_success2.py

roslaunch iden_controller global_first_graph_nav.launch

~/instant_ws/src/iden_controller/scripts/start_subtask1_xunfei2026_complete_delivery_v1.sh

./src/iden_controller/scripts/start_subtask1_xunfei2026_virtual_collaboration_v1.sh

 bash ~/instant_ws/src/iden_controller/scripts/start_subtask1_xunfei2026_complete_delivery_anchor_coverage_parking_lock_v2.sh

  bash ~/instant_ws/src/iden_controller/scripts/start_subtask1_xunfei2026_complete_delivery_src2_full_v6.sh

  bash ~/instant_ws/src3/start_src3_with_legacy_voice_spark_v1.sh