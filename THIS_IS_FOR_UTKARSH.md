This is the initial mission command

ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=3 \
  zed_rotate_180:=true \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  rpy_offset_rad:='[0.0, 0.0, -3.141592653589793]'

only change required_completion_count based on number of attack drones



Have the next two commands ready in different terminals



This is the start command:

ros2 service call /cdrone/cdrone3/demo/milestone3_start std_srvs/srv/Trigger "{}"


This is the abort command:

ros2 service call /cdrone/cdrone3/demo/milestone3_abort std_srvs/srv/Trigger "{}"



Note:
BE CAREFUL. THE DRONE FOLLOWS OTHER DRONES TILL 25m NOW. IT COMES VERY COLSE, SO LAND YOUR DRONES COMPLETELY IN CASES OF EMERGENCY. 