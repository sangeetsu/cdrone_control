source ~/.bashrc
ros2 launch drone_bringup external_pose_px4_bridge.launch.py \
  pose_source:=optitrack \
  optitrack_server:=192.168.0.217 \
  rigid_body_name:=RigidBody3


source ~/.bashrc
ros2 run drone_control_pkg keyboard_teleop_node


ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
