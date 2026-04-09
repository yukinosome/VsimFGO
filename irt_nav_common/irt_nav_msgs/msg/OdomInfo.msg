std_msgs/Header          header
uint64                   counter
uint64                   key_current
uint64                   key_previous

geometry_msgs/Vector3    odom_anchor_ecef_pos
geometry_msgs/Vector3    odom_anchor_ecef_to_enu_rpy
float64                  odom_anchor_yaw_offset

float64                  associated_state_timestamp_i
float64                  associated_state_timestamp_j
float64                  duration_to_i
bool                     associated_with_i
bool                     associated_with_j

geometry_msgs/Vector3    odom_local_pos
geometry_msgs/Vector3    odom_local_rpy
geometry_msgs/Vector3    odom_ecef_pos
geometry_msgs/Vector3    odom_ecef_rpy

geometry_msgs/Vector3    sensor_local_pos_from
geometry_msgs/Vector3    sensor_local_rpy_from
geometry_msgs/Vector3    sensor_enu_pos_from
geometry_msgs/Vector3    sensor_enu_rpy_from
geometry_msgs/Vector3    sensor_ecef_pos_from
geometry_msgs/Vector3    sensor_ecef_rpy_from
geometry_msgs/Vector3    sensor_local_pos_to
geometry_msgs/Vector3    sensor_local_rpy_to
geometry_msgs/Vector3    sensor_enu_pos_to
geometry_msgs/Vector3    sensor_enu_rpy_to
geometry_msgs/Vector3    sensor_ecef_pos_to
geometry_msgs/Vector3    sensor_ecef_rpy_to

geometry_msgs/Vector3    sensor_local_pos_to_optimized
geometry_msgs/Vector3    sensor_local_rpy_to_optimized
geometry_msgs/Vector3    sensor_enu_pos_to_optimized
geometry_msgs/Vector3    sensor_enu_rpy_to_optimized
geometry_msgs/Vector3    sensor_ecef_pos_to_optimized
geometry_msgs/Vector3    sensor_ecef_rpy_to_optimized


               




