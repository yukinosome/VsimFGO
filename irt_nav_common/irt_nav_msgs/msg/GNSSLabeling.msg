std_msgs/Header           header
int64                     related_state_timestamp_nanosec
uint64                    counter
float64                   time_receive
# solution type of the pvt: 0 = fixed 1 = float 2 = single
uint8                     solution_type  
SatLabel[]                ant_main_labels
SatLabel[]                ant_aux_labels
uint8[]				      faulty_prn_main
uint8[]					  faulty_prn_aux


bool                      gt_available

# calculated data from fgo

geometry_msgs/Vector3     ant_pos
geometry_msgs/Vector3     ant_vel
geometry_msgs/Vector3     gt_pos
geometry_msgs/Vector3     gt_vel
float64                   clock_bias
float64                   clock_drift
float64                   gt_clock_bias
float64                   gt_clock_drift

# calculated data from VBI

string                    environment