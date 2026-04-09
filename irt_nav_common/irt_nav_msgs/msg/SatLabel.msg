uint8                     integrity_flag
uint32                    prn
geometry_msgs/Vector3     sat_pos
geometry_msgs/Vector3     sat_vel
float64                   psr
float64                   psr_raw
float64                   psr_satclk_corrected
float64                   psr_dev_measured
float64                   psr_dev_preproc
float64                   dr
float64                   dr_raw
float64                   dr_satclk_corrected
float64                   dr_dev_preproc
float64                   cp
float64                   cp_raw
float64                   cp_satclk_corrected
float64                   cp_dev_measured
float64                   locktime
float64                   psr_correction
float64                   cn0
bool                      cycle_slip

float64                   psr_residual
float64                   dr_residual
float64                   cp_residual
float64                   elevation_angle
float64                   azimuth_angle
float64                   cmc
float64                   drc

# calculated data from VBI
string                    factor

uint8                     dim
float64[]                 residuals
GaussianComponent[]       gaussian
float64                   weight
float64[]                 data

# labels
uint8                     label      # 0 == opensky 1 == Multipath 2 == NLOS 3 == invalid
float64                   label_dev
string                    msg