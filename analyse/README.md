# analyse

Offline trajectory analysis package for CSV logs.

Run:

```bash
ros2 run analyse error_analysis --ros-args \
  -p estimate_csv:=/Data/fgo_state_optimized.csv \
  -p reference_csv:=/Data/novatel_bestpos_status.csv \
  -p reference_filter:=rtk_fixed_if_available \
  -p output_csv:=/Data/error_metrics.csv
```

The tool accepts CSV files with either ECEF columns (`x_ecef_m`, `y_ecef_m`, `z_ecef_m`) or geodetic columns
(`latitude_deg`, `longitude_deg`, `altitude_m`). Yaw columns are optional.

When the reference CSV contains RTK status columns from `fgo_logger` (`is_rtk_fixed`) or raw NovAtel status columns
(`sol_status`, `pos_type`), `reference_filter:=rtk_fixed_if_available` evaluates the estimate only against RTK Fixed
reference points. Use `reference_filter:=rtk_fixed` to require those columns, or `reference_filter:=none` to use all
reference rows. The metrics CSV also reports the RTK Fixed ratio used for the reference trajectory.

Position plots:

```bash
ros2 run analyse position_analysis --ros-args \
  -p input_csvs:=/Data/fgo_state_optimized.csv \
  -p input_labels:=ours \
  -p reference_csv:=/Data/novatel_bestpos_status.csv \
  -p reference_filter:=rtk_fixed_if_available \
  -p output_dir:=/Data/position_analysis
```

This writes `coordinates.png`, `attitude.png`, and `velocity.png`. The reference CSV is filtered to RTK Fixed points
when RTK status columns are available. The attitude and velocity plots require the corresponding columns in the input
CSV; new `fgo_logger` outputs include roll/pitch/yaw and NED velocity columns.
