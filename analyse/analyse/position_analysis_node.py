import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    rclpy = None

    class Node:  # type: ignore[no-redef]
        pass


@dataclass
class PlotSeries:
    label: str
    time_s: List[float]
    latitude_deg: Optional[List[float]] = None
    longitude_deg: Optional[List[float]] = None
    altitude_m: Optional[List[float]] = None
    roll_deg: Optional[List[float]] = None
    pitch_deg: Optional[List[float]] = None
    yaw_deg: Optional[List[float]] = None
    vel_north_mps: Optional[List[float]] = None
    vel_east_mps: Optional[List[float]] = None
    vel_down_mps: Optional[List[float]] = None


class PositionAnalysisNode(Node):
    """Plot WGS84 coordinates, NED attitude, and NED velocity from CSV logs."""

    def __init__(self) -> None:
        super().__init__('position_analysis')

        self.declare_parameter('input_csvs', '/Data/fgo_state_optimized.csv')
        self.declare_parameter('input_labels', 'estimate')
        self.declare_parameter('reference_csv', '')
        self.declare_parameter('reference_label', 'NovAtel RTK Fixed')
        self.declare_parameter('reference_filter', 'rtk_fixed_if_available')
        self.declare_parameter('output_dir', '/Data/position_analysis')
        self.declare_parameter('time_mode', 'relative')
        self.declare_parameter('start_time_s', '')
        self.declare_parameter('end_time_s', '')
        self.declare_parameter('dpi', 180)

        input_csvs = _split_list(self._param_str('input_csvs'))
        input_labels = _split_list(self._param_str('input_labels'))
        reference_csv = self._param_str('reference_csv')
        reference_label = self._param_str('reference_label')
        reference_filter = self._param_str('reference_filter')
        output_dir = Path(self._param_str('output_dir')).expanduser().resolve()
        time_mode = self._param_str('time_mode')
        start_time_s = _optional_param_float(self._param_str('start_time_s'))
        end_time_s = _optional_param_float(self._param_str('end_time_s'))
        dpi = self.get_parameter('dpi').get_parameter_value().integer_value

        if not input_csvs and not reference_csv:
            self.get_logger().error('Set input_csvs and/or reference_csv.')
            return
        if time_mode not in ('relative', 'absolute'):
            self.get_logger().error("time_mode must be 'relative' or 'absolute'")
            return

        try:
            series = []
            for index, csv_path in enumerate(input_csvs):
                label = input_labels[index] if index < len(input_labels) else Path(csv_path).stem
                series.append(read_plot_series(
                    Path(csv_path),
                    label=label,
                    reference_filter='none',
                    start_time_s=start_time_s,
                    end_time_s=end_time_s,
                ))

            if reference_csv:
                series.append(read_plot_series(
                    Path(reference_csv),
                    label=reference_label,
                    reference_filter=reference_filter,
                    start_time_s=start_time_s,
                    end_time_s=end_time_s,
                ))

            normalize_time(series, mode=time_mode)
            output_dir.mkdir(parents=True, exist_ok=True)
            plot_coordinate_series(series, output_dir / 'coordinates.png', dpi=dpi)
            plot_attitude_series(series, output_dir / 'attitude.png', dpi=dpi)
            plot_velocity_series(series, output_dir / 'velocity.png', dpi=dpi)
        except Exception as exc:
            self.get_logger().error(f'Position analysis failed: {exc}', exc_info=True)
            return

        self.get_logger().info(f'Wrote coordinate plot: {output_dir / "coordinates.png"}')
        self.get_logger().info(f'Wrote attitude plot: {output_dir / "attitude.png"}')
        self.get_logger().info(f'Wrote velocity plot: {output_dir / "velocity.png"}')

    def _param_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value


def read_plot_series(
    path: Path,
    label: str,
    reference_filter: str,
    start_time_s: Optional[float],
    end_time_s: Optional[float],
) -> PlotSeries:
    if reference_filter not in ('none', 'rtk_fixed', 'rtk_fixed_if_available'):
        raise ValueError("reference_filter must be 'none', 'rtk_fixed', or 'rtk_fixed_if_available'")
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open('r', newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f'{path} has no CSV header')

        rows = []
        for row_index, row in enumerate(reader, start=2):
            if _row_is_empty(row):
                continue
            try:
                time_s = _read_time(row)
                if start_time_s is not None and time_s < start_time_s:
                    continue
                if end_time_s is not None and time_s > end_time_s:
                    continue
                rtk_status = _read_rtk_fixed_status(row)
                if reference_filter == 'rtk_fixed':
                    if rtk_status is None:
                        raise ValueError(
                            'reference_filter=rtk_fixed requires is_rtk_fixed or sol_status/pos_type columns')
                    if not rtk_status:
                        continue
                elif reference_filter == 'rtk_fixed_if_available' and rtk_status is not None and not rtk_status:
                    continue
                rows.append((time_s, row))
            except ValueError as exc:
                raise ValueError(f'{path}:{row_index}: {exc}') from exc

    if not rows:
        raise ValueError(f'{path} has no rows after filtering')

    rows.sort(key=lambda item: item[0])
    times = [item[0] for item in rows]
    row_values = [item[1] for item in rows]

    lat, lon, alt = _read_llh_columns(row_values)
    roll, pitch, yaw = _read_attitude_columns(row_values)
    vn, ve, vd = _read_velocity_columns(row_values, lat, lon)
    return PlotSeries(
        label=label,
        time_s=times,
        latitude_deg=lat,
        longitude_deg=lon,
        altitude_m=alt,
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        vel_north_mps=vn,
        vel_east_mps=ve,
        vel_down_mps=vd,
    )


def normalize_time(series: Sequence[PlotSeries], mode: str) -> None:
    if mode == 'absolute':
        return
    first_time = min(sample.time_s[0] for sample in series if sample.time_s)
    for sample in series:
        sample.time_s = [time_s - first_time for time_s in sample.time_s]


def plot_coordinate_series(series: Sequence[PlotSeries], output_path: Path, dpi: int) -> None:
    plt = _load_pyplot()
    fig, axes = plt.subplots(3, 1, figsize=(9.0, 5.4), sharex=True)
    _plot_axis(axes[0], series, 'latitude_deg', 'Latitude', '(deg)')
    _plot_axis(axes[1], series, 'longitude_deg', 'Longitude', '(deg)')
    _plot_axis(axes[2], series, 'altitude_m', 'Height', '(m)')
    axes[2].set_xlabel('Time (sec)')
    _finish_figure(fig, axes, output_path, dpi)


def plot_attitude_series(series: Sequence[PlotSeries], output_path: Path, dpi: int) -> None:
    plt = _load_pyplot()
    fig, axes = plt.subplots(3, 1, figsize=(9.0, 5.4), sharex=True)
    _plot_axis(axes[0], series, 'roll_deg', 'Roll', '(deg)')
    _plot_axis(axes[1], series, 'pitch_deg', 'Pitch', '(deg)')
    _plot_axis(axes[2], series, 'yaw_deg', 'Yaw', '(deg)')
    axes[2].set_xlabel('Time (sec)')
    _finish_figure(fig, axes, output_path, dpi)


def plot_velocity_series(series: Sequence[PlotSeries], output_path: Path, dpi: int) -> None:
    plt = _load_pyplot()
    fig, axes = plt.subplots(3, 1, figsize=(9.0, 5.4), sharex=True)
    _plot_axis(axes[0], series, 'vel_north_mps', 'Vn', '(m/s)')
    _plot_axis(axes[1], series, 'vel_east_mps', 'Ve', '(m/s)')
    _plot_axis(axes[2], series, 'vel_down_mps', 'Vd', '(m/s)')
    axes[2].set_xlabel('Time (sec)')
    _finish_figure(fig, axes, output_path, dpi)


def _plot_axis(axis, series: Sequence[PlotSeries], field_name: str, title: str, unit: str) -> None:
    plotted = False
    for sample in series:
        values = getattr(sample, field_name)
        if values is None:
            continue
        axis.plot(sample.time_s, values, linewidth=1.4, label=sample.label)
        plotted = True
    axis.set_title(title, fontsize=10, fontweight='bold', pad=3)
    axis.set_ylabel(unit, rotation=0, labelpad=20)
    axis.grid(True, linewidth=0.5, alpha=0.45)
    if not plotted:
        axis.text(0.5, 0.5, f'No {title} data', ha='center', va='center', transform=axis.transAxes)


def _finish_figure(fig, axes, output_path: Path, dpi: int) -> None:
    handles, labels = [], []
    for axis in axes:
        axis.tick_params(labelsize=8)
        axis.label_outer()
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        for handle, label in zip(axis_handles, axis_labels):
            if label not in labels:
                handles.append(handle)
                labels.append(label)
    if handles:
        fig.legend(handles, labels, loc='upper center', ncol=min(4, len(labels)), fontsize=8)
        fig.subplots_adjust(top=0.88)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches='tight')
    _load_pyplot().close(fig)


def _read_llh_columns(rows: Sequence[Dict[str, str]]):
    if not rows or not _has_any(rows[0], ('latitude_deg', 'lat_deg', 'latitude', 'lat')):
        return None, None, None
    lat = [_float_any(row, ('latitude_deg', 'lat_deg', 'latitude', 'lat')) for row in rows]
    lon = [_float_any(row, ('longitude_deg', 'lon_deg', 'longitude', 'lon')) for row in rows]
    alt = [_read_altitude_m(row) for row in rows]
    return lat, lon, alt


def _read_attitude_columns(rows: Sequence[Dict[str, str]]):
    if not rows:
        return None, None, None
    if _has(rows[0], 'roll_deg') and _has(rows[0], 'pitch_deg') and _has(rows[0], 'yaw_deg'):
        return (
            [_float(row, 'roll_deg') for row in rows],
            [_float(row, 'pitch_deg') for row in rows],
            [_float(row, 'yaw_deg') for row in rows],
        )
    if _has(rows[0], 'roll_rad') and _has(rows[0], 'pitch_rad') and _has(rows[0], 'yaw_rad'):
        return (
            [math.degrees(_float(row, 'roll_rad')) for row in rows],
            [math.degrees(_float(row, 'pitch_rad')) for row in rows],
            [math.degrees(_float(row, 'yaw_rad')) for row in rows],
        )
    if all(_has(rows[0], name) for name in ('orientation_x', 'orientation_y', 'orientation_z', 'orientation_w')):
        roll, pitch, yaw = [], [], []
        for row in rows:
            rpy = _quaternion_to_rpy_rad(
                _float(row, 'orientation_x'),
                _float(row, 'orientation_y'),
                _float(row, 'orientation_z'),
                _float(row, 'orientation_w'),
            )
            roll.append(math.degrees(rpy[0]))
            pitch.append(math.degrees(rpy[1]))
            yaw.append(math.degrees(rpy[2]))
        return roll, pitch, yaw
    if _has(rows[0], 'yaw_deg'):
        return None, None, [_float(row, 'yaw_deg') for row in rows]
    if _has(rows[0], 'yaw_rad'):
        return None, None, [math.degrees(_float(row, 'yaw_rad')) for row in rows]
    return None, None, None


def _read_velocity_columns(
    rows: Sequence[Dict[str, str]],
    latitude_deg: Optional[Sequence[float]],
    longitude_deg: Optional[Sequence[float]],
):
    if not rows:
        return None, None, None
    if all(_has(rows[0], name) for name in ('vel_north_mps', 'vel_east_mps', 'vel_down_mps')):
        return (
            [_float(row, 'vel_north_mps') for row in rows],
            [_float(row, 'vel_east_mps') for row in rows],
            [_float(row, 'vel_down_mps') for row in rows],
        )
    if all(_has(rows[0], name) for name in ('vn_mps', 've_mps', 'vd_mps')):
        return (
            [_float(row, 'vn_mps') for row in rows],
            [_float(row, 've_mps') for row in rows],
            [_float(row, 'vd_mps') for row in rows],
        )
    if all(_has(rows[0], name) for name in ('vel_x_ecef_mps', 'vel_y_ecef_mps', 'vel_z_ecef_mps')):
        if latitude_deg is None or longitude_deg is None:
            return None, None, None
        vn, ve, vd = [], [], []
        for row, lat_deg, lon_deg in zip(rows, latitude_deg, longitude_deg):
            north, east, down = _ecef_velocity_to_ned(
                _float(row, 'vel_x_ecef_mps'),
                _float(row, 'vel_y_ecef_mps'),
                _float(row, 'vel_z_ecef_mps'),
                math.radians(lat_deg),
                math.radians(lon_deg),
            )
            vn.append(north)
            ve.append(east)
            vd.append(down)
        return vn, ve, vd
    return None, None, None


def _read_time(row: Dict[str, str]) -> float:
    if _has(row, 'stamp_sec') and _has(row, 'stamp_nanosec'):
        return _float(row, 'stamp_sec') + _float(row, 'stamp_nanosec') * 1e-9
    for name in ('time_s', 'timestamp_s', 'timestamp', 'stamp', 't'):
        if _has(row, name):
            return _float(row, name)
    raise ValueError('missing time column: use stamp_sec/stamp_nanosec or time_s')


def _read_altitude_m(row: Dict[str, str]) -> float:
    if _has(row, 'altitude_m'):
        return _float(row, 'altitude_m')
    if _has(row, 'height_msl_m') and _has(row, 'undulation_m'):
        return _float(row, 'height_msl_m') + _float(row, 'undulation_m')
    if _has(row, 'hgt') and _has(row, 'undulation'):
        return _float(row, 'hgt') + _float(row, 'undulation')
    return _float_any(row, ('height_m', 'height_msl_m', 'hgt', 'alt'))


def _read_rtk_fixed_status(row: Dict[str, str]) -> Optional[bool]:
    if _has(row, 'is_rtk_fixed'):
        return _parse_bool(row['is_rtk_fixed'])
    if _has(row, 'sol_status') and _has(row, 'pos_type'):
        sol_status = int(_float(row, 'sol_status'))
        pos_type = int(_float(row, 'pos_type'))
        return sol_status == 0 and pos_type in (48, 49, 50, 56)
    return None


def _parse_bool(value: str) -> bool:
    text = str(value).strip().lower()
    if text in ('1', 'true', 't', 'yes', 'y'):
        return True
    if text in ('0', 'false', 'f', 'no', 'n'):
        return False
    raise ValueError(f'invalid boolean value: {value}')


def _quaternion_to_rpy_rad(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _ecef_velocity_to_ned(vx: float, vy: float, vz: float, lat: float, lon: float):
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    east = -sin_lon * vx + cos_lon * vy
    north = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
    up = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz
    return north, east, -up


def _split_list(value: str) -> List[str]:
    if not value.strip():
        return []
    normalized = value.replace(';', ',')
    return [item.strip() for item in normalized.split(',') if item.strip()]


def _optional_param_float(value: str) -> Optional[float]:
    text = value.strip()
    return float(text) if text else None


def _row_is_empty(row: Dict[str, str]) -> bool:
    return all(value is None or str(value).strip() == '' for value in row.values())


def _has(row: Dict[str, str], name: str) -> bool:
    return name in row and str(row[name]).strip() != ''


def _has_any(row: Dict[str, str], names: Iterable[str]) -> bool:
    return any(_has(row, name) for name in names)


def _float(row: Dict[str, str], name: str) -> float:
    return float(str(row[name]).strip())


def _float_any(row: Dict[str, str], names: Iterable[str]) -> float:
    for name in names:
        if _has(row, name):
            return _float(row, name)
    raise ValueError(f'missing numeric column; expected one of: {", ".join(names)}')


def _load_pyplot():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError('matplotlib is required: install python3-matplotlib') from exc
    return plt


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError('rclpy is not available. Source your ROS 2 environment before running this node.')
    rclpy.init(args=args)
    node = PositionAnalysisNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
