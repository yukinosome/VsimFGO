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
class TrajectorySample:
    time_s: float
    ecef_m: Tuple[float, float, float]
    yaw_rad: Optional[float] = None


@dataclass
class TrajectoryCsvStats:
    total_count: int
    rtk_status_count: int
    rtk_fixed_count: int
    used_count: int
    filter_mode: str

    @property
    def rtk_fixed_ratio(self) -> Optional[float]:
        if self.rtk_status_count == 0:
            return None
        return self.rtk_fixed_count / self.rtk_status_count


@dataclass
class ErrorMetrics:
    count: int
    start_time_s: float
    end_time_s: float
    mean_2d_pos_error_m: float
    std_2d_pos_error_m: float
    max_2d_pos_error_m: float
    mean_3d_pos_error_m: float
    std_3d_pos_error_m: float
    max_3d_pos_error_m: float
    mean_yaw_error_deg: Optional[float]
    std_yaw_error_deg: Optional[float]
    max_yaw_error_deg: Optional[float]
    smoothness_s: float
    reference_total_count: int
    reference_rtk_status_count: int
    reference_rtk_fixed_count: int
    reference_rtk_fixed_ratio: Optional[float]
    reference_used_count: int


class ErrorAnalysisNode(Node):
    """Offline trajectory metrics for fgo_logger CSV outputs."""

    _A = 6378137.0
    _F = 1.0 / 298.257223563
    _B = _A * (1.0 - _F)
    _E2 = _F * (2.0 - _F)
    _EP2 = (_A * _A - _B * _B) / (_B * _B)

    def __init__(self) -> None:
        super().__init__('error_analysis')

        self.declare_parameter('estimate_csv', '')
        self.declare_parameter('reference_csv', '')
        self.declare_parameter('output_csv', '')
        self.declare_parameter('output_errors_csv', '')
        self.declare_parameter('max_time_difference_s', 0.05)
        self.declare_parameter('smoothness_mode', 'sum_squared_jerk')
        self.declare_parameter('reference_filter', 'rtk_fixed_if_available')

        estimate_csv = self._param_str('estimate_csv')
        reference_csv = self._param_str('reference_csv')
        output_csv = self._param_str('output_csv')
        output_errors_csv = self._param_str('output_errors_csv')
        max_time_difference_s = self.get_parameter(
            'max_time_difference_s').get_parameter_value().double_value
        smoothness_mode = self._param_str('smoothness_mode')
        reference_filter = self._param_str('reference_filter')

        if not estimate_csv or not reference_csv:
            self.get_logger().error(
                'Please set estimate_csv and reference_csv, for example: '
                'ros2 run analyse error_analysis --ros-args '
                '-p estimate_csv:=/Data/fgo_state_optimized.csv '
                '-p reference_csv:=/Data/reference.csv')
            return

        try:
            estimate = read_trajectory_csv(Path(estimate_csv))
            reference, reference_stats = read_reference_trajectory_csv(
                Path(reference_csv),
                reference_filter=reference_filter,
            )
            metrics, errors = compute_metrics(
                estimate,
                reference,
                max_time_difference_s=max_time_difference_s,
                smoothness_mode=smoothness_mode,
                reference_stats=reference_stats,
            )
        except Exception as exc:
            self.get_logger().error(f'Error analysis failed: {exc}', exc_info=True)
            return

        self._log_metrics(metrics)

        if output_csv:
            write_metrics_csv(Path(output_csv), metrics)
            self.get_logger().info(f'Wrote metrics CSV: {output_csv}')

        if output_errors_csv:
            write_errors_csv(Path(output_errors_csv), errors)
            self.get_logger().info(f'Wrote per-sample error CSV: {output_errors_csv}')

    def _param_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _log_metrics(self, metrics: ErrorMetrics) -> None:
        self.get_logger().info(
            'Trajectory metrics: '
            f'count={metrics.count}, '
            f'mean_2d={metrics.mean_2d_pos_error_m:.4f} m, '
            f'std_2d={metrics.std_2d_pos_error_m:.4f} m, '
            f'max_2d={metrics.max_2d_pos_error_m:.4f} m, '
            f'mean_3d={metrics.mean_3d_pos_error_m:.4f} m, '
            f'std_3d={metrics.std_3d_pos_error_m:.4f} m, '
            f'max_3d={metrics.max_3d_pos_error_m:.4f} m, '
            f'smoothness={metrics.smoothness_s:.6g}')
        if metrics.reference_rtk_fixed_ratio is None:
            self.get_logger().warn(
                'Reference RTK ratio unavailable: no RTK status columns found '
                '(expected is_rtk_fixed or sol_status/pos_type).')
        else:
            self.get_logger().info(
                'Reference RTK Fixed ratio: '
                f'{metrics.reference_rtk_fixed_ratio * 100.0:.2f}% '
                f'({metrics.reference_rtk_fixed_count}/'
                f'{metrics.reference_rtk_status_count}); '
                f'reference_used={metrics.reference_used_count}/'
                f'{metrics.reference_total_count}')
        if metrics.mean_yaw_error_deg is None:
            self.get_logger().warn('Yaw metrics unavailable: yaw/heading columns were not present in both CSV files.')
        else:
            self.get_logger().info(
                'Yaw metrics: '
                f'mean={metrics.mean_yaw_error_deg:.4f} deg, '
                f'std={metrics.std_yaw_error_deg:.4f} deg, '
                f'max={metrics.max_yaw_error_deg:.4f} deg')


def read_trajectory_csv(path: Path) -> List[TrajectorySample]:
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open('r', newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f'{path} has no CSV header')

        samples = []
        for row_index, row in enumerate(reader, start=2):
            if _row_is_empty(row):
                continue
            try:
                time_s = _read_time(row)
                ecef = _read_ecef(row)
                yaw = _read_yaw(row)
            except ValueError as exc:
                raise ValueError(f'{path}:{row_index}: {exc}') from exc
            samples.append(TrajectorySample(time_s=time_s, ecef_m=ecef, yaw_rad=yaw))

    samples.sort(key=lambda sample: sample.time_s)
    if len(samples) < 2:
        raise ValueError(f'{path} must contain at least two valid samples')
    return _deduplicate_times(samples)


def read_reference_trajectory_csv(path: Path, reference_filter: str) -> Tuple[List[TrajectorySample], TrajectoryCsvStats]:
    if reference_filter not in ('none', 'rtk_fixed', 'rtk_fixed_if_available'):
        raise ValueError("reference_filter must be 'none', 'rtk_fixed', or 'rtk_fixed_if_available'")
    return _read_trajectory_csv_with_stats(path, reference_filter=reference_filter)


def _read_trajectory_csv_with_stats(
    path: Path,
    reference_filter: str,
) -> Tuple[List[TrajectorySample], TrajectoryCsvStats]:
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open('r', newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f'{path} has no CSV header')

        samples = []
        total_count = 0
        rtk_status_count = 0
        rtk_fixed_count = 0
        for row_index, row in enumerate(reader, start=2):
            if _row_is_empty(row):
                continue
            total_count += 1
            try:
                is_rtk_fixed = _read_rtk_fixed_status(row)
                if is_rtk_fixed is not None:
                    rtk_status_count += 1
                    if is_rtk_fixed:
                        rtk_fixed_count += 1

                use_row = True
                if reference_filter == 'rtk_fixed':
                    if is_rtk_fixed is None:
                        raise ValueError(
                            'reference_filter=rtk_fixed requires is_rtk_fixed or sol_status/pos_type columns')
                    use_row = is_rtk_fixed
                elif reference_filter == 'rtk_fixed_if_available' and is_rtk_fixed is not None:
                    use_row = is_rtk_fixed

                if not use_row:
                    continue

                time_s = _read_time(row)
                ecef = _read_ecef(row)
                yaw = _read_yaw(row)
            except ValueError as exc:
                raise ValueError(f'{path}:{row_index}: {exc}') from exc
            samples.append(TrajectorySample(time_s=time_s, ecef_m=ecef, yaw_rad=yaw))

    samples.sort(key=lambda sample: sample.time_s)
    samples = _deduplicate_times(samples)
    if len(samples) < 2:
        raise ValueError(f'{path} must contain at least two valid samples after filtering')

    stats = TrajectoryCsvStats(
        total_count=total_count,
        rtk_status_count=rtk_status_count,
        rtk_fixed_count=rtk_fixed_count,
        used_count=len(samples),
        filter_mode=reference_filter,
    )
    return samples, stats


def compute_metrics(
    estimate: Sequence[TrajectorySample],
    reference: Sequence[TrajectorySample],
    max_time_difference_s: float,
    smoothness_mode: str,
    reference_stats: Optional[TrajectoryCsvStats] = None,
) -> Tuple[ErrorMetrics, List[Dict[str, Optional[float]]]]:
    origin_llh = ecef_to_llh_rad(reference[0].ecef_m)
    errors = []
    err_2d = []
    err_3d = []
    err_yaw = []

    for est in estimate:
        ref = interpolate_reference(reference, est.time_s, max_time_difference_s)
        if ref is None:
            continue

        est_enu = ecef_to_enu(est.ecef_m, reference[0].ecef_m, origin_llh)
        ref_enu = ecef_to_enu(ref.ecef_m, reference[0].ecef_m, origin_llh)

        de = est_enu[0] - ref_enu[0]
        dn = est_enu[1] - ref_enu[1]
        du = est_enu[2] - ref_enu[2]
        e2d = math.hypot(de, dn)
        e3d = math.sqrt(de * de + dn * dn + du * du)

        yaw_error_deg = None
        if est.yaw_rad is not None and ref.yaw_rad is not None:
            yaw_error_deg = abs(math.degrees(wrap_angle_rad(est.yaw_rad - ref.yaw_rad)))
            err_yaw.append(yaw_error_deg)

        err_2d.append(e2d)
        err_3d.append(e3d)
        errors.append({
            'time_s': est.time_s,
            'reference_time_s': ref.time_s,
            'east_error_m': de,
            'north_error_m': dn,
            'up_error_m': du,
            'error_2d_m': e2d,
            'error_3d_m': e3d,
            'yaw_error_deg': yaw_error_deg,
        })

    if not err_2d:
        raise ValueError('No overlapping samples after time alignment')

    smoothness = compute_smoothness(estimate, reference[0].ecef_m, origin_llh, smoothness_mode)

    metrics = ErrorMetrics(
        count=len(err_2d),
        start_time_s=errors[0]['time_s'],
        end_time_s=errors[-1]['time_s'],
        mean_2d_pos_error_m=mean(err_2d),
        std_2d_pos_error_m=std(err_2d),
        max_2d_pos_error_m=max(err_2d),
        mean_3d_pos_error_m=mean(err_3d),
        std_3d_pos_error_m=std(err_3d),
        max_3d_pos_error_m=max(err_3d),
        mean_yaw_error_deg=mean(err_yaw) if err_yaw else None,
        std_yaw_error_deg=std(err_yaw) if err_yaw else None,
        max_yaw_error_deg=max(err_yaw) if err_yaw else None,
        smoothness_s=smoothness,
        reference_total_count=reference_stats.total_count if reference_stats else len(reference),
        reference_rtk_status_count=reference_stats.rtk_status_count if reference_stats else 0,
        reference_rtk_fixed_count=reference_stats.rtk_fixed_count if reference_stats else 0,
        reference_rtk_fixed_ratio=reference_stats.rtk_fixed_ratio if reference_stats else None,
        reference_used_count=reference_stats.used_count if reference_stats else len(reference),
    )
    return metrics, errors


def interpolate_reference(
    reference: Sequence[TrajectorySample],
    time_s: float,
    max_time_difference_s: float,
) -> Optional[TrajectorySample]:
    if time_s < reference[0].time_s or time_s > reference[-1].time_s:
        return None

    lo = 0
    hi = len(reference) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if reference[mid].time_s < time_s:
            lo = mid + 1
        else:
            hi = mid - 1

    if lo == 0:
        return reference[0] if abs(reference[0].time_s - time_s) <= max_time_difference_s else None
    if lo >= len(reference):
        return reference[-1] if abs(reference[-1].time_s - time_s) <= max_time_difference_s else None

    before = reference[lo - 1]
    after = reference[lo]
    nearest_dt = min(abs(time_s - before.time_s), abs(after.time_s - time_s))
    if nearest_dt > max_time_difference_s:
        return None

    dt = after.time_s - before.time_s
    if dt <= 0.0:
        return before
    ratio = (time_s - before.time_s) / dt
    ecef = tuple(before.ecef_m[i] + ratio * (after.ecef_m[i] - before.ecef_m[i]) for i in range(3))

    yaw = None
    if before.yaw_rad is not None and after.yaw_rad is not None:
        yaw = before.yaw_rad + ratio * wrap_angle_rad(after.yaw_rad - before.yaw_rad)
        yaw = wrap_angle_rad(yaw)

    return TrajectorySample(time_s=time_s, ecef_m=ecef, yaw_rad=yaw)


def compute_smoothness(
    samples: Sequence[TrajectorySample],
    origin_ecef: Tuple[float, float, float],
    origin_llh: Tuple[float, float, float],
    mode: str,
) -> float:
    if len(samples) < 4:
        return 0.0

    enu = [ecef_to_enu(sample.ecef_m, origin_ecef, origin_llh) for sample in samples]
    times = [sample.time_s for sample in samples]
    jerk_norm_sq = []

    for idx in range(1, len(samples) - 2):
        dt = (times[idx + 2] - times[idx - 1]) / 3.0
        if dt <= 0.0:
            continue
        jerk = []
        for axis in range(3):
            third_difference = (
                enu[idx + 2][axis]
                - 3.0 * enu[idx + 1][axis]
                + 3.0 * enu[idx][axis]
                - enu[idx - 1][axis]
            )
            jerk.append(third_difference / (dt ** 3))
        jerk_norm_sq.append(sum(value * value for value in jerk))

    if not jerk_norm_sq:
        return 0.0
    if mode == 'mean_squared_jerk':
        return mean(jerk_norm_sq)
    if mode == 'sum_squared_jerk':
        return sum(jerk_norm_sq)
    raise ValueError("smoothness_mode must be 'sum_squared_jerk' or 'mean_squared_jerk'")


def write_metrics_csv(path: Path, metrics: ErrorMetrics) -> None:
    path.expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            'count',
            'start_time_s',
            'end_time_s',
            'mean_2d_pos_error_m',
            'std_2d_pos_error_m',
            'max_2d_pos_error_m',
            'mean_3d_pos_error_m',
            'std_3d_pos_error_m',
            'max_3d_pos_error_m',
            'mean_yaw_error_deg',
            'std_yaw_error_deg',
            'max_yaw_error_deg',
            'smoothness_s',
            'reference_total_count',
            'reference_rtk_status_count',
            'reference_rtk_fixed_count',
            'reference_rtk_fixed_ratio_percent',
            'reference_used_count',
        ])
        writer.writerow([
            metrics.count,
            f'{metrics.start_time_s:.9f}',
            f'{metrics.end_time_s:.9f}',
            f'{metrics.mean_2d_pos_error_m:.9f}',
            f'{metrics.std_2d_pos_error_m:.9f}',
            f'{metrics.max_2d_pos_error_m:.9f}',
            f'{metrics.mean_3d_pos_error_m:.9f}',
            f'{metrics.std_3d_pos_error_m:.9f}',
            f'{metrics.max_3d_pos_error_m:.9f}',
            _optional_float(metrics.mean_yaw_error_deg),
            _optional_float(metrics.std_yaw_error_deg),
            _optional_float(metrics.max_yaw_error_deg),
            f'{metrics.smoothness_s:.9f}',
            metrics.reference_total_count,
            metrics.reference_rtk_status_count,
            metrics.reference_rtk_fixed_count,
            _optional_float(
                metrics.reference_rtk_fixed_ratio * 100.0
                if metrics.reference_rtk_fixed_ratio is not None
                else None),
            metrics.reference_used_count,
        ])


def write_errors_csv(path: Path, errors: Sequence[Dict[str, Optional[float]]]) -> None:
    path.expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        'time_s',
        'reference_time_s',
        'east_error_m',
        'north_error_m',
        'up_error_m',
        'error_2d_m',
        'error_3d_m',
        'yaw_error_deg',
    ]
    with path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in errors:
            writer.writerow({key: _optional_float(row[key]) for key in fieldnames})


def _read_time(row: Dict[str, str]) -> float:
    if _has(row, 'stamp_sec') and _has(row, 'stamp_nanosec'):
        return _float(row, 'stamp_sec') + _float(row, 'stamp_nanosec') * 1e-9
    for name in ('time_s', 'timestamp_s', 'timestamp', 'stamp', 't'):
        if _has(row, name):
            return _float(row, name)
    raise ValueError('missing time column: use stamp_sec/stamp_nanosec or time_s')


def _read_ecef(row: Dict[str, str]) -> Tuple[float, float, float]:
    if _has_any(row, ('x_ecef_m', 'ecef_x_m', 'x')):
        x = _float_any(row, ('x_ecef_m', 'ecef_x_m', 'x'))
        y = _float_any(row, ('y_ecef_m', 'ecef_y_m', 'y'))
        z = _float_any(row, ('z_ecef_m', 'ecef_z_m', 'z'))
        return x, y, z

    lat = _float_any(row, ('latitude_deg', 'lat_deg', 'latitude', 'lat'))
    lon = _float_any(row, ('longitude_deg', 'lon_deg', 'longitude', 'lon'))
    alt = _read_altitude_m(row)

    if abs(lat) <= math.pi and abs(lon) <= 2.0 * math.pi:
        return llh_to_ecef_rad(lat, lon, alt)
    return llh_to_ecef_rad(math.radians(lat), math.radians(lon), alt)


def _read_altitude_m(row: Dict[str, str]) -> float:
    if _has(row, 'altitude_m'):
        return _float(row, 'altitude_m')
    if _has(row, 'height_m') and not _has(row, 'undulation_m'):
        return _float(row, 'height_m')
    if _has(row, 'height_msl_m') and _has(row, 'undulation_m'):
        return _float(row, 'height_msl_m') + _float(row, 'undulation_m')
    if _has(row, 'hgt') and _has(row, 'undulation'):
        return _float(row, 'hgt') + _float(row, 'undulation')
    return _float_any(row, ('height_m', 'height_msl_m', 'hgt', 'alt'))


def _read_yaw(row: Dict[str, str]) -> Optional[float]:
    for name in ('yaw_rad', 'heading_rad'):
        if _has(row, name):
            return wrap_angle_rad(_float(row, name))
    for name in ('yaw_deg', 'heading_deg', 'yaw', 'heading'):
        if _has(row, name):
            value = _float(row, name)
            if abs(value) <= 2.0 * math.pi:
                return wrap_angle_rad(value)
            return wrap_angle_rad(math.radians(value))
    return None


def llh_to_ecef_rad(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = ErrorAnalysisNode._A / math.sqrt(1.0 - ErrorAnalysisNode._E2 * sin_lat * sin_lat)
    x = (n + alt) * cos_lat * math.cos(lon)
    y = (n + alt) * cos_lat * math.sin(lon)
    z = (n * (1.0 - ErrorAnalysisNode._E2) + alt) * sin_lat
    return x, y, z


def ecef_to_llh_rad(ecef: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = ecef
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * ErrorAnalysisNode._A, p * ErrorAnalysisNode._B)
    lat = math.atan2(
        z + ErrorAnalysisNode._EP2 * ErrorAnalysisNode._B * math.sin(theta) ** 3,
        p - ErrorAnalysisNode._E2 * ErrorAnalysisNode._A * math.cos(theta) ** 3,
    )
    for _ in range(5):
        sin_lat = math.sin(lat)
        n = ErrorAnalysisNode._A / math.sqrt(1.0 - ErrorAnalysisNode._E2 * sin_lat * sin_lat)
        lat = math.atan2(z + ErrorAnalysisNode._E2 * n * sin_lat, p)

    sin_lat = math.sin(lat)
    n = ErrorAnalysisNode._A / math.sqrt(1.0 - ErrorAnalysisNode._E2 * sin_lat * sin_lat)
    alt = p / max(math.cos(lat), 1e-12) - n
    return lat, lon, alt


def ecef_to_enu(
    ecef: Tuple[float, float, float],
    origin_ecef: Tuple[float, float, float],
    origin_llh: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    lat0, lon0, _ = origin_llh
    dx = ecef[0] - origin_ecef[0]
    dy = ecef[1] - origin_ecef[1]
    dz = ecef[2] - origin_ecef[2]

    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def wrap_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values)


def std(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    return math.sqrt(sum((value - mu) ** 2 for value in values) / (len(values) - 1))


def _deduplicate_times(samples: Sequence[TrajectorySample]) -> List[TrajectorySample]:
    deduped = []
    last_time = None
    for sample in samples:
        if last_time is not None and abs(sample.time_s - last_time) < 1e-12:
            deduped[-1] = sample
        else:
            deduped.append(sample)
            last_time = sample.time_s
    return deduped


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


def _optional_float(value: Optional[float]) -> str:
    if value is None:
        return ''
    return f'{value:.9f}'


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError('rclpy is not available. Source your ROS 2 environment before running this node.')
    rclpy.init(args=args)
    node = ErrorAnalysisNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
