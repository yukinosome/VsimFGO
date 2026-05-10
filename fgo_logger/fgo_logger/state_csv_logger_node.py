import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from irt_nav_msgs.msg import FGOState
from novatel_oem7_msgs.msg import BESTPOS


class StateCsvLoggerNode(Node):
    """Log online_fgo optimized state in ECEF and converted LLH into CSV."""

    _NOVATEL_SOL_COMPUTED = 0
    _NOVATEL_RTK_FIXED_TYPES = {48, 49, 50, 56}

    # WGS84 constants
    _A = 6378137.0
    _F = 1.0 / 298.257223563
    _B = _A * (1.0 - _F)
    _E2 = _F * (2.0 - _F)
    _EP2 = (_A * _A - _B * _B) / (_B * _B)

    def __init__(self) -> None:
        super().__init__('csv_logger')

        self.declare_parameter('input_topic', '/deutschland/stateOptimized')
        self.declare_parameter('output_csv_path', '/Data/fgo_state_optimized.csv')
        self.declare_parameter('append', False)
        self.declare_parameter('novatel_bestpos_topic', '/novatel/oem7/bestpos')
        self.declare_parameter('novatel_rtk_csv_path', '/Data/novatel_rtk_fixed.csv')
        self.declare_parameter('novatel_rtk_append', False)
        self.declare_parameter('novatel_status_csv_path', '/Data/novatel_bestpos_status.csv')
        self.declare_parameter('novatel_status_append', False)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_csv_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        self._append = self.get_parameter('append').get_parameter_value().bool_value
        self._novatel_bestpos_topic = self.get_parameter(
            'novatel_bestpos_topic').get_parameter_value().string_value
        self._novatel_rtk_csv_path = self.get_parameter(
            'novatel_rtk_csv_path').get_parameter_value().string_value
        self._novatel_rtk_append = self.get_parameter(
            'novatel_rtk_append').get_parameter_value().bool_value
        self._novatel_status_csv_path = self.get_parameter(
            'novatel_status_csv_path').get_parameter_value().string_value
        self._novatel_status_append = self.get_parameter(
            'novatel_status_append').get_parameter_value().bool_value

        self._row_count = 0
        self._rtk_row_count = 0
        self._bestpos_status_row_count = 0
        self._csv_file = None
        self._csv_writer = None
        self._rtk_csv_file = None
        self._rtk_csv_writer = None
        self._bestpos_status_csv_file = None
        self._bestpos_status_csv_writer = None

        self._open_csv()
        self._open_rtk_csv()
        self._open_bestpos_status_csv()

        self._sub = self.create_subscription(
            FGOState,
            self._input_topic,
            self._on_state,
            100,
        )
        self._bestpos_sub = self.create_subscription(
            BESTPOS,
            self._novatel_bestpos_topic,
            self._on_bestpos,
            100,
        )

        self.get_logger().info(f'Subscribing: {self._input_topic}')
        self.get_logger().info(f'CSV output: {self._output_csv_path}')
        self.get_logger().info(f'Append mode: {self._append}')
        self.get_logger().info(f'Subscribing NovAtel RTK Fixed: {self._novatel_bestpos_topic}')
        self.get_logger().info(f'NovAtel RTK Fixed CSV output: {self._novatel_rtk_csv_path}')
        self.get_logger().info(f'NovAtel RTK Fixed append mode: {self._novatel_rtk_append}')
        self.get_logger().info(f'NovAtel BESTPOS status CSV output: {self._novatel_status_csv_path}')
        self.get_logger().info(f'NovAtel BESTPOS status append mode: {self._novatel_status_append}')

        # Periodic status log to help detect stalls
        self.create_timer(60.0, self._periodic_status_log)

    def _open_csv(self) -> None:
        out_path = Path(self._output_csv_path).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)

        mode = 'a' if self._append else 'w'
        self._csv_file = open(out_path, mode, newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)

        need_header = (not self._append) or (out_path.stat().st_size == 0)
        if need_header:
            self._csv_writer.writerow([
                'stamp_sec',
                'stamp_nanosec',
                'frame_id',
                'x_ecef_m',
                'y_ecef_m',
                'z_ecef_m',
                'latitude_deg',
                'longitude_deg',
                'altitude_m',
                'orientation_x',
                'orientation_y',
                'orientation_z',
                'orientation_w',
                'roll_rad',
                'roll_deg',
                'pitch_rad',
                'pitch_deg',
                'yaw_rad',
                'yaw_deg',
                'heading_msg',
                'vel_x_ecef_mps',
                'vel_y_ecef_mps',
                'vel_z_ecef_mps',
                'vel_north_mps',
                'vel_east_mps',
                'vel_down_mps',
            ])
            self._csv_file.flush()

    def _on_state(self, msg: FGOState) -> None:
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z

            lat_deg, lon_deg, alt = self._ecef_to_llh_deg(x, y, z)
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            roll_rad, pitch_rad, yaw_rad = self._quaternion_to_rpy_rad(qx, qy, qz, qw)
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)
            vx = msg.vel.linear.x
            vy = msg.vel.linear.y
            vz = msg.vel.linear.z
            vel_north, vel_east, vel_down = self._ecef_velocity_to_ned(
                vx,
                vy,
                vz,
                math.radians(lat_deg),
                math.radians(lon_deg),
            )

            self._csv_writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                f'{x:.6f}',
                f'{y:.6f}',
                f'{z:.6f}',
                f'{lat_deg:.10f}',
                f'{lon_deg:.10f}',
                f'{alt:.6f}',
                f'{qx:.12f}',
                f'{qy:.12f}',
                f'{qz:.12f}',
                f'{qw:.12f}',
                f'{roll_rad:.12f}',
                f'{roll_deg:.10f}',
                f'{pitch_rad:.12f}',
                f'{pitch_deg:.10f}',
                f'{yaw_rad:.12f}',
                f'{yaw_deg:.10f}',
                f'{msg.heading:.12f}',
                f'{vx:.9f}',
                f'{vy:.9f}',
                f'{vz:.9f}',
                f'{vel_north:.9f}',
                f'{vel_east:.9f}',
                f'{vel_down:.9f}',
            ])
            self._csv_file.flush()

            self._row_count += 1
            if self._row_count == 1:
                self.get_logger().info('Received first message and wrote first CSV row.')
            elif self._row_count % 100 == 0:
                self.get_logger().info(f'Rows written: {self._row_count}')
        except Exception as e:
            self.get_logger().error(f'Exception in _on_state: {e}', exc_info=True)

    def _open_rtk_csv(self) -> None:
        out_path = Path(self._novatel_rtk_csv_path).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)

        mode = 'a' if self._novatel_rtk_append else 'w'
        self._rtk_csv_file = open(out_path, mode, newline='', encoding='utf-8')
        self._rtk_csv_writer = csv.writer(self._rtk_csv_file)

        need_header = (not self._novatel_rtk_append) or (out_path.stat().st_size == 0)
        if need_header:
            self._rtk_csv_writer.writerow([
                'stamp_sec',
                'stamp_nanosec',
                'frame_id',
                'gps_week_number',
                'gps_week_milliseconds',
                'sol_status',
                'pos_type',
                'latitude_deg',
                'longitude_deg',
                'altitude_m',
                'height_msl_m',
                'undulation_m',
                'x_ecef_m',
                'y_ecef_m',
                'z_ecef_m',
                'lat_stdev_m',
                'lon_stdev_m',
                'hgt_stdev_m',
                'diff_age_s',
                'sol_age_s',
                'num_svs',
                'num_sol_svs',
                'num_sol_l1_svs',
                'num_sol_multi_svs',
                'station_id',
                'ext_sol_stat',
            ])
            self._rtk_csv_file.flush()

    def _on_bestpos(self, msg: BESTPOS) -> None:
        try:
            self._write_bestpos_status(msg)

            if not self._is_novatel_rtk_fixed(msg):
                return

            altitude = msg.hgt + msg.undulation
            x, y, z = self._llh_to_ecef_deg(msg.lat, msg.lon, altitude)

            self._rtk_csv_writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                msg.nov_header.gps_week_number,
                msg.nov_header.gps_week_milliseconds,
                msg.sol_status.status,
                msg.pos_type.type,
                f'{msg.lat:.10f}',
                f'{msg.lon:.10f}',
                f'{altitude:.6f}',
                f'{msg.hgt:.6f}',
                f'{msg.undulation:.6f}',
                f'{x:.6f}',
                f'{y:.6f}',
                f'{z:.6f}',
                f'{msg.lat_stdev:.9f}',
                f'{msg.lon_stdev:.9f}',
                f'{msg.hgt_stdev:.9f}',
                f'{msg.diff_age:.6f}',
                f'{msg.sol_age:.6f}',
                msg.num_svs,
                msg.num_sol_svs,
                msg.num_sol_l1_svs,
                msg.num_sol_multi_svs,
                self._decode_station_id(msg.stn_id),
                msg.ext_sol_stat.status,
            ])
            self._rtk_csv_file.flush()

            self._rtk_row_count += 1
            if self._rtk_row_count == 1:
                self.get_logger().info('Received first NovAtel RTK Fixed point and wrote first CSV row.')
            elif self._rtk_row_count % 100 == 0:
                self.get_logger().info(f'NovAtel RTK Fixed rows written: {self._rtk_row_count}')
        except Exception as e:
            self.get_logger().error(f'Exception in _on_bestpos: {e}', exc_info=True)

    def _is_novatel_rtk_fixed(self, msg: BESTPOS) -> bool:
        return (
            msg.sol_status.status == self._NOVATEL_SOL_COMPUTED
            and msg.pos_type.type in self._NOVATEL_RTK_FIXED_TYPES
        )

    def _open_bestpos_status_csv(self) -> None:
        out_path = Path(self._novatel_status_csv_path).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)

        mode = 'a' if self._novatel_status_append else 'w'
        self._bestpos_status_csv_file = open(out_path, mode, newline='', encoding='utf-8')
        self._bestpos_status_csv_writer = csv.writer(self._bestpos_status_csv_file)

        need_header = (not self._novatel_status_append) or (out_path.stat().st_size == 0)
        if need_header:
            self._bestpos_status_csv_writer.writerow([
                'stamp_sec',
                'stamp_nanosec',
                'frame_id',
                'gps_week_number',
                'gps_week_milliseconds',
                'sol_status',
                'pos_type',
                'is_rtk_fixed',
                'latitude_deg',
                'longitude_deg',
                'height_msl_m',
                'undulation_m',
                'lat_stdev_m',
                'lon_stdev_m',
                'hgt_stdev_m',
                'diff_age_s',
                'sol_age_s',
                'num_svs',
                'num_sol_svs',
                'num_sol_l1_svs',
                'num_sol_multi_svs',
                'station_id',
                'ext_sol_stat',
            ])
            self._bestpos_status_csv_file.flush()

    def _write_bestpos_status(self, msg: BESTPOS) -> None:
        self._bestpos_status_csv_writer.writerow([
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            msg.nov_header.gps_week_number,
            msg.nov_header.gps_week_milliseconds,
            msg.sol_status.status,
            msg.pos_type.type,
            int(self._is_novatel_rtk_fixed(msg)),
            f'{msg.lat:.10f}',
            f'{msg.lon:.10f}',
            f'{msg.hgt:.6f}',
            f'{msg.undulation:.6f}',
            f'{msg.lat_stdev:.9f}',
            f'{msg.lon_stdev:.9f}',
            f'{msg.hgt_stdev:.9f}',
            f'{msg.diff_age:.6f}',
            f'{msg.sol_age:.6f}',
            msg.num_svs,
            msg.num_sol_svs,
            msg.num_sol_l1_svs,
            msg.num_sol_multi_svs,
            self._decode_station_id(msg.stn_id),
            msg.ext_sol_stat.status,
        ])
        self._bestpos_status_csv_file.flush()

        self._bestpos_status_row_count += 1
        if self._bestpos_status_row_count == 1:
            self.get_logger().info('Received first NovAtel BESTPOS status and wrote first CSV row.')
        elif self._bestpos_status_row_count % 100 == 0:
            self.get_logger().info(
                f'NovAtel BESTPOS status rows written: {self._bestpos_status_row_count}')

    @classmethod
    def _ecef_to_llh_deg(cls, x: float, y: float, z: float):
        lon = math.atan2(y, x)
        p = math.hypot(x, y)

        # Bowring's initial latitude estimate + fixed-point iteration.
        theta = math.atan2(z * cls._A, p * cls._B)
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        lat = math.atan2(
            z + cls._EP2 * cls._B * sin_theta ** 3,
            p - cls._E2 * cls._A * cos_theta ** 3,
        )

        for _ in range(5):
            sin_lat = math.sin(lat)
            n = cls._A / math.sqrt(1.0 - cls._E2 * sin_lat * sin_lat)
            lat = math.atan2(z + cls._E2 * n * sin_lat, p)

        sin_lat = math.sin(lat)
        n = cls._A / math.sqrt(1.0 - cls._E2 * sin_lat * sin_lat)
        alt = p / max(math.cos(lat), 1e-12) - n

        return math.degrees(lat), math.degrees(lon), alt

    @classmethod
    def _llh_to_ecef_deg(cls, lat_deg: float, lon_deg: float, alt: float):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        n = cls._A / math.sqrt(1.0 - cls._E2 * sin_lat * sin_lat)

        x = (n + alt) * cos_lat * math.cos(lon)
        y = (n + alt) * cos_lat * math.sin(lon)
        z = (n * (1.0 - cls._E2) + alt) * sin_lat
        return x, y, z

    @staticmethod
    def _decode_station_id(station_id) -> str:
        chars = []
        for value in station_id:
            if value == 0:
                continue
            chars.append(chr(value) if 32 <= value <= 126 else str(value))
        return ''.join(chars)

    @staticmethod
    def _quaternion_to_rpy_rad(x: float, y: float, z: float, w: float):
        # Standard ZYX Euler extraction from a normalized or near-normalized quaternion.
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _ecef_velocity_to_ned(vx: float, vy: float, vz: float, lat: float, lon: float):
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)

        east = -sin_lon * vx + cos_lon * vy
        north = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
        up = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz
        return north, east, -up

    def destroy_node(self) -> bool:
        if self._csv_file is not None and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        if self._rtk_csv_file is not None and not self._rtk_csv_file.closed:
            self._rtk_csv_file.flush()
            self._rtk_csv_file.close()
        if self._bestpos_status_csv_file is not None and not self._bestpos_status_csv_file.closed:
            self._bestpos_status_csv_file.flush()
            self._bestpos_status_csv_file.close()
        return super().destroy_node()

    def _periodic_status_log(self) -> None:
        self.get_logger().info(
            f'Periodic status: rows_written={self._row_count}, '
            f'novatel_rtk_fixed_rows_written={self._rtk_row_count}, '
            f'novatel_bestpos_status_rows_written={self._bestpos_status_row_count}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateCsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
