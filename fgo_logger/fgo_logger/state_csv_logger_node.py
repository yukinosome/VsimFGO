import csv
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from irt_nav_msgs.msg import FGOState


class StateCsvLoggerNode(Node):
    """Log online_fgo optimized state in ECEF and converted LLH into CSV."""

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

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_csv_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        self._append = self.get_parameter('append').get_parameter_value().bool_value

        self._row_count = 0
        self._csv_file = None
        self._csv_writer = None

        self._open_csv()

        self._sub = self.create_subscription(
            FGOState,
            self._input_topic,
            self._on_state,
            100,
        )

        self.get_logger().info(f'Subscribing: {self._input_topic}')
        self.get_logger().info(f'CSV output: {self._output_csv_path}')
        self.get_logger().info(f'Append mode: {self._append}')

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
            ])
            self._csv_file.flush()

    def _on_state(self, msg: FGOState) -> None:
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z

            lat_deg, lon_deg, alt = self._ecef_to_llh_deg(x, y, z)

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
            ])
            self._csv_file.flush()

            self._row_count += 1
            if self._row_count == 1:
                self.get_logger().info('Received first message and wrote first CSV row.')
            elif self._row_count % 100 == 0:
                self.get_logger().info(f'Rows written: {self._row_count}')
        except Exception as e:
            self.get_logger().error(f'Exception in _on_state: {e}', exc_info=True)

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

    def destroy_node(self) -> bool:
        if self._csv_file is not None and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        return super().destroy_node()

    def _periodic_status_log(self) -> None:
        self.get_logger().info(f'Periodic status: rows_written={self._row_count}')


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
