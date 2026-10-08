import socket

import rclpy
from rclpy.node import Node

from .codec import apply_scale_offset, cast_value, decode_binary, decode_json
from .config import load_json_config, require_port, require_positive_float
from .field_access import get_path, set_path
from .ros_helpers import load_message_class, qos_from_config


class _ReaderEndpoint:
    def __init__(self, node, cfg):
        self.node = node
        self.name = str(cfg.get('name', 'udp_reader'))
        self.cfg = cfg
        self.encoding = cfg.get('encoding', {})
        self.encoding_type = str(self.encoding.get('type', 'binary')).lower()
        if self.encoding_type not in ('binary', 'json'):
            raise ValueError(f"{self.name}: encoding.type must be 'binary' or 'json'")

        bind_ip = str(cfg.get('bind_ip', '0.0.0.0'))
        bind_port = require_port(cfg.get('bind_port'), f'{self.name}.bind_port')
        self.source_ip = str(cfg.get('source_ip', '')).strip()
        self.source_port = require_port(
            cfg.get('source_port', 0), f'{self.name}.source_port', allow_zero=True)
        self.max_datagram_bytes = int(cfg.get('max_datagram_bytes', 65535))
        if self.max_datagram_bytes < 1 or self.max_datagram_bytes > 65535:
            raise ValueError(f'{self.name}.max_datagram_bytes must be in [1, 65535]')

        ros_cfg = cfg.get('ros', {})
        topic = str(ros_cfg.get('topic', '')).strip()
        type_name = str(ros_cfg.get('type', '')).strip()
        if not topic or not type_name:
            raise ValueError(f'{self.name}: ros.topic and ros.type are required')
        self.msg_cls = load_message_class(type_name)
        self.publisher = node.create_publisher(
            self.msg_cls, topic, qos_from_config(ros_cfg.get('qos')))
        self.defaults = ros_cfg.get('defaults', {})
        if not isinstance(self.defaults, dict):
            raise ValueError(f'{self.name}: ros.defaults must be an object')
        self.stamp_now = ros_cfg.get('stamp_now', [])
        if not isinstance(self.stamp_now, list):
            raise ValueError(f'{self.name}: ros.stamp_now must be a list')

        fields = self.encoding.get('fields', [])
        if not isinstance(fields, list) or not fields:
            raise ValueError(f'{self.name}: encoding.fields must be a non-empty list')
        self.fields = fields

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receive_buffer = int(cfg.get('receive_buffer_bytes', 1 << 20))
        if receive_buffer > 0:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer)
        self.socket.bind((bind_ip, bind_port))
        self.socket.setblocking(False)
        self.node.get_logger().info(
            f"UDP reader '{self.name}': {bind_ip}:{bind_port} -> {topic} [{type_name}] "
            f"({self.encoding_type})")

    def close(self):
        self.socket.close()

    def _source_allowed(self, address):
        ip, port = address[0], address[1]
        if self.source_ip and ip != self.source_ip:
            return False
        if self.source_port and port != self.source_port:
            return False
        return True

    def _decode(self, payload):
        if self.encoding_type == 'binary':
            return decode_binary(payload, self.encoding)
        return decode_json(payload, self.encoding)

    def _raw_field(self, decoded, field):
        if self.encoding_type == 'binary':
            name = str(field.get('name', '')).strip()
            if not name:
                raise ValueError("binary field requires 'name'")
            return decoded[name]
        udp_field = str(field.get('udp_field', field.get('name', ''))).strip()
        if not udp_field:
            raise ValueError("JSON field requires 'udp_field' or 'name'")
        return get_path(decoded, udp_field)

    def handle_datagram(self, payload, address):
        if not self._source_allowed(address):
            return
        decoded = self._decode(payload)
        msg = self.msg_cls()
        for ros_field, default_value in self.defaults.items():
            set_path(msg, ros_field, default_value)
        for field in self.fields:
            ros_field = str(field.get('ros_field', '')).strip()
            if not ros_field:
                continue
            value = self._raw_field(decoded, field)
            value = cast_value(value, field.get('value_type'))
            value = apply_scale_offset(
                value, field.get('scale', 1.0), field.get('offset', 0.0))
            set_path(msg, ros_field, value)
        for ros_field in self.stamp_now:
            set_path(msg, str(ros_field), self.node.get_clock().now().to_msg())
        self.publisher.publish(msg)


class UdpToRosNode(Node):
    def __init__(self):
        super().__init__('udp_to_ros')
        config_file = self.declare_parameter('config_file', '').value
        if not config_file:
            raise RuntimeError('config_file parameter is required')
        self.config_path, root = load_json_config(config_file)
        self.endpoints = []
        for cfg in root.get('endpoints', []):
            if not bool(cfg.get('enabled', True)):
                continue
            self.endpoints.append(_ReaderEndpoint(self, cfg))
        if not self.endpoints:
            self.get_logger().warning('UDP reader config contains no enabled endpoints')
        period_ms = require_positive_float(
            root.get('poll_period_ms', 2.0), 'poll_period_ms')
        self.max_datagrams_per_tick = int(root.get('max_datagrams_per_tick', 100))
        if self.max_datagrams_per_tick < 1:
            raise ValueError('max_datagrams_per_tick must be >= 1')
        self.timer = self.create_timer(period_ms / 1000.0, self._poll)
        self.error_counts = {}
        self.get_logger().info(f"Loaded UDP reader config '{self.config_path}'")

    def destroy_node(self):
        for endpoint in getattr(self, 'endpoints', []):
            endpoint.close()
        return super().destroy_node()

    def _warn_decode(self, endpoint, exc):
        count = self.error_counts.get(endpoint.name, 0) + 1
        self.error_counts[endpoint.name] = count
        if count <= 5 or count % 100 == 0:
            self.get_logger().warning(
                f"UDP reader '{endpoint.name}' dropped datagram #{count}: {exc}")

    def _poll(self):
        for endpoint in self.endpoints:
            for _ in range(self.max_datagrams_per_tick):
                try:
                    payload, address = endpoint.socket.recvfrom(endpoint.max_datagram_bytes)
                except BlockingIOError:
                    break
                except OSError as exc:
                    self._warn_decode(endpoint, exc)
                    break
                try:
                    endpoint.handle_datagram(payload, address)
                except Exception as exc:
                    self._warn_decode(endpoint, exc)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UdpToRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'udp_to_ros fatal: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
