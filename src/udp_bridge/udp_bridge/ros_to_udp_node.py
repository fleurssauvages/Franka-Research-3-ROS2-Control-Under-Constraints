import socket

import rclpy
from rclpy.node import Node

from .codec import apply_scale_offset, cast_value, encode_binary, encode_json
from .config import load_json_config, require_port
from .field_access import get_path, set_json_path
from .ros_helpers import load_message_class, qos_from_config


class _WriterEndpoint:
    def __init__(self, node, cfg):
        self.node = node
        self.name = str(cfg.get('name', 'ros_to_udp'))
        self.cfg = cfg
        self.encoding = cfg.get('encoding', {})
        self.encoding_type = str(self.encoding.get('type', 'binary')).lower()
        if self.encoding_type not in ('binary', 'json'):
            raise ValueError(f"{self.name}: encoding.type must be 'binary' or 'json'")

        self.remote_ip = str(cfg.get('remote_ip', '')).strip()
        if not self.remote_ip:
            raise ValueError(f'{self.name}.remote_ip is required')
        self.remote_port = require_port(
            cfg.get('remote_port'), f'{self.name}.remote_port')
        local_ip = str(cfg.get('local_ip', '0.0.0.0'))
        local_port = require_port(
            cfg.get('local_port', 0), f'{self.name}.local_port', allow_zero=True)

        ros_cfg = cfg.get('ros', {})
        topic = str(ros_cfg.get('topic', '')).strip()
        type_name = str(ros_cfg.get('type', '')).strip()
        if not topic or not type_name:
            raise ValueError(f'{self.name}: ros.topic and ros.type are required')
        msg_cls = load_message_class(type_name)

        fields = self.encoding.get('fields', [])
        if not isinstance(fields, list) or not fields:
            raise ValueError(f'{self.name}: encoding.fields must be a non-empty list')
        self.fields = fields

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bool(cfg.get('broadcast', False)):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        ttl = int(cfg.get('ttl', 1))
        if ttl < 1 or ttl > 255:
            raise ValueError(f'{self.name}.ttl must be in [1, 255]')
        self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_TTL, ttl)
        self.socket.bind((local_ip, local_port))

        self.subscription = node.create_subscription(
            msg_cls,
            topic,
            self._callback,
            qos_from_config(ros_cfg.get('qos')),
        )
        self.node.get_logger().info(
            f"UDP writer '{self.name}': {topic} [{type_name}] -> "
            f"{self.remote_ip}:{self.remote_port} ({self.encoding_type})")

    def close(self):
        self.socket.close()

    def _field_value(self, msg, field):
        ros_field = str(field.get('ros_field', '')).strip()
        if not ros_field:
            raise ValueError("each writer field requires 'ros_field'")
        value = get_path(msg, ros_field)
        value = cast_value(value, field.get('value_type'))
        return apply_scale_offset(
            value, field.get('scale', 1.0), field.get('offset', 0.0))

    def _encode(self, msg):
        if self.encoding_type == 'binary':
            values = {}
            for field in self.fields:
                name = str(field.get('name', '')).strip()
                if not name:
                    raise ValueError("each binary writer field requires 'name'")
                values[name] = self._field_value(msg, field)
            return encode_binary(values, self.encoding)

        root = {}
        for field in self.fields:
            udp_field = str(field.get('udp_field', field.get('name', ''))).strip()
            if not udp_field:
                raise ValueError("each JSON writer field requires 'udp_field' or 'name'")
            set_json_path(root, udp_field, self._field_value(msg, field))
        return encode_json(root, self.encoding)

    def _callback(self, msg):
        try:
            payload = self._encode(msg)
            self.socket.sendto(payload, (self.remote_ip, self.remote_port))
        except Exception as exc:
            self.node.report_send_error(self.name, exc)


class RosToUdpNode(Node):
    def __init__(self):
        super().__init__('ros_to_udp')
        config_file = self.declare_parameter('config_file', '').value
        if not config_file:
            raise RuntimeError('config_file parameter is required')
        self.config_path, root = load_json_config(config_file)
        self.error_counts = {}
        self.endpoints = []
        for cfg in root.get('endpoints', []):
            if not bool(cfg.get('enabled', True)):
                continue
            self.endpoints.append(_WriterEndpoint(self, cfg))
        if not self.endpoints:
            self.get_logger().warning('UDP publisher config contains no enabled endpoints')
        self.get_logger().info(f"Loaded UDP publisher config '{self.config_path}'")

    def destroy_node(self):
        for endpoint in getattr(self, 'endpoints', []):
            endpoint.close()
        return super().destroy_node()

    def report_send_error(self, endpoint_name, exc):
        count = self.error_counts.get(endpoint_name, 0) + 1
        self.error_counts[endpoint_name] = count
        if count <= 5 or count % 100 == 0:
            self.get_logger().warning(
                f"UDP writer '{endpoint_name}' failed packet #{count}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RosToUdpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'ros_to_udp fatal: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
