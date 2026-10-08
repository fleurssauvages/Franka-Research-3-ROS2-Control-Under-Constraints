import json
import math
import os


_ALLOWED_MESSAGE_PACKAGES = {
    'std_msgs',
    'geometry_msgs',
    'sensor_msgs',
    'nav_msgs',
    'builtin_interfaces',
    'trajectory_msgs',
    'shape_msgs',
    'visualization_msgs',
    'diagnostic_msgs',
}


def load_json_config(path):
    path = os.path.abspath(os.path.expanduser(path))
    with open(path, 'r', encoding='utf-8') as handle:
        root = json.load(handle)
    if not isinstance(root, dict):
        raise ValueError('top-level JSON value must be an object')
    if int(root.get('version', 1)) != 1:
        raise ValueError('only UDP bridge config version 1 is supported')
    endpoints = root.get('endpoints', [])
    if not isinstance(endpoints, list):
        raise ValueError("'endpoints' must be a list")
    return path, root


def require_message_type(type_name):
    if not isinstance(type_name, str):
        raise ValueError('ROS message type must be a string')
    parts = type_name.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(
            f"ROS message type '{type_name}' must use package/msg/Type syntax")
    if parts[0] not in _ALLOWED_MESSAGE_PACKAGES:
        raise ValueError(
            f"ROS message package '{parts[0]}' is not enabled; allowed packages: "
            + ', '.join(sorted(_ALLOWED_MESSAGE_PACKAGES)))
    return type_name


def require_port(value, name, allow_zero=False):
    value = int(value)
    low = 0 if allow_zero else 1
    if value < low or value > 65535:
        raise ValueError(f'{name} must be in [{low}, 65535]')
    return value


def require_positive_float(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and > 0')
    return value
