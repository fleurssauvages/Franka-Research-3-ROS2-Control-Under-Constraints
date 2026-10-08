from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from .config import require_message_type


def load_message_class(type_name):
    require_message_type(type_name)
    return get_message(type_name)


def qos_from_config(root, default_depth=10):
    root = root or {}
    depth = int(root.get('depth', default_depth))
    if depth < 1:
        raise ValueError('QoS depth must be >= 1')

    reliability_name = str(root.get('reliability', 'best_effort')).lower()
    reliability = {
        'best_effort': ReliabilityPolicy.BEST_EFFORT,
        'reliable': ReliabilityPolicy.RELIABLE,
    }.get(reliability_name)
    if reliability is None:
        raise ValueError("QoS reliability must be 'best_effort' or 'reliable'")

    durability_name = str(root.get('durability', 'volatile')).lower()
    durability = {
        'volatile': DurabilityPolicy.VOLATILE,
        'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
    }.get(durability_name)
    if durability is None:
        raise ValueError("QoS durability must be 'volatile' or 'transient_local'")

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=durability,
    )
