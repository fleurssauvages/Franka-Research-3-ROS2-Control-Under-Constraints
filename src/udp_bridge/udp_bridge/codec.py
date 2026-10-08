import json
import math
import struct
from collections.abc import Sequence


_STRUCT_TYPES = {
    'int8': 'b',
    'uint8': 'B',
    'int16': 'h',
    'uint16': 'H',
    'int32': 'i',
    'uint32': 'I',
    'int64': 'q',
    'uint64': 'Q',
    'float32': 'f',
    'float64': 'd',
    'bool': '?',
}

_ENDIAN = {
    'little': '<',
    'big': '>',
    'network': '!',
}


def _field_count(field):
    count = int(field.get('count', 1))
    if count < 1:
        raise ValueError("field 'count' must be >= 1")
    return count


def binary_layout(spec):
    endian_name = str(spec.get('endianness', 'little')).lower()
    if endian_name not in _ENDIAN:
        raise ValueError("endianness must be 'little', 'big', or 'network'")
    fields = spec.get('fields', [])
    if not isinstance(fields, list) or not fields:
        raise ValueError("binary encoding requires a non-empty 'fields' list")

    fmt = [_ENDIAN[endian_name]]
    value_fields = []
    for index, field in enumerate(fields):
        if not isinstance(field, dict):
            raise ValueError(f'fields[{index}] must be an object')
        field_type = str(field.get('type', '')).lower()
        count = _field_count(field)
        if field_type == 'padding':
            fmt.append(f'{count}x')
            continue
        if field_type not in _STRUCT_TYPES:
            raise ValueError(
                f"unsupported binary field type '{field_type}' at index {index}")
        fmt.append(f'{count}{_STRUCT_TYPES[field_type]}')
        value_fields.append((field, count))
    compiled = struct.Struct(''.join(fmt))
    return compiled, value_fields


def decode_binary(payload, spec):
    compiled, value_fields = binary_layout(spec)
    strict_size = bool(spec.get('strict_size', True))
    if strict_size and len(payload) != compiled.size:
        raise ValueError(
            f'UDP payload is {len(payload)} bytes but binary format requires {compiled.size}')
    if len(payload) < compiled.size:
        raise ValueError(
            f'UDP payload is {len(payload)} bytes but binary format requires at least {compiled.size}')
    values = compiled.unpack_from(payload)
    result = {}
    offset = 0
    for field, count in value_fields:
        name = str(field.get('name', '')).strip()
        if not name:
            raise ValueError("each non-padding binary field requires a 'name'")
        if count == 1:
            result[name] = values[offset]
        else:
            result[name] = list(values[offset:offset + count])
        offset += count
    return result


def encode_binary(values_by_name, spec):
    compiled, value_fields = binary_layout(spec)
    packed_values = []
    for field, count in value_fields:
        name = str(field.get('name', '')).strip()
        if name not in values_by_name:
            raise ValueError(f"missing binary field '{name}'")
        value = values_by_name[name]
        if count == 1:
            packed_values.append(value)
        else:
            if not isinstance(value, (list, tuple)) or len(value) != count:
                raise ValueError(
                    f"binary field '{name}' requires exactly {count} values")
            packed_values.extend(value)
    return compiled.pack(*packed_values)


def decode_json(payload, spec):
    encoding = str(spec.get('text_encoding', 'utf-8'))
    obj = json.loads(payload.decode(encoding))
    if not isinstance(obj, (dict, list)):
        raise ValueError('JSON UDP payload must decode to an object or array')
    return obj


def encode_json(obj, spec):
    encoding = str(spec.get('text_encoding', 'utf-8'))
    compact = bool(spec.get('compact', True))
    separators = (',', ':') if compact else None
    return json.dumps(obj, separators=separators, allow_nan=False).encode(encoding)


def apply_scale_offset(value, scale=1.0, offset=0.0):
    scale = float(scale)
    offset = float(offset)
    if not math.isfinite(scale) or not math.isfinite(offset):
        raise ValueError('scale and offset must be finite')
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [apply_scale_offset(v, scale, offset) for v in value]
    if isinstance(value, bool):
        if scale != 1.0 or offset != 0.0:
            raise ValueError('scale/offset cannot be applied to bool values')
        return value
    return value * scale + offset


def cast_value(value, value_type):
    if not value_type:
        return value
    value_type = str(value_type).lower()
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [cast_value(v, value_type) for v in value]
    if value_type in ('float32', 'float64', 'float'):
        return float(value)
    if value_type in ('int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'int'):
        return int(value)
    if value_type in ('bool', 'boolean'):
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    if value_type in ('string', 'str'):
        return str(value)
    raise ValueError(f"unsupported value_type '{value_type}'")
