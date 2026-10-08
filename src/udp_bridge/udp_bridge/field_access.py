import re


_TOKEN_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)(?:\[([0-9]+)\])?$')


def _tokens(path):
    if not isinstance(path, str) or not path.strip():
        raise ValueError('field path must be a non-empty string')
    result = []
    for part in path.split('.'):
        match = _TOKEN_RE.fullmatch(part)
        if not match:
            raise ValueError(f"invalid field path component '{part}' in '{path}'")
        result.append((match.group(1), None if match.group(2) is None else int(match.group(2))))
    return result


def _get_member(obj, name):
    if isinstance(obj, dict):
        if name not in obj:
            raise KeyError(name)
        return obj[name]
    return getattr(obj, name)


def _set_member(obj, name, value):
    if isinstance(obj, dict):
        obj[name] = value
    else:
        setattr(obj, name, value)


def get_path(obj, path):
    current = obj
    for name, index in _tokens(path):
        current = _get_member(current, name)
        if index is not None:
            current = current[index]
    return current


def set_path(obj, path, value):
    tokens = _tokens(path)
    current = obj
    for name, index in tokens[:-1]:
        current = _get_member(current, name)
        if index is not None:
            current = current[index]

    name, index = tokens[-1]
    if index is None:
        _set_member(current, name, value)
        return

    sequence = _get_member(current, name)
    if index >= len(sequence):
        if isinstance(sequence, list):
            sequence.extend([0] * (index + 1 - len(sequence)))
        else:
            raise IndexError(
                f"cannot grow non-list sequence '{name}' to index {index}")
    sequence[index] = value


def set_json_path(root, path, value):
    tokens = _tokens(path)
    current = root
    for name, index in tokens[:-1]:
        if name not in current:
            current[name] = [] if index is not None else {}
        current = current[name]
        if index is not None:
            while len(current) <= index:
                current.append({})
            current = current[index]

    name, index = tokens[-1]
    if index is None:
        current[name] = value
        return
    if name not in current:
        current[name] = []
    sequence = current[name]
    while len(sequence) <= index:
        sequence.append(None)
    sequence[index] = value
