"""Read-only validation of the YAML/PGM pair produced by map_saver_cli."""

import math
from pathlib import Path

import yaml


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{label} must be a number')
    if not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return value


def _validate_pgm(path):
    data = path.read_bytes()
    offset = 0

    def token():
        nonlocal offset
        while offset < len(data):
            if data[offset:offset + 1] == b'#':
                end = data.find(b'\n', offset)
                offset = len(data) if end < 0 else end + 1
            elif data[offset:offset + 1] in b' \t\r\n\v\f':
                offset += 1
            else:
                break
        start = offset
        while offset < len(data) and data[offset:offset + 1] not in b' \t\r\n\v\f#':
            offset += 1
        if start == offset:
            raise ValueError('PGM header/pixels are incomplete')
        return data[start:offset]

    magic = token()
    if magic not in (b'P5', b'P2'):
        raise ValueError('Expected a P5/P2 PGM map')
    width, height, maximum = (int(token()) for _ in range(3))
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError('Expected positive map dimensions and 8-bit PGM maximum 255')
    count = width * height
    if magic == b'P5':
        # Consume only the header separator: pixel values may themselves be whitespace.
        if data[offset:offset + 2] == b'\r\n':
            offset += 2
        elif data[offset:offset + 1] in (b' ', b'\t', b'\r', b'\n', b'\v', b'\f'):
            offset += 1
        else:
            raise ValueError('PGM binary data separator is missing')
        if len(data) - offset != count:
            raise ValueError('PGM pixel count does not match its dimensions')
    else:
        for _ in range(count):
            if not 0 <= int(token()) <= maximum:
                raise ValueError('PGM pixel is out of range')
        # The remainder may contain only whitespace and comments.
        if any(line.split(b'#', 1)[0].strip() for line in data[offset:].splitlines()):
            raise ValueError('PGM has extra pixels')
    return width, height


def validate_saved_map(map_file):
    """Return an absolute YAML path, or fail before any launch Node is created.

    File validity is NOT map quality, localization, or navigation acceptance.
    This entry intentionally accepts only map_saver_cli's 8-bit PGM output.
    """
    path = Path(map_file).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValueError('map_file must name a YAML file')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('Map YAML must contain a mapping')
    image = document.get('image')
    if not isinstance(image, str) or not image.strip():
        raise ValueError('Map YAML requires an image filename')
    image_path = (path.parent / image).resolve(strict=True)
    if not image_path.is_file() or image_path.suffix.lower() != '.pgm':
        raise ValueError('This entry requires the PGM produced by map_saver_cli')
    if _number(document.get('resolution'), 'resolution') <= 0:
        raise ValueError('resolution must be positive')
    origin = document.get('origin')
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError('origin must be [x, y, yaw]')
    for value in origin:
        _number(value, 'origin')
    free = _number(document.get('free_thresh'), 'free_thresh')
    occupied = _number(document.get('occupied_thresh'), 'occupied_thresh')
    if not 0 <= free < occupied <= 1:
        raise ValueError('Expected 0 <= free_thresh < occupied_thresh <= 1')
    if type(document.get('negate')) is not int or document['negate'] not in (0, 1):
        raise ValueError('negate must be 0 or 1')
    if document.get('mode', 'trinary') not in ('trinary', 'scale', 'raw'):
        raise ValueError('Unsupported map mode')
    _validate_pgm(image_path)
    return str(path)
