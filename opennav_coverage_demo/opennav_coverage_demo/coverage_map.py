# Copyright 2026 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate editable field polygons and export ROS occupancy maps without ROS."""

import json
import math
import os
import tempfile
from pathlib import Path


_EPSILON = 1e-9
_MAX_MAP_PIXELS = 16000000


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(point, a, b):
    return (abs(_cross(a, b, point)) <= _EPSILON and
            min(a[0], b[0]) - _EPSILON <= point[0] <= max(a[0], b[0]) + _EPSILON and
            min(a[1], b[1]) - _EPSILON <= point[1] <= max(a[1], b[1]) + _EPSILON)


def _segments_intersect(a, b, c, d):
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if ((ab_c > _EPSILON and ab_d < -_EPSILON or
         ab_c < -_EPSILON and ab_d > _EPSILON) and
            (cd_a > _EPSILON and cd_b < -_EPSILON or
             cd_a < -_EPSILON and cd_b > _EPSILON)):
        return True
    return (_on_segment(c, a, b) or _on_segment(d, a, b) or
            _on_segment(a, c, d) or _on_segment(b, c, d))


def _strictly_inside(point, ring):
    inside = False
    for a, b in zip(ring, ring[1:]):
        if _on_segment(point, a, b):
            return False
        if (a[1] > point[1]) != (b[1] > point[1]):
            intersection = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if point[0] < intersection:
                inside = not inside
    return inside


def _rings_intersect(first, second):
    return any(_segments_intersect(a, b, c, d)
               for a, b in zip(first, first[1:])
               for c, d in zip(second, second[1:]))


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    try:
        number = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f'{label} must be a finite number') from error
    if not math.isfinite(number):
        raise ValueError(f'{label} must be a finite number')
    return number


def _normalize_ring(points, label, clockwise=False):
    if not isinstance(points, (list, tuple)):
        raise ValueError(f'{label} must be a list of [x, y] vertices')
    ring = []
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f'{label} vertex {index + 1} must contain exactly x and y')
        ring.append([_number(point[0], f'{label} x'), _number(point[1], f'{label} y')])
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring.pop()
    if len(ring) < 3:
        raise ValueError(f'{label} needs at least three different vertices')
    if len({tuple(point) for point in ring}) != len(ring):
        raise ValueError(f'{label} contains repeated vertices')
    ring.append(ring[0][:])
    count = len(ring) - 1
    for first in range(count):
        for second in range(first + 1, count):
            if second == first + 1 or first == 0 and second == count - 1:
                # Adjacent edges may meet only at their shared vertex.
                if second == first + 1:
                    a, b, c = ring[first], ring[second], ring[second + 1]
                else:
                    a, b, c = ring[count - 1], ring[0], ring[1]
                if _on_segment(c, a, b) or _on_segment(a, b, c):
                    raise ValueError(f'{label} has overlapping adjacent edges')
                continue
            if _segments_intersect(ring[first], ring[first + 1],
                                   ring[second], ring[second + 1]):
                raise ValueError(f'{label} has a self-intersection')
    # Translate around the first vertex to avoid cancellation far from the origin.
    origin = ring[0]
    area = sum(_cross(origin, a, b) for a, b in zip(ring, ring[1:])) / 2.0
    if not math.isfinite(area) or abs(area) <= _EPSILON:
        raise ValueError(f'{label} must enclose a nonzero finite area')
    if (area < 0.0) != clockwise:
        ring = [ring[0]] + list(reversed(ring[1:-1])) + [ring[0][:]]
    return ring


def validate_field(field):
    """Return a normalized field, with closed CCW boundary and closed CW holes.

    Coordinates are in meters. Rings must be simple; holes must be strictly inside
    the boundary, disjoint, and must not touch any other ring. Only the four public
    fields (frame_id, resolution, boundary, holes) are returned.
    """
    if not isinstance(field, dict):
        raise ValueError('Field JSON must be an object')
    frame_id = field.get('frame_id', 'map')
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError('frame_id must be a nonempty string')
    resolution = _number(field.get('resolution', 0.1), 'resolution')
    if resolution <= 0.0:
        raise ValueError('resolution must be greater than zero')
    boundary = _normalize_ring(field.get('boundary'), 'Boundary')
    raw_holes = field.get('holes', [])
    if not isinstance(raw_holes, (list, tuple)):
        raise ValueError('holes must be a list of polygon rings')
    holes = [_normalize_ring(points, f'Hole {index + 1}', clockwise=True)
             for index, points in enumerate(raw_holes)]
    for index, hole in enumerate(holes):
        if _rings_intersect(boundary, hole) or not _strictly_inside(hole[0], boundary):
            raise ValueError(
                f'Hole {index + 1} must be strictly inside the boundary without touching')
        for other_index, other in enumerate(holes[:index]):
            if (_rings_intersect(hole, other) or _strictly_inside(hole[0], other) or
                    _strictly_inside(other[0], hole)):
                raise ValueError(f'Holes {other_index + 1} and {index + 1} overlap or touch')
    return {'frame_id': frame_id, 'resolution': resolution,
            'boundary': boundary, 'holes': holes}


def load_field(path):
    """Load and validate a field.json file; return the normalized field dict."""
    path = Path(path).expanduser()
    try:
        with path.open(encoding='utf-8') as stream:
            field = json.load(stream)
    except json.JSONDecodeError as error:
        raise ValueError(f'Invalid JSON in {path}: {error}') from error
    return validate_field(field)


def _scanline_spans(ring, y, origin_x, resolution, width):
    crossings = []
    for a, b in zip(ring, ring[1:]):
        if (a[1] > y) != (b[1] > y):
            crossings.append(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
    crossings.sort()
    for left, right in zip(crossings[::2], crossings[1::2]):
        # Rasterize cell centers, with left-inclusive/right-exclusive intervals.
        start = max(0, min(width, math.ceil((left - origin_x) / resolution - 0.5)))
        stop = max(0, min(width, math.ceil((right - origin_x) / resolution - 0.5)))
        yield start, stop


def _atomic_write(path, data):
    descriptor, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            os.fchmod(stream.fileno(), 0o644)
            stream.write(data)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def save_field_map(field, output_directory):
    """Write field.json, map.pgm, and Nav2 map.yaml; return their absolute paths.

    Return keys are ``field`` (JSON), ``map`` (YAML), and ``image`` (PGM). A one
    meter occupied border surrounds the field; holes and the exterior are occupied.
    Individual files are replaced atomically, with field.json committed last so a
    watcher sees the new field only once both map files have been written.
    """
    field = validate_field(field)
    resolution = field['resolution']
    boundary = field['boundary']
    origin_column = (min(p[0] for p in boundary) - 1.0) / resolution
    origin_row = (min(p[1] for p in boundary) - 1.0) / resolution
    if not math.isfinite(origin_column) or not math.isfinite(origin_row):
        raise ValueError('Map dimensions are too large; increase resolution')
    origin_x = math.floor(origin_column) * resolution
    origin_y = math.floor(origin_row) * resolution
    extent_x = (max(p[0] for p in boundary) + 1.0 - origin_x) / resolution
    extent_y = (max(p[1] for p in boundary) + 1.0 - origin_y) / resolution
    if not math.isfinite(extent_x) or not math.isfinite(extent_y):
        raise ValueError(
            'Map dimensions are too large; increase resolution or reduce coordinates')
    width, height = math.ceil(extent_x), math.ceil(extent_y)
    if width < 1 or height < 1 or width * height > _MAX_MAP_PIXELS:
        raise ValueError(
            f'Map exceeds {_MAX_MAP_PIXELS} cells; increase resolution or reduce its size')
    pixels = bytearray(width * height)
    for row in range(height):
        # PGM starts at the top left; ROS map origin is at the bottom left.
        y = origin_y + (height - row - 0.5) * resolution
        offset = row * width
        for start, stop in _scanline_spans(boundary, y, origin_x, resolution, width):
            pixels[offset + start:offset + stop] = b'\xfe' * (stop - start)
        for hole in field['holes']:
            for start, stop in _scanline_spans(hole, y, origin_x, resolution, width):
                pixels[offset + start:offset + stop] = b'\x00' * (stop - start)
    directory = Path(output_directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    paths = {'field': directory / 'field.json', 'map': directory / 'map.yaml',
             'image': directory / 'map.pgm'}
    header = f'P5\n# Coverage field occupancy map\n{width} {height}\n255\n'.encode('ascii')
    yaml = ('image: map.pgm\n'
            'mode: trinary\n'
            f'resolution: {resolution:.17g}\n'
            f'origin: [{origin_x:.17g}, {origin_y:.17g}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n')
    _atomic_write(paths['image'], header + pixels)
    _atomic_write(paths['map'], yaml.encode('utf-8'))
    _atomic_write(paths['field'], (json.dumps(field, indent=2, allow_nan=False) + '\n').encode())
    return {name: str(path) for name, path in paths.items()}
