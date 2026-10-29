#!/usr/bin/env python3
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

"""Draw a polygon field in meters and save its JSON and ROS occupancy map.

Run this module or file with --output DIRECTORY. Left click adds vertices, Enter
or right click closes a ring, H starts a hole, Backspace undoes, R resets, and S
saves. Drag any vertex of a closed ring to edit it. Existing output/field.json is
reopened automatically; --input selects another file. --example saves without a
graphical display, using a rectangle when no existing field is available.
"""

import argparse
import copy
import sys
from pathlib import Path

try:
    from .coverage_map import load_field, save_field_map, validate_field
except ImportError:
    from coverage_map import load_field, save_field_map, validate_field


class FieldEditor:
    """Small matplotlib polygon editor with validation on close and save."""

    def __init__(self, field, output):
        """Create an editor for an existing field or an empty drawing."""
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.plt = plt
        self.output = output
        self.frame_id = field['frame_id']
        self.resolution = field['resolution']
        self.rings = [copy.deepcopy(field['boundary'][:-1])] if field['boundary'] else [[]]
        self.rings.extend(copy.deepcopy(hole[:-1]) for hole in field['holes'])
        self.active = None if field['boundary'] else 0
        self.drag = None
        self.history = []
        self.message = 'Draw the boundary: left click vertices, Enter / right click to close.'
        self.figure, self.axes = plt.subplots(figsize=(11, 8))
        # Matplotlib's S/H/R shortcuts otherwise open its save dialog or reset the
        # view before the editor handles the same key. Keep toolbar mouse tools.
        default_keys = getattr(self.figure.canvas.manager, 'key_press_handler_id', None)
        if default_keys is not None:
            self.figure.canvas.mpl_disconnect(default_keys)
        self.figure.subplots_adjust(bottom=0.19)
        self.status = self.figure.text(0.07, 0.08, '', fontsize=10)
        self.figure.text(0.07, 0.035,
                         'Click: add | Drag: move vertex | Enter: close | H: hole | '
                         'Backspace: undo | R: reset | S: save', fontsize=9)
        self.buttons = []
        callbacks = [('Save [S]', self.save), ('Close [Enter]', self.close_ring),
                     ('Add hole [H]', self.add_hole), ('Undo', self.undo),
                     ('Reset [R]', self.reset)]
        for index, (label, callback) in enumerate(callbacks):
            button = Button(self.figure.add_axes([0.07 + index * 0.18, 0.12, 0.16, 0.04]), label)
            button.on_clicked(callback)
            self.buttons.append(button)
        self.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.figure.canvas.mpl_connect('key_press_event', self.on_key)
        self.redraw(fit=True)

    def snapshot(self):
        """Store the current rings for undo."""
        self.history.append((copy.deepcopy(self.rings), self.active))

    def field(self):
        """Return a validated field from the completed rings."""
        return validate_field({'frame_id': self.frame_id, 'resolution': self.resolution,
                               'boundary': self.rings[0], 'holes': self.rings[1:]})

    def redraw(self, fit=False):
        """Redraw rings and retain the zoom level while editing."""
        x_limits, y_limits = self.axes.get_xlim(), self.axes.get_ylim()
        self.axes.clear()
        self.axes.set(title='Coverage field editor (meters)', xlabel='x [m]', ylabel='y [m]')
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.grid(True, alpha=0.3)
        for index, ring in enumerate(self.rings):
            if not ring:
                continue
            points = ring + [ring[0]] if index != self.active else ring
            x, y = zip(*points)
            color = 'tab:blue' if index == 0 else 'tab:red'
            self.axes.plot(x, y, '.-' if index != self.active else '.--', color=color,
                           markersize=10, linewidth=1.5)
            if index != self.active:
                self.axes.fill(x, y, color='lightblue' if index == 0 else 'white', alpha=0.5)
            for vertex, (px, py) in enumerate(ring):
                self.axes.annotate(str(vertex + 1), (px, py), xytext=(5, 5),
                                   textcoords='offset points', fontsize=8)
        if fit:
            points = [point for ring in self.rings for point in ring]
            if points:
                x, y = zip(*points)
                padding = max(max(x) - min(x), max(y) - min(y), 5.0) * 0.15
                self.axes.set_xlim(min(x) - padding, max(x) + padding)
                self.axes.set_ylim(min(y) - padding, max(y) + padding)
            else:
                self.axes.set_xlim(-2, 22)
                self.axes.set_ylim(-2, 14)
        else:
            self.axes.set_xlim(x_limits)
            self.axes.set_ylim(y_limits)
        self.status.set_text(self.message)
        self.figure.canvas.draw_idle()

    def on_press(self, event):
        """Add a point, close a ring, or begin dragging a completed vertex."""
        if event.inaxes != self.axes:
            return
        if event.button == 3:
            self.close_ring()
            return
        if event.button != 1:
            return
        if self.active is not None:
            self.snapshot()
            self.rings[self.active].append([event.xdata, event.ydata])
            self.redraw()
            return
        closest, distance = None, 12.0
        for ring_index, ring in enumerate(self.rings):
            for vertex_index, point in enumerate(ring):
                px, py = self.axes.transData.transform(point)
                candidate = ((event.x - px) ** 2 + (event.y - py) ** 2) ** 0.5
                if candidate < distance:
                    closest, distance = (ring_index, vertex_index), candidate
        if closest is not None:
            self.snapshot()
            self.drag = closest

    def on_motion(self, event):
        """Move the dragged vertex in map coordinates."""
        if self.drag is not None and event.inaxes == self.axes:
            ring, vertex = self.drag
            self.rings[ring][vertex] = [event.xdata, event.ydata]
            self.message = 'Editing vertices. Press S to validate and save.'
            self.redraw()

    def on_release(self, event):
        """Finish dragging a vertex."""
        self.drag = None

    def on_key(self, event):
        """Handle the keyboard shortcuts shown below the plot."""
        commands = {'enter': self.close_ring, 'h': self.add_hole,
                    'backspace': self.undo, 'ctrl+z': self.undo, 'r': self.reset, 's': self.save}
        if event.key in commands:
            commands[event.key]()

    def close_ring(self, event=None):
        """Validate and complete the ring currently being drawn."""
        if self.active is None:
            return
        try:
            self.field()
        except ValueError as error:
            self.message = f'Cannot close: {error}'
        else:
            self.snapshot()
            self.active = None
            self.message = 'Ring closed. Drag vertices, add a hole [H], or save [S].'
        self.redraw()

    def add_hole(self, event=None):
        """Start drawing an obstacle inside the completed boundary."""
        if self.active is not None:
            self.message = 'Close the current ring before adding a hole.'
        else:
            self.snapshot()
            self.rings.append([])
            self.active = len(self.rings) - 1
            self.message = 'Draw a hole inside the boundary, then press Enter.'
        self.redraw()

    def undo(self, event=None):
        """Undo the last point, drag, close, new hole, or reset."""
        if self.history:
            self.rings, self.active = self.history.pop()
            self.message = 'Undone. Finish the ring with Enter, or save with S.'
            self.redraw()

    def reset(self, event=None):
        """Clear all rings and start a new boundary."""
        self.snapshot()
        self.rings = [[]]
        self.active = 0
        self.message = 'Draw a new boundary. Undo restores the previous map.'
        self.redraw()

    def save(self, event=None):
        """Validate and atomically replace the three map files."""
        if self.active is not None:
            self.close_ring()
            if self.active is not None:
                return
        try:
            paths = save_field_map(self.field(), self.output)
        except (ValueError, OSError) as error:
            self.message = f'Cannot save: {error}'
        else:
            self.message = f'Saved: {paths["field"]}'
            print(f'Saved field: {paths["field"]}\nSaved ROS map: {paths["map"]}', flush=True)
        self.redraw()

    def show(self):
        """Run the graphical editor."""
        self.plt.show()


def main(argv=None):
    """Run the GUI, or generate an example map without matplotlib."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', default='coverage_map',
                        help='Directory for the three map files')
    parser.add_argument('--input', help='Existing field.json to edit')
    parser.add_argument('--resolution', type=float,
                        help='Occupancy map resolution in meters per cell')
    parser.add_argument('--frame-id', help='ROS coordinate frame (default: map)')
    parser.add_argument('--example', action='store_true',
                        help='Save without a GUI; create a 20 x 12 m example if no field exists')
    args = parser.parse_args(argv)
    try:
        existing = Path(args.output).expanduser() / 'field.json'
        source = args.input or (str(existing) if existing.is_file() else None)
        if source:
            field = load_field(source)
        else:
            field = {'frame_id': 'map', 'resolution': 0.1, 'boundary': [], 'holes': []}
            if args.example:
                field['boundary'] = [[0, 0], [20, 0], [20, 12], [0, 12], [0, 0]]
        if args.resolution is not None:
            field['resolution'] = args.resolution
        if args.frame_id is not None:
            field['frame_id'] = args.frame_id
        # Check metadata before opening an empty editor as well.
        probe = dict(field, boundary=field['boundary'] or [[0, 0], [1, 0], [0, 1]])
        validate_field(probe)
        if args.example:
            paths = save_field_map(field, args.output)
            print(f'Saved field: {paths["field"]}\nSaved ROS map: {paths["map"]}')
        else:
            FieldEditor(field, args.output).show()
    except (OSError, ValueError, ImportError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
