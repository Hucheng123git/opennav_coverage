#!/usr/bin/env python3
"""Export real ComputeCoveragePath results; run inside the Humble Docker shell.

Start the coverage demo first, source scripts/coverage_demo_env.sh, then run
python3 scripts/export_coverage_results.py. No planning algorithm is reimplemented.
"""

import argparse
import math
import os
from pathlib import Path
import tempfile


def request_plan(node, client, goal, timeout):
    """Wait for one action result, cancelling an accepted goal on timeout."""
    import rclpy
    from action_msgs.msg import GoalStatus

    accepted = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, accepted, timeout_sec=timeout)
    if not accepted.done():
        # A slow server may still accept the pending goal after our deadline.
        accepted.add_done_callback(
            lambda future: future.result().cancel_goal_async()
            if future.result() is not None and future.result().accepted else None)
        rclpy.spin_once(node, timeout_sec=0.1)
        raise RuntimeError('Timed out waiting for action goal acceptance')
    handle = accepted.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('Coverage server rejected the goal')
    completed = handle.get_result_async()
    rclpy.spin_until_future_complete(node, completed, timeout_sec=timeout)
    if not completed.done():
        canceled = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, canceled, timeout_sec=2.0)
        raise RuntimeError('Timed out waiting for the coverage path; cancellation requested')
    wrapped = completed.result()
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code != 0:
        raise RuntimeError(
            f'Planning failed: status={wrapped.status}, error={wrapped.result.error_code}')
    result = wrapped.result
    if not result.nav_path.poses or not result.coverage_path.swaths_ordered:
        raise RuntimeError('Planner returned an empty or unordered coverage path')
    for pose in result.nav_path.poses:
        if not all(math.isfinite(value) for value in
                   (pose.pose.position.x, pose.pose.position.y)):
            raise RuntimeError('Planner returned non-finite coordinates')
    return result


def path_length(path):
    points = [pose.pose.position for pose in path.poses]
    return sum(math.hypot(end.x - start.x, end.y - start.y)
               for start, end in zip(points, points[1:]))


def draw_result(axis, entry, field, background, label_size=9):
    from matplotlib.patches import FancyArrowPatch, Polygon
    from matplotlib.transforms import Affine2D
    from opennav_coverage_demo.coverage_visualization import make_plan_markers

    result = entry['result']
    if background is not None:
        image, resolution, origin = background
        height, width = image.shape[:2]
        transform = (Affine2D().rotate(origin[2]).translate(*origin[:2]) + axis.transData)
        axis.imshow(image, cmap='gray', vmin=0, vmax=255, origin='upper',
                    extent=(0, width * resolution, 0, height * resolution),
                    transform=transform, zorder=0)
    axis.add_patch(Polygon(field['boundary'], fill=False, edgecolor='#00a7c9', lw=1.7))
    for hole in field['holes']:
        axis.add_patch(Polygon(hole, facecolor='#444444', edgecolor='#c03e20', lw=1.2))
    markers = make_plan_markers(
        result.nav_path, result.coverage_path, entry['mode'], entry['spiral_n'],
        result.nav_path.header.stamp, arrow_spacing=2.0,
        contour_spacing=entry.get('contour_spacing', 0.0))
    for marker in markers.markers:
        points = [(point.x, point.y) for point in marker.points]
        if marker.ns == 'coverage_work':
            for start, end in zip(points[::2], points[1::2]):
                axis.plot([start[0], end[0]], [start[1], end[1]],
                          color='#2073ed', lw=1.5, zorder=2)
        elif marker.ns == 'coverage_turns':
            axis.plot(*zip(*points), color='#f17316', lw=1.4, zorder=3)
        elif marker.ns == 'coverage_direction':
            axis.add_patch(FancyArrowPatch(
                points[0], points[1], arrowstyle='-|>', mutation_scale=9,
                facecolor='#ffe346', edgecolor='#675623', lw=0.65, zorder=4))
        elif marker.ns == 'coverage_swath_numbers':
            axis.text(marker.pose.position.x, marker.pose.position.y, marker.text,
                      ha='center', va='center', fontsize=label_size, color='#10233e',
                      bbox=dict(facecolor='white', edgecolor='none', alpha=0.8, pad=0.4),
                      zorder=5)
    points = [pose.pose.position for pose in result.nav_path.poses]
    for label, point, color, offset in (
            ('START', points[0], '#15963a', (0, -19)),
            ('END', points[-1], '#d8208b', (0, 12))):
        axis.scatter([point.x], [point.y], s=35, color=color, edgecolors='white', zorder=6)
        axis.annotate(label, (point.x, point.y), xytext=offset, textcoords='offset points',
                      ha='center', fontsize=label_size, fontweight='bold', color=color,
                      bbox=dict(facecolor='white', edgecolor='none', alpha=0.85, pad=1),
                      zorder=7)
    bounds = field['boundary'] + [(point.x, point.y) for point in points]
    axis.set_xlim(min(x for x, _ in bounds) - 0.8, max(x for x, _ in bounds) + 0.8)
    axis.set_ylim(min(y for _, y in bounds) - 1.0, max(y for _, y in bounds) + 1.0)
    axis.set_aspect('equal', adjustable='box')
    axis.set_xlabel('x (m)')
    axis.set_ylabel('y (m)')
    axis.grid(alpha=0.18)
    axis.set_axisbelow(True)
    unit = 'contour edges' if entry['mode'] == 'CONCENTRIC' else 'swaths'
    axis.set_title(
        f"{entry['title']}\n{len(result.coverage_path.swaths)} {unit} | "
        f"{len(result.nav_path.poses)} poses | {path_length(result.nav_path):.2f} m",
        fontsize=label_size + 2, pad=9)


def save_images(entries, field, map_directory, output):
    # Keep matplotlib's font cache temporary, outside the source tree.
    with tempfile.TemporaryDirectory(prefix='coverage_plot_') as cache:
        os.environ['MPLCONFIGDIR'] = cache
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
        import numpy as np
        from PIL import Image
        import yaml

        map_yaml = map_directory / 'map.yaml'
        metadata = yaml.safe_load(map_yaml.read_text())
        image_path = Path(metadata['image'])
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        with Image.open(image_path) as image:
            background = (np.asarray(image.convert('L')), metadata['resolution'],
                          metadata['origin'])
        legend = [
            Line2D([], [], color='#2073ed', label='Work swath'),
            Line2D([], [], color='#f17316', label='Turn / connection'),
            Line2D([], [], marker='>', color='#b89913', ls='none', label='Travel direction'),
            Line2D([], [], marker='o', color='#15963a', ls='none', label='START'),
            Line2D([], [], marker='o', color='#d8208b', ls='none', label='END'),
        ]
        output.mkdir(parents=True, exist_ok=True)
        for entry in entries:
            figure, axis = plt.subplots(figsize=(14, 9))
            draw_result(axis, entry, field, background)
            figure.legend(handles=legend, loc='lower center', ncol=5, frameon=False)
            figure.text(0.5, 0.045, 'Labels = visit order (C = contour); actual planner output',
                        ha='center', fontsize=10)
            figure.tight_layout(rect=(0, 0.075, 1, 1))
            destination = output / f"{entry['name']}.png"
            figure.savefig(destination, dpi=180, facecolor='white')
            plt.close(figure)
            print(f'Saved {destination}', flush=True)
        if len(entries) < 2:
            return
        rows = math.ceil(len(entries) / 2)
        figure, axes = plt.subplots(rows, 2, figsize=(18, 6 * rows), squeeze=False)
        for axis, entry in zip(axes.flat, entries):
            draw_result(axis, entry, field, background, label_size=7)
        for axis in list(axes.flat)[len(entries):]:
            axis.set_visible(False)
        figure.suptitle(
            'Coverage modes — parallel swaths and inward boundary contours', fontsize=17)
        figure.legend(handles=legend, loc='lower center', ncol=5, frameon=False)
        figure.tight_layout(rect=(0, 0.035, 1, 0.97))
        destination = output / 'coverage_comparison.png'
        figure.savefig(destination, dpi=160, facecolor='white')
        plt.close(figure)
        print(f'Saved {destination}', flush=True)


def main():
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=repository / 'maps/coverage_map')
    parser.add_argument('--output', type=Path, default=repository / 'scripts/result')
    parser.add_argument('--swath-angle', type=float, default=0.0, help='Radians')
    parser.add_argument('--contour-spacing', type=float, default=0.0,
                        help='CONCENTRIC inset step in meters; 0 uses server defaults')
    parser.add_argument('--modes', nargs='+',
                        choices=['BOUSTROPHEDON', 'SNAKE', 'SPIRAL', 'CUSTOM', 'CONCENTRIC'],
                        help='Only export these modes (default: all five)')
    parser.add_argument('--timeout', type=float, default=60.0, help='Seconds per request')
    args = parser.parse_args()
    if not Path('/.dockerenv').exists() and not Path('/run/.containerenv').exists():
        parser.error('Run in Docker using /home/hucheng/shell/navigation_sim.sh')
    if os.environ.get('ROS_DISTRO') != 'humble':
        parser.error('First source scripts/coverage_demo_env.sh in the Humble container')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')

    from opennav_coverage_demo.coverage_map import load_field
    from opennav_coverage_demo.coverage_map_client import make_coverage_goal
    from opennav_coverage_msgs.action import ComputeCoveragePath
    import rclpy
    from rclpy.action import ActionClient

    field = load_field(args.map_dir / 'field.json')
    rclpy.init()
    node = rclpy.create_node('coverage_result_exporter')
    client = ActionClient(node, ComputeCoveragePath, 'compute_coverage_path')
    entries = []
    cases = [
        ('boustrophedon', 'BOUSTROPHEDON', 4, 'BOUSTROPHEDON: adjacent rows'),
        ('snake', 'SNAKE', 4, 'SNAKE: skip rows, then return'),
        ('spiral_2', 'SPIRAL', 2, 'SPIRAL: groups of 2 swaths'),
        ('spiral_4', 'SPIRAL', 4, 'SPIRAL: groups of 4 swaths'),
        ('spiral_8', 'SPIRAL', 8, 'SPIRAL: groups of 8 swaths'),
        ('custom', 'CUSTOM', 4, 'CUSTOM example: every third row, then fill'),
        ('concentric', 'CONCENTRIC', 4, 'CONCENTRIC: successive inward contours (POLYLINE)'),
    ]
    if args.modes:
        cases = [case for case in cases if case[1] in args.modes]
    try:
        if not client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError('Coverage action unavailable; start run_coverage_map_demo.sh first')
        count = None
        for name, mode, spiral_n, title in cases:
            if mode == 'CUSTOM' and count is None:
                reference = request_plan(
                    node, client, make_coverage_goal(field, swath_angle=args.swath_angle),
                    args.timeout)
                count = len(reference.coverage_path.swaths)
            custom_order = (
                [index for offset in range(3) for index in range(offset, count, 3)]
                if mode == 'CUSTOM' else ())
            goal = make_coverage_goal(
                field, mode, spiral_n, args.swath_angle, custom_order, args.contour_spacing)
            print(f'Planning {title}', flush=True)
            result = request_plan(node, client, goal, args.timeout)
            if mode != 'CONCENTRIC':
                if count is None:
                    count = len(result.coverage_path.swaths)
                elif len(result.coverage_path.swaths) != count:
                    raise RuntimeError('Swath count changed between parallel-swath algorithms')
            if mode == 'CUSTOM':
                print(f'CUSTOM zero-based indices: {custom_order}', flush=True)
            entries.append(dict(name=name, mode=mode, spiral_n=spiral_n,
                                title=title, result=result, contour_spacing=args.contour_spacing))
            print(f'  {len(result.nav_path.poses)} poses, {path_length(result.nav_path):.2f} m',
                  flush=True)
        save_images(entries, field, args.map_dir, args.output)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
