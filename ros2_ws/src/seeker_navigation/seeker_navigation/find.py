"""find — friendly CLI wrapper around the /seek_object action.

Usage:
  ros2 run seeker_navigation find teddy_bear
  ros2 run seeker_navigation find "sports ball" --timeout 60
  ros2 run seeker_navigation find chair -f

Class names that contain spaces (e.g. "teddy bear", "sports ball") may also be
written with underscores — "teddy_bear", "sports_ball" — which makes them easier
to type without quoting.
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from mcu_msgs.action import SeekObject


_STATE_NAMES = {
    SeekObject.Feedback.STATE_WAITING_FOR_NAV2: 'WAITING_FOR_NAV2',
    SeekObject.Feedback.STATE_INITIAL_ROTATION: 'INITIAL_ROTATION',
    SeekObject.Feedback.STATE_FRONTIER_NAV:     'FRONTIER_NAV',
    SeekObject.Feedback.STATE_COVERAGE_NAV:     'COVERAGE_NAV',
    SeekObject.Feedback.STATE_APPROACHING:      'APPROACHING',
}


class FindCli(Node):
    def __init__(self) -> None:
        super().__init__('seeker_find_cli')
        self._client = ActionClient(self, SeekObject, '/seek_object')
        self._goal_handle = None
        self._last_state: int | None = None

    # --------------------------------------------------------------

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, class_name: str, timeout_sec: float,
                  show_feedback: bool) -> int:
        goal = SeekObject.Goal()
        goal.class_name = class_name
        goal.timeout_sec = float(timeout_sec)

        print(f"-> seeking '{class_name}' "
              f"(timeout={timeout_sec:g}s{' with feedback' if show_feedback else ''})")

        feedback_cb = self._feedback_cb if show_feedback else None
        send_future = self._client.send_goal_async(
            goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            print('[FAIL] goal rejected by server')
            return 1
        self._goal_handle = gh
        print('[OK]   goal accepted, waiting for result...')

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped = result_future.result()
        if wrapped is None:
            print('[FAIL] no result received')
            return 1
        result = wrapped.result

        if result.success:
            p = result.final_position
            print(f"[DONE] SUCCESS: {result.message}")
            print(f"       final position (map): "
                  f"x={p.x:+.2f}  y={p.y:+.2f}  z={p.z:+.2f}")
            return 0

        print(f"[DONE] FAILED: {result.message}")
        return 2

    # --------------------------------------------------------------

    def _feedback_cb(self, fb_msg) -> None:
        fb = fb_msg.feedback
        state_name = _STATE_NAMES.get(fb.state, f'UNKNOWN({fb.state})')
        transition = fb.state != self._last_state
        self._last_state = fb.state

        if fb.last_bbox_area > 0.0:
            bearing = f'{math.degrees(fb.last_bearing):+6.1f} deg'
            bbox = f'{int(fb.last_bbox_area):>6d} px^2'
            target = f'target bbox={bbox} bearing={bearing}'
        else:
            target = 'target not seen'

        prefix = '>>' if transition else '  '
        print(f'{prefix} [{state_name:<17}] goals={fb.nav2_goal_count:<3}  {target}')

    # --------------------------------------------------------------

    def cancel(self) -> None:
        if self._goal_handle is None:
            return
        print('\n-- cancelling seek goal...')
        fut = self._goal_handle.cancel_goal_async()
        try:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        except Exception:
            pass


# ------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='ros2 run seeker_navigation find',
        description='Tell the Seeker robot to locate a COCO-class object.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  ros2 run seeker_navigation find teddy_bear\n'
            '  ros2 run seeker_navigation find "sports ball" --timeout 60\n'
            '  ros2 run seeker_navigation find chair -f\n'
            '\n'
            'Underscores in the class name are converted to spaces, so\n'
            '"teddy_bear" and "teddy bear" are equivalent.\n'
            '\n'
            'Exit codes: 0=found, 1=rejected/unavailable, 2=failed,\n'
            '            130=cancelled by user (Ctrl-C).'
        ),
    )
    parser.add_argument(
        'class_name',
        help='Target COCO class (e.g. teddy_bear, "sports ball", chair).',
    )
    parser.add_argument(
        '-t', '--timeout', type=float, default=120.0,
        help='Abort after N seconds (default 120; 0 = no timeout).',
    )
    parser.add_argument(
        '-f', '--feedback', action='store_true',
        help='Stream live state/bbox/bearing feedback while seeking.',
    )
    return parser


def main() -> None:
    argv = remove_ros_args(sys.argv)[1:]
    args = _build_parser().parse_args(argv)

    class_name = args.class_name.replace('_', ' ').strip()
    if not class_name:
        print('[FAIL] empty class_name')
        sys.exit(1)

    rclpy.init()
    node = FindCli()
    rc = 1
    try:
        if not node.wait_for_server(timeout_sec=5.0):
            print("[FAIL] action server '/seek_object' not available "
                  '(is object_seeker running?)')
            rc = 1
        else:
            rc = node.send_goal(class_name, args.timeout, args.feedback)
    except KeyboardInterrupt:
        try:
            node.cancel()
        except Exception:
            pass
        rc = 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)
