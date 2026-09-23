"""Recover navigation after corrupted wheel odometry.

The robot must be stationary and have no active navigation goal. This command
resets local wheel odometry, then reloads a known posegraph at the supplied
map-frame pose so slam_toolbox can scan-match and rebuild map-to-odom.
"""

import argparse
import math

from .posegraph import call_service, load, posegraph_path


def recover(name, x, y, yaw_degrees):
    path = posegraph_path(name)
    if not path.exists():
        raise RuntimeError(f'{path} does not exist.')

    yaw = math.radians(yaw_degrees)
    print('Resetting wheel odometry.')
    call_service('/odom_reset', 'std_srvs/srv/Empty', '{}')
    print(f'Reloading {path} at x={x}, y={y}, yaw={yaw_degrees} degrees.')
    load(name, x, y, yaw)
    print('Posegraph reloaded. Verify the pose in RViz before sending a goal.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('posegraph')
    parser.add_argument('--x', type=float, required=True,
                        help='robot x position in the map frame, in metres')
    parser.add_argument('--y', type=float, required=True,
                        help='robot y position in the map frame, in metres')
    parser.add_argument('--yaw-degrees', type=float, required=True,
                        help='robot yaw in the map frame, in degrees')
    parser.add_argument('--confirm-stationary', action='store_true',
                        help='confirm that the robot is stopped and its goal is canceled')
    args = parser.parse_args(argv)
    if not args.confirm_stationary:
        parser.error('refusing to reset odometry until --confirm-stationary is supplied')
    try:
        recover(args.posegraph, args.x, args.y, args.yaw_degrees)
    except RuntimeError as error:
        parser.exit(1, f'armatron-navigation-recover: error: {error}\n')


if __name__ == '__main__':
    main()
