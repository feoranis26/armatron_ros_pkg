"""Small command-line interface for slam_toolbox posegraph files."""

import argparse
import math
import os
from pathlib import Path
import subprocess
import sys


DEFAULT_DIRECTORY = Path.home() / '.local' / 'share' / 'armatron' / 'posegraphs'


def posegraph_directory():
    return Path(os.environ.get('ARMATRON_POSEGRAPH_DIR', DEFAULT_DIRECTORY)).expanduser()


def posegraph_path(name):
    path = Path(name)
    if path.suffix or path.parent != Path('.'):
        return path.expanduser()
    return posegraph_directory() / f'{name}.posegraph'


def call_service(service, service_type, payload):
    command = ['ros2', 'service', 'call', service, service_type, payload]
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError:
        raise RuntimeError('ros2 is not available; source the ROS 2 environment first.')
    except subprocess.CalledProcessError as error:
        raise RuntimeError(
            f'{service} did not accept the request ({error.returncode}). '
            'Confirm that the service is available in ROS domain 67.'
        )


def save(name, force):
    path = posegraph_path(name)
    if path.exists() and not force:
        raise RuntimeError(f'{path} already exists; pass --force to replace it.')
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f'Saving posegraph to {path}')
    call_service(
        '/slam_toolbox/serialize_map',
        'slam_toolbox/srv/SerializePoseGraph',
        "{filename: '" + str(path) + "'}",
    )


def load(name, x=0.0, y=0.0, yaw=0.0):
    path = posegraph_path(name)
    if not path.exists():
        raise RuntimeError(f'{path} does not exist.')
    print(f'Loading posegraph from {path}')
    call_service(
        '/slam_toolbox/deserialize_map',
        'slam_toolbox/srv/DeserializePoseGraph',
        "{filename: '" + str(path) +
        "', match_type: 2, initial_pose: {position: {x: " + str(x) +
        ", y: " + str(y) + ", z: 0.0}, orientation: {x: 0.0, y: 0.0, z: " +
        str(math.sin(yaw / 2.0)) + ", w: " + str(math.cos(yaw / 2.0)) + "}}}",
    )


def list_posegraphs():
    directory = posegraph_directory()
    if not directory.exists():
        print(f'No posegraphs: {directory} does not exist.')
        return
    for path in sorted(directory.glob('*.posegraph')):
        print(path.stem)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest='command', required=True)
    save_parser = subcommands.add_parser('save', help='save the active posegraph')
    save_parser.add_argument('name')
    save_parser.add_argument('--force', action='store_true')
    load_parser = subcommands.add_parser('load', help='load a saved posegraph')
    load_parser.add_argument('name')
    load_parser.add_argument('--x', type=float, default=0.0)
    load_parser.add_argument('--y', type=float, default=0.0)
    load_parser.add_argument('--yaw', type=float, default=0.0,
                             help='initial map-frame yaw in radians')
    subcommands.add_parser('list', help='list saved posegraphs')
    args = parser.parse_args(argv)
    try:
        if args.command == 'save':
            save(args.name, args.force)
        elif args.command == 'load':
            load(args.name, args.x, args.y, args.yaw)
        else:
            list_posegraphs()
    except RuntimeError as error:
        parser.exit(1, f'armatron-posegraph: error: {error}\n')


if __name__ == '__main__':
    main()
