"""Operator CLI for selecting and safely serializing SLAM profiles."""

import argparse
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys

import yaml

from .map_state import load_state, profile_dir, save_state, state_root


def profiles(root):
    maps = root / "maps"
    if not maps.exists():
        return []
    return sorted(path.name for path in maps.iterdir() if path.is_dir())


def command_list(args):
    state = load_state(args.root)
    for name in profiles(args.root):
        marker = "*" if name == state["active_profile"] else " "
        print(f"{marker} {name}")


def command_status(args):
    state = load_state(args.root)
    if getattr(args, 'json', False):
        print(json.dumps(state))
        return
    active = state["active_profile"] or "none"
    print(f"Active profile: {active}\nMode: {state['mode']}")


def command_new(args):
    directory = profile_dir(args.name, args.root)
    if directory.exists():
        raise RuntimeError(f"Profile already exists: {args.name}")
    directory.mkdir(parents=True)
    metadata = {"name": args.name, "created_by": "armatron-map"}
    (directory / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    print(f"Created empty profile: {args.name}")


def set_selection(name, mode, root):
    directory = profile_dir(name, root)
    if not directory.is_dir():
        raise RuntimeError(f"No such ARMATRON profile: {name}")
    state = load_state(root)
    # Preserve the restart hint when changing mode/reselecting this map, but
    # never transfer coordinates from a different map.
    if state['active_profile'] != name:
        state = {'active_profile': name}
    state['mode'] = mode
    save_state(state, root)
    print(f"Active profile: {name} ({mode})")


def command_select(args):
    set_selection(args.name, args.mode or load_state(args.root)["mode"], args.root)


def command_set_mode(args):
    state = load_state(args.root)
    if not state["active_profile"]:
        raise RuntimeError("No active ARMATRON map profile. Run: ros2 run armatron armatron-map select <profile>")
    set_selection(state["active_profile"], args.mode, args.root)


def command_save(args):
    if (args.root / 'heading_fault.json').exists():
        raise RuntimeError('Map save refused: heading fault latched. Repair gyro and '
                           'verify localization before clearing heading_fault.json.')
    state = load_state(args.root)
    if not state["active_profile"]:
        raise RuntimeError("No active ARMATRON map profile. Run: ros2 run armatron armatron-map select <profile>")
    if state["mode"] != "mapping":
        print("Localization mode: map save is intentionally a no-op.")
        return

    directory = profile_dir(state["active_profile"], args.root)
    staging = directory / ".staging"
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir()
    filename = staging / "map"
    command = ["ros2", "service", "call", "/slam_toolbox/serialize_map",
               "slam_toolbox/srv/SerializePoseGraph", "{filename: '" + str(filename) + "'}"]
    try:
        completed = subprocess.run(command, text=True, capture_output=True,
                                   timeout=args.timeout)
    except OSError as error:
        shutil.rmtree(staging, ignore_errors=True)
        raise RuntimeError(f"Cannot invoke ros2 to save the map: {error}") from error
    # Humble ros2service prints repr(response), e.g.
    # SerializePoseGraph_Response(result=0), rather than YAML (result: 0).
    response = completed.stdout.partition('response:')[2]
    result = re.search(r'\bresult\s*[=:]\s*(-?\d+)\b', response)
    if completed.returncode != 0 or result is None or int(result.group(1)) != 0:
        raise RuntimeError("slam_toolbox serialization failed:\n" + completed.stdout + completed.stderr)

    artifacts = [filename.with_suffix(suffix) for suffix in ('.posegraph', '.data')]
    if any(not path.is_file() or path.stat().st_size == 0 for path in artifacts):
        raise RuntimeError("slam_toolbox reported success but did not produce both nonempty "
                           f"map.posegraph and map.data files; inspect {staging}")
    previous = directory / "previous"
    current = directory / "current"
    shutil.rmtree(previous, ignore_errors=True)
    if current.exists():
        current.replace(previous)
    staging.replace(current)
    print(f"Saved {state['active_profile']}; previous revision retained.")


def command_interactive(args):
    names = profiles(args.root)
    command_status(args)
    for index, name in enumerate(names, start=1):
        print(f"  {index}. {name}")
    selected = input("Select map (blank to keep current): ").strip()
    if selected:
        set_selection(names[int(selected) - 1], load_state(args.root)["mode"], args.root)


def parser():
    result = argparse.ArgumentParser(
        prog="ros2 run armatron armatron-map",
        description="Create, select and save ARMATRON map profiles.",
        epilog="First map: new primary, then select primary --mode mapping.")
    result.add_argument("--state-dir", type=Path, dest="root", default=state_root())
    commands = result.add_subparsers(dest="command")
    commands.add_parser("list", help="List available profiles").set_defaults(func=command_list)
    status = commands.add_parser("status", help="Show the active profile and mode")
    status.add_argument("--json", action="store_true")
    status.set_defaults(func=command_status)
    create = commands.add_parser("new", help="Create an empty profile")
    create.add_argument("name")
    create.set_defaults(func=command_new)
    select = commands.add_parser("select", help="Select a profile and optionally its mode")
    select.add_argument("name")
    select.add_argument("--mode", choices=("mapping", "localization"))
    select.set_defaults(func=command_select)
    mode = commands.add_parser("set-mode", help="Choose mapping or localization")
    mode.add_argument("mode", choices=("mapping", "localization"))
    mode.set_defaults(func=command_set_mode)
    save = commands.add_parser("save", help="Serialize the active SLAM map")
    save.add_argument("--timeout", type=float, default=20.0)
    save.set_defaults(func=command_save)
    return result


def main():
    cli = parser()
    args = cli.parse_args()
    if args.command is None:
        cli.print_help()
        return 0
    try:
        args.func(args)
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"armatron-map: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
