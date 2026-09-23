"""Operator CLI for selecting and safely serializing SLAM profiles."""

import argparse
import json
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
    if args.json:
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
    save_state({"active_profile": name, "mode": mode}, root)
    print(f"Active profile: {name} ({mode})")


def command_select(args):
    set_selection(args.name, args.mode or load_state(args.root)["mode"], args.root)


def command_set_mode(args):
    state = load_state(args.root)
    if not state["active_profile"]:
        raise RuntimeError("No active ARMATRON map profile. Run: armatron-map select <profile>")
    set_selection(state["active_profile"], args.mode, args.root)


def command_save(args):
    state = load_state(args.root)
    if not state["active_profile"]:
        raise RuntimeError("No active ARMATRON map profile. Run: armatron-map select <profile>")
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
    if completed.returncode != 0 or "result: 0" not in completed.stdout:
        shutil.rmtree(staging, ignore_errors=True)
        raise RuntimeError("slam_toolbox serialization failed:\n" + completed.stdout + completed.stderr)

    artifacts = list(staging.iterdir())
    if not artifacts:
        raise RuntimeError("slam_toolbox reported success but produced no map artifacts")
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
    result = argparse.ArgumentParser(prog="armatron-map")
    result.add_argument("--state-dir", type=Path, dest="root", default=state_root())
    commands = result.add_subparsers(dest="command")
    commands.add_parser("list").set_defaults(func=command_list)
    status = commands.add_parser("status")
    status.add_argument("--json", action="store_true")
    status.set_defaults(func=command_status)
    create = commands.add_parser("new")
    create.add_argument("name")
    create.set_defaults(func=command_new)
    select = commands.add_parser("select")
    select.add_argument("name")
    select.add_argument("--mode", choices=("mapping", "localization"))
    select.set_defaults(func=command_select)
    mode = commands.add_parser("set-mode")
    mode.add_argument("mode", choices=("mapping", "localization"))
    mode.set_defaults(func=command_set_mode)
    save = commands.add_parser("save")
    save.add_argument("--timeout", type=float, default=20.0)
    save.set_defaults(func=command_save)
    return result


def main():
    args = parser().parse_args()
    try:
        if args.command is None:
            command_interactive(args)
        else:
            args.func(args)
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"armatron-map: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
