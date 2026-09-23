"""Small, file-backed state model for ARMATRON SLAM profiles."""

import os
from pathlib import Path
import tempfile

import yaml


def state_root():
    return Path(os.environ.get("ARMATRON_STATE_DIR", "/var/lib/armatron"))


def state_path(root=None):
    return (root or state_root()) / "state.yaml"


def load_state(root=None):
    path = state_path(root)
    if not path.exists():
        return {"active_profile": None, "mode": "localization"}
    with path.open(encoding="utf-8") as stream:
        state = yaml.safe_load(stream) or {}
    return {"active_profile": state.get("active_profile"),
            "mode": state.get("mode", "localization"),
            "last_pose": state.get("last_pose")}


def save_state(state, root=None):
    root = root or state_root()
    root.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=root,
                                     delete=False) as stream:
        yaml.safe_dump(state, stream, sort_keys=False)
        temporary = Path(stream.name)
    temporary.replace(state_path(root))


def profile_dir(name, root=None):
    if not name or Path(name).name != name or name in {".", ".."}:
        raise ValueError("Profile name must be a single directory name")
    return (root or state_root()) / "maps" / name
