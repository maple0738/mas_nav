"""Navigation configuration and map paths, shared by launch and packaging."""

import argparse
from pathlib import Path

import yaml


PARAMETER_FILES = (
    "node_params.yaml",
    "perception.yaml",
    "terrain_recognition.yaml",
    "planner_params.yaml",
    "mpc_params.yaml",
    "region_control.yaml",
)


def read_manifest(path, map_source=""):
    path = Path(path).expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a mapping")
    mode = map_source or document.get("map_source")
    if mode not in ("online", "prior"):
        raise ValueError(f"{path}: map_source must be online or prior")
    root = document.get("map_root", "..")
    log_dir = document.get("log_dir", "~/.local/state/mas_nav")
    if not isinstance(root, str) or not root.strip():
        raise ValueError(f"{path}: map_root must be a non-empty path")
    if not isinstance(log_dir, str) or not log_dir.strip():
        raise ValueError(f"{path}: log_dir must be a non-empty path")

    def resolve(value):
        expanded = Path(value).expanduser()
        return (path.parent / expanded).resolve()

    return {
        "path": str(path),
        "map_source": mode,
        "map_root": str(resolve(root)),
        "log_dir": str(resolve(log_dir)),
        "map_files": document.get("map_files", {}),
    }


def resolve_map_files(manifest):
    """Validate the complete prior-map bundle, including occupancy image."""
    files = manifest["map_files"]
    if not isinstance(files, dict):
        raise ValueError(f"{manifest['path']}: map_files must be a mapping")
    resolved = {}
    for key in ("localization_pcd", "occupancy_yaml", "terrain_msgpack"):
        value = files.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{manifest['path']}: map_files.{key} must name a file")
        path = (Path(manifest["map_root"]) / Path(value).expanduser()).resolve()
        if not path.is_file():
            raise ValueError(f"{manifest['path']}: map_files.{key} does not exist: {path}")
        resolved[key] = str(path)
    occupancy = Path(resolved["occupancy_yaml"])
    with occupancy.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image = metadata.get("image") if isinstance(metadata, dict) else None
    if not isinstance(image, str) or not image.strip():
        raise ValueError(f"{occupancy}: image must name a file")
    image_path = (occupancy.parent / Path(image).expanduser()).resolve()
    if not image_path.is_file():
        raise ValueError(f"{occupancy}: image does not exist: {image_path}")
    resolved["occupancy_image"] = str(image_path)
    return resolved


def parameter_files(config_dir):
    """Return the explicit file set and reject ambiguous parameter ownership."""
    root = Path(config_dir).expanduser().resolve()
    paths = [root / name for name in PARAMETER_FILES]
    owners = {}

    def leaves(document, prefix=""):
        for key, value in document.items():
            name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                yield from leaves(value, name)
            else:
                yield name

    for path in paths:
        if not path.is_file():
            raise ValueError(f"Missing navigation parameter file: {path}")
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        try:
            parameters = document["/**"]["ros__parameters"]
        except (KeyError, TypeError) as error:
            raise ValueError(f"{path}: missing /**.ros__parameters mapping") from error
        if not isinstance(parameters, dict):
            raise ValueError(f"{path}: ros__parameters must be a mapping")
        for key in leaves(parameters):
            if key in owners:
                raise ValueError(f"{path}: duplicate parameter {key}, also in {owners[key]}")
            owners[key] = str(path)
    return [str(path) for path in paths]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest")
    args = parser.parse_args()
    selected = read_manifest(args.manifest)
    try:
        bundle = resolve_map_files(selected)
    except (ValueError, OSError, yaml.YAMLError):
        if selected["map_source"] == "prior":
            raise
        bundle = {}  # Online builds do not require any prior-map assets.
    for filename in bundle.values():
        print(filename)
