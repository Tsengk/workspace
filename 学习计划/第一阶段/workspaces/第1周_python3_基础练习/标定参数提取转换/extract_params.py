#!/usr/bin/env python3
"""Update camera distortion/projection/mirror params in YAML files from proj_cfg."""

import re
import os

CFG_FILE = "cameras_distortion.proj_cfg"
CAMERAS = [
    "front_120", "left_front_100", "right_front_100",
    "left_back_100", "right_back_100", "back_120",
    "front_60", "left_back_60", "right_back_60",
    "front_fish_eye", "left_fish_eye", "right_fish_eye", "back_fish_eye",
]

# Parameter mapping: (section, key) -> source from block
PARAM_MAP = [
    ("distortion_parameters", "k1", lambda b: b["D"][0]),
    ("distortion_parameters", "k2", lambda b: b["D"][1]),
    ("distortion_parameters", "p1", lambda b: b["D"][2]),
    ("distortion_parameters", "p2", lambda b: b["D"][3]),
    ("mirror_parameters",    "xi", lambda b: b["Epsilon"]),
    ("projection_parameters", "gamma1", lambda b: b["K"][0]),
    ("projection_parameters", "gamma2", lambda b: b["K"][4]),
    ("projection_parameters", "u0",     lambda b: b["K"][2]),
    ("projection_parameters", "v0",     lambda b: b["K"][5]),
]


def parse_value(text):
    text = text.strip()
    if text.startswith("["):
        return [float(x) for x in text.strip("[]").split(",")]
    return float(text)


def is_balanced(text):
    return text.count("[") == text.count("]")


def read_blocks(cfg_path):
    """Parse proj_cfg and return dict of camera_name -> block."""
    with open(cfg_path, "r") as f:
        raw_lines = f.readlines()

    # Merge multi-line values (unbalanced brackets)
    merged = []
    for line in raw_lines:
        if merged and line.startswith((" ", "\t")):
            if not is_balanced(merged[-1]):
                merged[-1] = merged[-1].rstrip("\n") + " " + line.lstrip()
                continue
        merged.append(line)

    blocks = {}
    current_name = None
    current_block = None

    for line in merged:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        if not line.startswith((" ", "\t")) and stripped.endswith(":"):
            if current_name and current_block is not None:
                blocks[current_name] = current_block
            current_name = stripped.rstrip(":")
            current_block = {}
            continue

        if current_block is not None and line.startswith((" ", "\t")):
            if ":" in stripped:
                key, _, val = stripped.partition(":")
                val = val.strip()
                if val:
                    current_block[key] = parse_value(val)

    if current_name and current_block is not None:
        blocks[current_name] = current_block

    return blocks


def format_value(v):
    """Format a float for YAML with full precision."""
    return str(v)


def update_yaml(yaml_path, block, cam_name):
    """Update specific parameters in an existing YAML file in-place."""
    with open(yaml_path, "r") as f:
        content = f.read()

    for section, key, getter in PARAM_MAP:
        new_val = getter(block)
        pattern = re.compile(
            rf"^(\s*{re.escape(key)}:\s*).*$",
            re.MULTILINE,
        )

        if not pattern.search(content):
            print(f"  WARNING: {cam_name}: key {key} not found in yaml")
            continue

        def replacer(m):
            return m.group(1) + format_value(new_val)

        content = pattern.sub(replacer, content)

    with open(yaml_path, "w") as f:
        f.write(content)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(script_dir, CFG_FILE)

    blocks = read_blocks(cfg_path)

    for cam in CAMERAS:
        if cam not in blocks:
            print(f"WARNING: {cam} not found in config file, skipping")
            continue

        yaml_path = os.path.join(script_dir, f"{cam}.yaml")
        if not os.path.exists(yaml_path):
            print(f"WARNING: {yaml_path} does not exist, skipping")
            continue

        print(f"Updating {cam}.yaml")
        update_yaml(yaml_path, blocks[cam], cam)


if __name__ == "__main__":
    main()
