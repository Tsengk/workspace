#!/usr/bin/env python3
import argparse
from pathlib import Path
import shutil
import subprocess

import yaml


def prepare(input_path, output_path):
    input_path = Path(input_path).resolve()
    output_path = Path(output_path).resolve()
    generator = shutil.which("dcfgen")
    if generator is None:
        raise ValueError("dcfgen not found; source the workspace containing lely_core_libraries")
    with input_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or "master" not in config or len(config) != 5:
        raise ValueError("Expected a master and exactly four independently controlled motors")
    node_ids = set()
    sources = {}
    for name, entry in config.items():
        if not isinstance(entry, dict):
            raise ValueError("Invalid bus entry: " + name)
        node_id = entry.get("node_id")
        if type(node_id) is not int or not 1 <= node_id <= 127 or node_id in node_ids:
            raise ValueError("CANopen node IDs must be unique integers in [1, 127]")
        node_ids.add(node_id)
        if name == "master":
            continue
        if entry.get("driver") != "ros2_canopen::MotionControllerDriver":
            raise ValueError("Expected the Foxy MotionControllerDriver for " + name)
        source = Path(entry.get("dcf", ""))
        if not source.is_absolute():
            source = input_path.parent / source
        source = source.resolve()
        if not source.is_file():
            raise ValueError("Supply the real manufacturer's EDS/DCF: " + str(source))
        if source.name in sources and sources[source.name] != source:
            raise ValueError("Different EDS files share a basename: " + source.name)
        sources[source.name] = source
        entry["dcf"] = source.name
        entry["dcf_path"] = str(output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    for filename, source in sources.items():
        destination = output_path / filename
        if destination != source:
            shutil.copy2(str(source), str(destination))
    bus_path = output_path / "bus.yml"
    with bus_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    subprocess.run([generator, "-d", str(output_path), "-rS", str(bus_path)], check=True,
                   cwd=str(output_path))
    if not (output_path / "master.dcf").is_file():
        raise ValueError("dcfgen did not generate master.dcf")
    print("Prepared bus configuration: " + str(bus_path))
    print("Master DCF: " + str(output_path / "master.dcf"))


def main():
    parser = argparse.ArgumentParser(description="Validate and generate a Foxy CANopen bus configuration")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-dir", required=True)
    arguments = parser.parse_args()
    try:
        prepare(arguments.input, arguments.output_dir)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error) + "\n")


if __name__ == "__main__":
    main()
