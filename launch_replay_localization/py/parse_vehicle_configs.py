#!/usr/bin/env python3
"""Emit vehicle configs from a YAML file as pipe-delimited lines for bash.

Output format (one line per vehicle):
    fragment|model|id|sensor

Consumed by ../vehicle_configs.sh via `mapfile`.
"""
import sys

import yaml


def main() -> int:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <vehicle_hash_map.yaml>", file=sys.stderr)
        return 2

    path = sys.argv[1]
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: file not found: {path}", file=sys.stderr)
        return 1
    except yaml.YAMLError as exc:
        print(f"Error: failed to parse YAML {path}: {exc}", file=sys.stderr)
        return 1

    vehicles = (data or {}).get("vehicles")
    if not isinstance(vehicles, list):
        print(f"Error: '{path}' must contain a 'vehicles' list", file=sys.stderr)
        return 1

    fields = ("fragment", "model", "id", "sensor")
    for i, entry in enumerate(vehicles):
        if not isinstance(entry, dict):
            print(f"Error: vehicle entry #{i} is not a mapping", file=sys.stderr)
            return 1
        values = []
        for key in fields:
            if key not in entry or entry[key] is None or str(entry[key]) == "":
                print(f"Error: vehicle entry #{i} missing '{key}'", file=sys.stderr)
                return 1
            value = str(entry[key])
            if "|" in value:
                print(
                    f"Error: vehicle entry #{i} field '{key}' contains '|'",
                    file=sys.stderr,
                )
                return 1
            values.append(value)
        print("|".join(values))

    return 0


if __name__ == "__main__":
    sys.exit(main())
