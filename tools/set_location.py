#!/usr/bin/env python3
import argparse
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description='Update one named navigation location in a YAML file.')
    parser.add_argument('yaml_file', help='Path to a locations YAML file.')
    parser.add_argument('location', help='Location key, for example kitchen or living_room.')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, nargs='?', default=0.0)
    parser.add_argument('--name', help='Human-readable display name. Defaults to the location key.')
    args = parser.parse_args()

    yaml_path = Path(args.yaml_file).expanduser()
    if yaml_path.exists():
        with yaml_path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    else:
        data = {}

    locations = data.setdefault('locations', {})
    old = locations.get(args.location, {})
    locations[args.location] = {
        'name': args.name or old.get('name') or args.location.replace('_', ' '),
        'x': float(args.x),
        'y': float(args.y),
        'yaw': float(args.yaw),
    }

    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with yaml_path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)

    print(f'Updated {args.location}: x={args.x}, y={args.y}, yaw={args.yaw} in {yaml_path}')


if __name__ == '__main__':
    main()
