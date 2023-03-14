#!/usr/bin/env python3

import yaml
import sys

try:
    with open('../waypoints_cfg/waypoints.yaml') as file:
        obj = yaml.safe_load(file)
        print(obj["waypoints"][0]["point"]["pose"])
        print(obj["waypoints"][0]["point"]["orientation"])
        with open('output.yaml', 'w') as file:
            yaml.dump(obj, file)
except Exception as e:
    print('Exception occurred while loading YAML...', file=sys.stderr)
    print(e, file=sys.stderr)
    sys.exit(1)