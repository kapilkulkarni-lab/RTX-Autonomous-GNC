#!/usr/bin/env python3
"""Compare independent camera coverage JSON exports; no ROS required."""
import argparse
import csv
import json
import math
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    args = parser.parse_args()
    writer = csv.writer(sys.stdout)
    writer.writerow(['run', 'coverage_percent', 'observed_dice', 'target_found',
                     'recorded_horizontal_distance_m'])
    for path in args.runs:
        data = json.loads(path.read_text())
        points = data.get('trajectory_enu', [])
        distance = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                       for a, b in zip(points, points[1:]) if a is not None and b is not None)
        writer.writerow([path.stem,
                         round(100*data['searched_area_m2']/data['arena_area_m2'], 2),
                         len(data.get('observed_dice', [])), data.get('target_found'),
                         round(distance, 2)])


if __name__ == '__main__':
    main()
