#!/usr/bin/env python3
"""Render a saved camera coverage JSON as a full-field bird's-eye PNG."""
import argparse
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'gnc_offboard'))

from gnc_offboard.coverage_plot import save_coverage_png
from gnc_offboard.field_map import default_manifest_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('coverage', nargs='?', default=str(REPO/'plots/camera_coverage.json'))
    parser.add_argument('--manifest', default=str(default_manifest_path()),
                        help='Fallback for older coverage JSON without an embedded manifest')
    parser.add_argument('-o', '--out', help='Default: same filename as JSON, with .png extension')
    args = parser.parse_args()
    source = Path(args.coverage)
    output = save_coverage_png(json.loads(source.read_text()), args.manifest,
                               args.out or source.with_suffix('.png'))
    print(f'Wrote {output}')


if __name__ == '__main__':
    main()
