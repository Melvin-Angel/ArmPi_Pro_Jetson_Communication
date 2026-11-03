#!/usr/bin/env python3
"""Print candidate encoder counts without sending any motor commands."""
import argparse
import time
# Catkin devel-space wrappers execute the source script from another directory.
import os
import sys
import rospkg
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('armpi_jetson_bridge'), 'scripts'))
from encoder_reader import read_counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval', type=float, default=0.2)
    parser.add_argument('--bus', type=int, default=1)
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error('interval must be positive')
    print('Reading candidate register 0x3c; Ctrl-C to stop.', flush=True)
    try:
        while True:
            print('M1..M4:', read_counts(args.bus), flush=True)
            time.sleep(args.interval)
    except (OSError, ValueError) as exc:
        raise SystemExit('Encoder read failed: {}'.format(exc))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
