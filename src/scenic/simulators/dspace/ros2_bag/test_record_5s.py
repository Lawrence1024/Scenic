#!/usr/bin/env python3
"""Smoke-test ROS 2 bag start/stop without running Scenic.

Starts ``ros2 bag record`` in the Docker container (same path as dSPACE / ART reset:
``docker exec <container> bash -c ...``), waits a fixed duration, then sends SIGINT via
``Ros2BagRecorder.stop()``.

Run from the repo root with Scenic on ``PYTHONPATH`` or an editable install::

    python -m scenic.simulators.dspace.ros2_bag.test_record_5s

Or::

    python src/scenic/simulators/dspace/ros2_bag/test_record_5s.py

Examples::

    python -m scenic.simulators.dspace.ros2_bag.test_record_5s --container art_driving_stack
    python -m scenic.simulators.dspace.ros2_bag.test_record_5s --duration 10
    python -m scenic.simulators.dspace.ros2_bag.test_record_5s --topics /topic/a /topic/b
"""

from __future__ import annotations

import argparse
import sys
import time

from scenic.simulators.dspace.ros2_bag.config import ART_STACK_DEFAULT_SETUP, Ros2BagConfig
from scenic.simulators.dspace.ros2_bag.recorder import Ros2BagRecorder


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Record a ROS 2 bag for N seconds (test start/stop without Scenic).",
    )
    parser.add_argument(
        "--container",
        default="art_driving_stack",
        help="Docker container name (default: art_driving_stack)",
    )
    parser.add_argument(
        "--parent-dir",
        default="/ros_bags",
        help="Bag output parent directory inside the container (default: /ros_bags)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to record before stop (default: 5)",
    )
    parser.add_argument(
        "--setup-source",
        default=ART_STACK_DEFAULT_SETUP,
        help=f"Shell line run before ros2 bag record (default: {ART_STACK_DEFAULT_SETUP!r})",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=None,
        metavar="TOPIC",
        help="If set, record only these topics; otherwise use -a (all)",
    )
    args = parser.parse_args(argv)

    if args.topics:
        record_all = False
        topics = tuple(str(t).strip() for t in args.topics if str(t).strip())
        if not topics:
            print("error: --topics was empty", file=sys.stderr)
            return 2
    else:
        record_all = True
        topics = ()

    setup = (args.setup_source or "").strip() or ART_STACK_DEFAULT_SETUP
    cfg = Ros2BagConfig(
        container=args.container.strip(),
        bag_parent_dir=args.parent_dir.strip() or "/ros_bags",
        record_all_topics=record_all,
        topics=topics,
        setup_source_line=setup,
    )

    print(f"[test_record_5s] container={cfg.container!r} duration={args.duration}s")
    rec = Ros2BagRecorder(cfg)
    if not rec.start():
        print("[test_record_5s] start failed", file=sys.stderr)
        return 1

    print(f"[test_record_5s] recording for {args.duration} s ...")
    try:
        time.sleep(max(0.0, float(args.duration)))
    except KeyboardInterrupt:
        print("\n[test_record_5s] interrupted; stopping ...")
    rec.stop()
    print("[test_record_5s] done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
