#!/usr/bin/python3
"""Full flow: src2 anchor navigation with exclusive stage-locked v1 parking."""

import sys

import xunfei2026_launch_src2_dual_stage_v4 as launch_v4


launch_v4.NEW_MANAGER = "xunfei2026_room_delivery_v1_locked_v9.py"


if __name__ == "__main__":
    try:
        launch_v4.main()
    except Exception as exc:
        sys.stderr.write(
            "[ERROR] src2 navigation + locked v1 parking failed: {}\n".format(
                exc))
        sys.exit(1)
