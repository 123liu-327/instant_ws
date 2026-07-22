#!/usr/bin/python3
"""v5 launch: reuse the v4 in-memory launch composition with the v5 manager."""

import sys

import xunfei2026_launch_src2_dual_stage_v4 as launch_v4


launch_v4.NEW_MANAGER = "xunfei2026_room_delivery_src2_dual_stage_v5.py"


if __name__ == "__main__":
    try:
        launch_v4.main()
    except Exception as exc:
        sys.stderr.write("[ERROR] src2 dual-stage v5 launch failed: {}\n".format(exc))
        sys.exit(1)
