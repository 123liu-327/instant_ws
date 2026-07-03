#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert a validated three-head YOLOv5 ONNX model to RK3588 FP16 RKNN."""

import argparse
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx_model", type=Path)
    parser.add_argument("output_model", type=Path)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def require_success(stage, result):
    if result != 0:
        raise RuntimeError("{} failed with code {}".format(stage, result))


def main():
    args = parse_args()
    if not args.onnx_model.is_file():
        raise RuntimeError("ONNX model does not exist: {}".format(args.onnx_model))
    try:
        from rknn.api import RKNN
    except ImportError as exc:
        raise RuntimeError(
            "rknn-toolkit2 1.6.0 is required on the conversion machine"
        ) from exc

    args.output_model.parent.mkdir(parents=True, exist_ok=True)
    rknn = RKNN(verbose=args.verbose)
    try:
        require_success(
            "RKNN config",
            rknn.config(
                mean_values=[[0.0, 0.0, 0.0]],
                std_values=[[255.0, 255.0, 255.0]],
                target_platform="rk3588",
            ),
        )
        require_success("ONNX load", rknn.load_onnx(model=str(args.onnx_model)))
        # do_quantization=False is the locked first-release FP16 policy.
        require_success("RKNN build", rknn.build(do_quantization=False))
        require_success("RKNN export", rknn.export_rknn(str(args.output_model)))
    finally:
        rknn.release()
    print("exported {}".format(args.output_model))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        sys.exit(1)

