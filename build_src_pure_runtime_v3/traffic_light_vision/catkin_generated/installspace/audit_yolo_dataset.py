#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Audit a four-class YOLO dataset before traffic-light training."""

import argparse
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2


CLASS_NAMES = ("red", "straight", "left", "right")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
SPLITS = ("train", "val", "test")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate labels, images and split leakage in a YOLO dataset"
    )
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--report", type=Path, help="optional JSON report path")
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def label_path_for(image_path, dataset_root, split):
    relative = image_path.relative_to(dataset_root / "images" / split)
    return (dataset_root / "labels" / split / relative).with_suffix(".txt")


def parse_label(label_path, errors, class_counts):
    lines = label_path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return True
    valid = True
    for line_number, line in enumerate(lines, 1):
        fields = line.split()
        if len(fields) != 5:
            errors.append("{}:{} expected 5 fields".format(label_path, line_number))
            valid = False
            continue
        try:
            class_value = float(fields[0])
            values = [float(value) for value in fields[1:]]
        except ValueError:
            errors.append("{}:{} contains a non-numeric value".format(label_path, line_number))
            valid = False
            continue
        if not class_value.is_integer() or int(class_value) not in range(len(CLASS_NAMES)):
            errors.append("{}:{} invalid class {}".format(label_path, line_number, fields[0]))
            valid = False
            continue
        if not all(math.isfinite(value) for value in values):
            errors.append("{}:{} contains NaN or infinity".format(label_path, line_number))
            valid = False
            continue
        x_center, y_center, width, height = values
        in_range = all(0.0 <= value <= 1.0 for value in values)
        inside_image = (
            width > 0.0
            and height > 0.0
            and x_center - width / 2.0 >= -1e-6
            and x_center + width / 2.0 <= 1.0 + 1e-6
            and y_center - height / 2.0 >= -1e-6
            and y_center + height / 2.0 <= 1.0 + 1e-6
        )
        if not in_range or not inside_image:
            errors.append("{}:{} box is outside normalized image bounds".format(label_path, line_number))
            valid = False
            continue
        class_counts[int(class_value)] += 1
    return valid


def audit(dataset_root):
    errors = []
    warnings = []
    class_counts = Counter()
    split_counts = Counter()
    empty_labels = Counter()
    hashes = defaultdict(list)

    for split in SPLITS:
        image_root = dataset_root / "images" / split
        label_root = dataset_root / "labels" / split
        if not image_root.is_dir():
            errors.append("missing image split: {}".format(image_root))
            continue
        if not label_root.is_dir():
            errors.append("missing label split: {}".format(label_root))
            continue

        images = sorted(
            path for path in image_root.rglob("*") if path.suffix.lower() in IMAGE_SUFFIXES
        )
        if not images:
            errors.append("split {} contains no images".format(split))
            continue
        for image_path in images:
            split_counts[split] += 1
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None or image.size == 0:
                errors.append("unreadable image: {}".format(image_path))
                continue
            hashes[sha256(image_path)].append((split, str(image_path)))
            label_path = label_path_for(image_path, dataset_root, split)
            if not label_path.is_file():
                errors.append("missing label: {}".format(label_path))
                continue
            if label_path.stat().st_size == 0:
                empty_labels[split] += 1
            parse_label(label_path, errors, class_counts)

    for paths in hashes.values():
        distinct_splits = {split for split, _ in paths}
        if len(distinct_splits) > 1:
            errors.append(
                "duplicate image crosses splits {}: {}".format(
                    sorted(distinct_splits), [path for _, path in paths]
                )
            )

    for class_id, class_name in enumerate(CLASS_NAMES):
        if class_counts[class_id] == 0:
            errors.append("class {} ({}) has no boxes".format(class_id, class_name))
    if class_counts:
        nonzero = [class_counts[index] for index in range(len(CLASS_NAMES)) if class_counts[index]]
        if nonzero and max(nonzero) / float(min(nonzero)) > 5.0:
            warnings.append("class imbalance exceeds 5:1")

    return {
        "ok": not errors,
        "class_order": list(CLASS_NAMES),
        "images_per_split": dict(split_counts),
        "empty_labels_per_split": dict(empty_labels),
        "boxes_per_class": {
            CLASS_NAMES[index]: class_counts[index] for index in range(len(CLASS_NAMES))
        },
        "warnings": warnings,
        "errors": errors,
    }


def main():
    args = parse_args()
    root = args.dataset_root.resolve()
    report = audit(root)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())

