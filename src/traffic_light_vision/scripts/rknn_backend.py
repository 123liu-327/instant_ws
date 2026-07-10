#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure YOLOv5/RKNN inference and temporal-decision helpers.

This module deliberately has no ROS imports.  It can therefore be unit tested on
the development machine where RKNNLite is not installed.  RKNNLite is imported
only when :class:`YoloV5RknnBackend` is constructed.
"""

from __future__ import division

from collections import deque
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


CLASS_NAMES = ("red", "straight", "left", "right")
DEFAULT_ANCHORS = (
    (10.0, 13.0),
    (16.0, 30.0),
    (33.0, 23.0),
    (30.0, 61.0),
    (62.0, 45.0),
    (59.0, 119.0),
    (116.0, 90.0),
    (156.0, 198.0),
    (373.0, 326.0),
)


@dataclass(frozen=True)
class Detection:
    class_id: int
    state: str
    confidence: float
    box: Tuple[float, float, float, float]


def letterbox(
    image: np.ndarray,
    size: int = 640,
    color: Tuple[int, int, int] = (114, 114, 114),
) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """Resize with unchanged aspect ratio and symmetric padding."""
    if image is None or image.ndim != 3 or image.shape[0] == 0 or image.shape[1] == 0:
        raise ValueError("input image must be a non-empty HxWxC array")
    if size <= 0:
        raise ValueError("input size must be positive")

    height, width = image.shape[:2]
    ratio = min(float(size) / float(height), float(size) / float(width))
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

    pad_width = size - resized_width
    pad_height = size - resized_height
    left = int(round(pad_width / 2.0 - 0.1))
    right = int(round(pad_width / 2.0 + 0.1))
    top = int(round(pad_height / 2.0 - 0.1))
    bottom = int(round(pad_height / 2.0 + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return padded, ratio, (float(left), float(top))


def _sigmoid(values: np.ndarray) -> np.ndarray:
    clipped = np.clip(values, -60.0, 60.0)
    return 1.0 / (1.0 + np.exp(-clipped))


def _head_to_grid(output: np.ndarray, attributes: int) -> np.ndarray:
    """Normalize common RKNN output layouts to HxWx3xAttributes."""
    array = np.asarray(output)
    if array.ndim in (4, 5) and array.shape[0] == 1:
        array = array[0]

    channels = 3 * attributes
    if array.ndim == 3:
        if array.shape[0] == channels:
            array = np.transpose(array, (1, 2, 0))
        elif array.shape[-1] != channels:
            raise ValueError("YOLO head must contain {} channels, got {}".format(channels, array.shape))
        height, width = array.shape[:2]
        return array.reshape(height, width, 3, attributes)

    if array.ndim == 4:
        if array.shape[0] == 3 and array.shape[1] == attributes:
            return np.transpose(array, (2, 3, 0, 1))
        if array.shape[0] == 3 and array.shape[-1] == attributes:
            return np.transpose(array, (1, 2, 0, 3))
        if array.shape[-2:] == (3, attributes):
            return array

    raise ValueError("unsupported YOLO head layout: {}".format(array.shape))


def _box_iou_one_to_many(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    box_area = max(0.0, float(box[2] - box[0])) * max(0.0, float(box[3] - box[1]))
    areas = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(
        0.0, boxes[:, 3] - boxes[:, 1]
    )
    return intersection / np.maximum(box_area + areas - intersection, 1e-9)


def class_aware_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    threshold: float,
) -> List[int]:
    """Return retained indices after per-class NMS."""
    retained = []
    for class_id in np.unique(class_ids):
        indices = np.where(class_ids == class_id)[0]
        order = indices[np.argsort(scores[indices])[::-1]]
        while order.size:
            current = int(order[0])
            retained.append(current)
            if order.size == 1:
                break
            remaining = order[1:]
            overlaps = _box_iou_one_to_many(boxes[current], boxes[remaining])
            order = remaining[overlaps <= threshold]
    return sorted(retained, key=lambda index: float(scores[index]), reverse=True)


def decode_yolov5_heads(
    outputs: Sequence[np.ndarray],
    input_size: int,
    confidence_threshold: float,
    nms_threshold: float,
    class_names: Sequence[str] = CLASS_NAMES,
    anchors: Sequence[Sequence[float]] = DEFAULT_ANCHORS,
    apply_sigmoid: bool = False,
) -> List[Detection]:
    """Decode three post-sigmoid YOLOv5 heads produced by an RKNN model."""
    if len(outputs) != 3:
        raise ValueError("expected exactly three YOLO heads, got {}".format(len(outputs)))
    if len(anchors) != 9:
        raise ValueError("YOLOv5 requires exactly nine anchors")
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence threshold must be in [0, 1]")

    attributes = 5 + len(class_names)
    grids = [_head_to_grid(output, attributes).astype(np.float32, copy=False) for output in outputs]
    # A larger grid is the small-object head and must use the first anchor group.
    grids.sort(key=lambda item: item.shape[0] * item.shape[1], reverse=True)

    all_boxes = []
    all_scores = []
    all_classes = []
    anchors_array = np.asarray(anchors, dtype=np.float32).reshape(3, 3, 2)

    for head_index, grid in enumerate(grids):
        if apply_sigmoid:
            grid = _sigmoid(grid)
        grid_height, grid_width = grid.shape[:2]
        if grid_height != grid_width or input_size % grid_height != 0:
            raise ValueError(
                "unexpected YOLO grid {}x{} for input {}".format(
                    grid_height, grid_width, input_size
                )
            )

        objectness = grid[..., 4]
        class_probabilities = grid[..., 5:]
        class_ids = np.argmax(class_probabilities, axis=-1)
        class_scores = np.max(class_probabilities, axis=-1)
        scores = objectness * class_scores
        selected = scores >= confidence_threshold
        if not np.any(selected):
            continue

        y_indices, x_indices, anchor_indices = np.where(selected)
        selected_values = grid[y_indices, x_indices, anchor_indices]
        stride = float(input_size) / float(grid_height)
        centers_x = (selected_values[:, 0] * 2.0 - 0.5 + x_indices) * stride
        centers_y = (selected_values[:, 1] * 2.0 - 0.5 + y_indices) * stride
        selected_anchors = anchors_array[head_index, anchor_indices]
        widths = np.square(selected_values[:, 2] * 2.0) * selected_anchors[:, 0]
        heights = np.square(selected_values[:, 3] * 2.0) * selected_anchors[:, 1]

        boxes = np.column_stack(
            (
                centers_x - widths / 2.0,
                centers_y - heights / 2.0,
                centers_x + widths / 2.0,
                centers_y + heights / 2.0,
            )
        )
        all_boxes.append(boxes)
        all_scores.append(scores[selected])
        all_classes.append(class_ids[selected])

    if not all_boxes:
        return []

    boxes = np.concatenate(all_boxes).astype(np.float32)
    scores = np.concatenate(all_scores).astype(np.float32)
    class_ids = np.concatenate(all_classes).astype(np.int32)
    retained = class_aware_nms(boxes, scores, class_ids, nms_threshold)
    return [
        Detection(
            class_id=int(class_ids[index]),
            state=str(class_names[int(class_ids[index])]),
            confidence=float(scores[index]),
            box=tuple(float(value) for value in boxes[index]),
        )
        for index in retained
    ]


def scale_detections(
    detections: Iterable[Detection],
    ratio: float,
    padding: Tuple[float, float],
    image_shape: Tuple[int, int],
) -> List[Detection]:
    """Map letterboxed boxes back into the original image coordinate system."""
    height, width = image_shape
    pad_x, pad_y = padding
    scaled = []
    for detection in detections:
        x1, y1, x2, y2 = detection.box
        x1 = np.clip((x1 - pad_x) / ratio, 0.0, max(0.0, width - 1.0))
        y1 = np.clip((y1 - pad_y) / ratio, 0.0, max(0.0, height - 1.0))
        x2 = np.clip((x2 - pad_x) / ratio, 0.0, max(0.0, width - 1.0))
        y2 = np.clip((y2 - pad_y) / ratio, 0.0, max(0.0, height - 1.0))
        scaled.append(
            Detection(
                detection.class_id,
                detection.state,
                detection.confidence,
                (float(x1), float(y1), float(x2), float(y2)),
            )
        )
    return scaled


def select_primary_detection(detections: Sequence[Detection]) -> Optional[Detection]:
    """Select one lamp; red wins conflicts because stopping is the safe action."""
    red = [detection for detection in detections if detection.state == "red"]
    candidates = red if red else list(detections)
    if not candidates:
        return None
    return max(candidates, key=lambda detection: detection.confidence)


class TemporalVoter:
    """Confidence-aware temporal confirmation for traffic-light states."""

    DIRECTIONS = ("straight", "left", "right")

    def __init__(
        self,
        window_size: int = 7,
        direction_min_votes: int = 5,
        direction_min_average_confidence: float = 0.65,
        red_confirm_frames: int = 2,
    ):
        if window_size < 1:
            raise ValueError("window size must be positive")
        if not 1 <= direction_min_votes <= window_size:
            raise ValueError("direction_min_votes must be within the vote window")
        if not 1 <= red_confirm_frames <= window_size:
            raise ValueError("red_confirm_frames must be within the vote window")
        self.window_size = int(window_size)
        self.direction_min_votes = int(direction_min_votes)
        self.direction_min_average_confidence = float(direction_min_average_confidence)
        self.red_confirm_frames = int(red_confirm_frames)
        self.history = deque(maxlen=self.window_size)

    def reset(self) -> None:
        self.history.clear()

    def update(self, state: str, confidence: float) -> str:
        normalized_state = state if state in CLASS_NAMES else "unknown"
        self.history.append((normalized_state, float(confidence)))
        return self.stable_state()

    def stable_state(self) -> str:
        if len(self.history) >= self.red_confirm_frames:
            recent = list(self.history)[-self.red_confirm_frames :]
            if all(state == "red" for state, _ in recent):
                return "red"

        for state in self.DIRECTIONS:
            confidences = [confidence for item, confidence in self.history if item == state]
            if len(confidences) < self.direction_min_votes:
                continue
            if float(np.mean(confidences)) >= self.direction_min_average_confidence:
                return state
        return "unknown"


class YoloV5RknnBackend:
    """RKNNLite-backed YOLOv5 detector for the four traffic-light states."""

    def __init__(
        self,
        model_path: str,
        input_size: int = 640,
        confidence_threshold: float = 0.55,
        nms_threshold: float = 0.45,
        class_names: Sequence[str] = CLASS_NAMES,
        anchors: Sequence[Sequence[float]] = DEFAULT_ANCHORS,
        apply_sigmoid: bool = False,
        use_all_npu_cores: bool = True,
    ):
        if tuple(class_names) != CLASS_NAMES:
            raise ValueError("class order must be {}".format(CLASS_NAMES))
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError(
                "rknnlite is unavailable; install RKNN Toolkit Lite2 1.6.0 on the RK3588"
            ) from exc

        self.input_size = int(input_size)
        self.confidence_threshold = float(confidence_threshold)
        self.nms_threshold = float(nms_threshold)
        self.class_names = tuple(class_names)
        self.anchors = tuple(tuple(float(item) for item in pair) for pair in anchors)
        self.apply_sigmoid = bool(apply_sigmoid)
        self.rknn = RKNNLite(verbose=False)

        result = self.rknn.load_rknn(model_path)
        if result != 0:
            self.rknn.release()
            raise RuntimeError("failed to load RKNN model {} (code {})".format(model_path, result))

        if use_all_npu_cores:
            result = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        else:
            result = self.rknn.init_runtime()
        if result != 0:
            self.rknn.release()
            raise RuntimeError("failed to initialize RKNN runtime (code {})".format(result))

    def infer(self, bgr_image: np.ndarray) -> List[Detection]:
        prepared, ratio, padding = letterbox(bgr_image, self.input_size)
        rgb_image = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        # The converted RKNN model declares an NCHW, batched input.  OpenCV
        # images are HWC and the RKNN runtime will reject that three-dimensional
        # buffer ("need 4dims input").  Keep uint8 pixels, but make both the
        # channel order and batch dimension explicit: [1, 3, H, W].
        input_tensor = np.ascontiguousarray(
            rgb_image.transpose(2, 0, 1)[np.newaxis, ...]
        )
        outputs = self.rknn.inference(inputs=[input_tensor], data_format=["nchw"])
        if outputs is None:
            raise RuntimeError("RKNN inference returned no outputs")
        detections = decode_yolov5_heads(
            outputs,
            self.input_size,
            self.confidence_threshold,
            self.nms_threshold,
            self.class_names,
            self.anchors,
            self.apply_sigmoid,
        )
        return scale_detections(detections, ratio, padding, bgr_image.shape[:2])

    def release(self) -> None:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
