# RKNN model directory

Place the validated production model here as:

```text
traffic_light_yolov5s_fp16.rknn
```

The binary is intentionally not represented by a placeholder model. The ROS node
fails at startup when it is missing. Its required contract is documented in
`../YOLO_REBUILD_PLAN.md`.

