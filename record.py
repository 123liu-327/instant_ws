#!/usr/bin/env python3
import cv2
import sys
import signal

# 自动寻找可用摄像头
cap = None
for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"成功打开摄像头 /dev/video{i}")
        break
    cap.release()

if cap is None or not cap.isOpened():
    print("错误：没有找到可用的摄像头！")
    sys.exit(1)

# 设置参数（必须与比赛推理时一致）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# 视频保存
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 30.0, (640, 480))

# 录制停止标志
stop_recording = False

def signal_handler(sig, frame):
    global stop_recording
    stop_recording = True
    print("\n录制手动停止，正在保存视频...")

# 注册 Ctrl+C 信号
signal.signal(signal.SIGINT, signal_handler)

print("开始录制（无预览窗口），按 Ctrl+C 停止...")
try:
    while not stop_recording:
        ret, frame = cap.read()
        if not ret:
            print("警告：丢失画面")
            break
        frame = cv2.flip(frame, 1)
        out.write(frame)
except KeyboardInterrupt:
    pass  # 已经由 signal_handler 处理

cap.release()
out.release()
cv2.destroyAllWindows()
print("视频已保存为 output.avi")