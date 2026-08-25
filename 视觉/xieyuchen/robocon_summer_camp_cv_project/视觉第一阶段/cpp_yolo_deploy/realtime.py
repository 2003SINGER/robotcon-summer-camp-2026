import cv2
import sys
import os
import numpy as np
from ultralytics import YOLO

import time

import contextlib
import io




fps_counter = 0
fps_start_time = time.time()



# 1代表我们是蓝色， 0代表我们是红色
ourColor = 1
# 1为左边场地， 0为右边场地
ourPlace = 1
# 从距离镜头远近排序


squareSize = 26
REAL_H = 0.35
fx_approx = 718  # 标定的焦距,这里是fx

# ---------- 路径 ----------
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, 'models', 'best.pt')  # onnx
model = YOLO(model_path)

# ---------- 摄像头 ----------
cap = cv2.VideoCapture('/dev/video4')

# 在加载模型前，把 stdout 重定向到 devnull
with open(os.devnull, 'w') as devnull:
    with contextlib.redirect_stdout(devnull):
        model = YOLO(model_path)   # 这里的加载信息被吞掉


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("输出格式: x1 y1 x2 y2 cls angle interference_pos", file=sys.stderr)
print("按 ESC 退出", file=sys.stderr)


# pass
def get_kfs_angle_by_edges(roi):
    # 通过边缘检测提取KFS外轮廓，返回角度（度），范围 [-45, 45)
    #  roi 从原图中裁剪出的、只包含目标物体的小图像块（即 YOLO 检测框内的区域）。
    if roi.size == 0:
        return 0.0
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    max_cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(max_cnt) < 50:
        return 0.0
    rect = cv2.minAreaRect(max_cnt)
    angle = rect[2]
    if angle < -45:
        angle += 90
    elif angle > 45:
        angle -= 90
    
    return angle
    




while True:
    ret, frame = cap.read()
    if not ret:
        break

# --------------测试-------------------------------------
    fps_counter += 1
    if time.time() - fps_start_time >= 1.0:  # 每秒更新一次
        print(f"[性能] 当前 FPS: {fps_counter}", file=sys.stderr)
        fps_counter = 0
        fps_start_time = time.time()


    results = model(frame, imgsz=640, verbose=False)
    boxes = results[0].boxes

    # ---------- 收集所有检测框 ----------
    detections = []
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        if conf < 0.9:
            continue
        area = (x2 - x1) * (y2 - y1)
        center_x = (x1 + x2) / 2.0
        detections.append((x1, y1, x2, y2, cls, area, center_x))

    # ---------- 按中心x排序 ----------
    # 1为左边场地， 0为右边场地

    detections.sort(key=lambda d: -d[6]) if ourPlace == 0 else detections.sort(key=lambda d: d[6])

    # ---------- 找出干扰项的位置（序号从1开始） ----------
    interference_positions = []
    for idx, d in enumerate(detections, start=1):
        cls = d[4]
        if cls != ourColor:   
            interference_positions.append(idx)

    # ---------- 筛选本队（类别1） ----------
    team_detections = [d for d in detections if d[4] == ourColor]

    # ---------- 选出最近的本队（面积最大） ----------
    output_data = None
    if team_detections:
        team_detections.sort(key=lambda d: d[5], reverse=True)
        best = team_detections[0]
        x1, y1, x2, y2, cls, _, _ = best
        roi = frame[y1:y2, x1:x2]
        angle = get_kfs_angle_by_edges(roi)
        #---------- 在原框位置显示边缘 ----------
        #     # 将边缘图（单通道）转为BGR，方便叠加
        # edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        #     # 把处理后的边缘区域替换回原图的对应位置
        # frame[y1:y2, x1:x2] = edges_colored
        output_data = (x1, y1, x2, y2, cls, angle)

    # ---------- 画所有框，标注序号和本队/干扰 ----------
    for idx, d in enumerate(detections, start=1):
        x1, y1, x2, y2, cls, _, _ = d
        color = (0, 255, 0) if cls == ourColor else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, str(idx), (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        label = "本队" if cls == ourColor else "干扰"
        cv2.putText(frame, label, (x1, y2+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    #  如果有本队目标，显示距离并输出数据 
    if output_data:
        x1, y1, x2, y2, cls, angle = output_data
        pixel_h = y2 - y1
        Z_disp = (REAL_H * fx_approx) / pixel_h  if pixel_h > 2 else 0 
        info = f"Z:{Z_disp:.2f}m "  # 角:{angle:.1f}° 姿态角
        cv2.putText(frame, info, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # 构造干扰位置字符串（只取第一个干扰的序号）
        if interference_positions:
            interference_str = str(interference_positions[0])   # 只输出第一个干扰的位置
        else:
            interference_str = '0'

        # 输出：x1 y1 x2 y2 cls angle interference_str（字符串）
        try:
            print(f"{x1} {y1} {x2} {y2} {ourColor} {angle:.1f} {interference_str}")
            sys.stdout.flush()
        except BrokenPipeError:
            sys.exit(0)

    # ---------- 显示画面 ----------
    cv2.namedWindow("KFS Detection", cv2.WINDOW_NORMAL)  # 拖拽边框
    cv2.imshow("KFS Detection", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
