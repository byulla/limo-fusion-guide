"""
LIMO 융합 서버 (S3) — YOLO 세그멘테이션 + 2D LiDAR 투영·매칭
기존 limo_server_nolib.py와 동일한 WebSocket/JSON 규격 유지 (앱 수정 불필요)
+ 추가 필드 "label": 감지된 객체 이름 (앱이 몰라도 무해)

핵심 차이: 라이다가 잡은 아무 점이나 알리지 않고,
YOLO 마스크 안에 떨어진 라이다 점이 있을 때만 장애물로 판정 (랜덤 TTS 방지)

방어 3층:
  1층 마스크 매칭  — YOLO가 인식한 물체 위의 라이다 점만 사용
  2층 연속 확인    — 같은 클래스가 CONFIRM_N회 연속 잡힐 때만 알림
  3층 전송 주기    — SEND_HZ로 제한 (기존과 동일)

실행 (launch + 카메라 launch가 켜진 상태에서):
  /usr/bin/python3 ~/fusion_server.py
"""
import base64
import hashlib
import json
import math
import socket
import struct
import threading
import time

import numpy as np
import cv2

viz = {"pub": None}

# ===== 판정 기준 (기존 서버와 동일, 앱 설정과 짝) =====
EMERGENCY_DIST = 0.5
WARN_DIST = 1.5
INFO_DIST = 3.0
SEND_HZ = 5.0

# ===== 융합 설정 =====
CONF_TH = 0.4          # YOLO 확신도 임계값
CONFIRM_N = 3          # 같은 클래스 연속 N회 확인 후 알림 (2층 방어)
MAX_RANGE = 5.0        # 라이다 유효 거리 상한 (m)
SECTOR_DEG = 60        # 좌우 ±60도 안의 물체만 대상 (기존 섹터 범위와 동일)

# ===== 캘리브레이션 (2026-08-06 확보, 고정 상수) =====
T_EXT = np.array([-0.030, 0.020, -0.000])
Q_EXT = np.array([0.501, -0.500, 0.498, 0.501])  # [x, y, z, w]
K_CAM = np.array([[489.2136535644531, 0.0, 318.94427490234375],
                  [0.0, 489.2136535644531, 205.9024658203125],
                  [0.0, 0.0, 1.0]])
IMG_W, IMG_H = 640, 480


def quat_to_rot(q):
    """쿼터니언 [x,y,z,w] -> 3x3 회전 행렬 (scipy 없이 표준 공식)"""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


R_EXT = quat_to_rot(Q_EXT)

# ===== 공유 상태 =====
latest = {"direction": "front", "distance": 99.9,
          "urgency": "safe", "label": ""}
lock = threading.Lock()
latest_img = {"data": None, "stamp": 0.0}
img_lock = threading.Lock()
latest_scan = {"msg": None}
scan_lock = threading.Lock()

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


# ---------- WebSocket 최소 구현 (기존 서버와 동일) ----------
def ws_handshake(conn) -> bool:
    try:
        request = b""
        while b"\r\n\r\n" not in request:
            chunk = conn.recv(1024)
            if not chunk:
                return False
            request += chunk
        key = None
        for line in request.decode(errors="ignore").split("\r\n"):
            if line.lower().startswith("sec-websocket-key:"):
                key = line.split(":", 1)[1].strip()
        if not key:
            return False
        accept = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        conn.send((
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
        return True
    except OSError:
        return False


def ws_send_text(conn, text: str):
    payload = text.encode()
    n = len(payload)
    if n < 126:
        header = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x81, 126, n)
    else:
        header = struct.pack("!BBQ", 0x81, 127, n)
    conn.sendall(header + payload)


def drain_incoming(conn):
    try:
        conn.settimeout(0.001)
        while True:
            data = conn.recv(4096)
            if not data:
                raise OSError("closed")
    except socket.timeout:
        pass
    finally:
        conn.settimeout(None)


def make_payload() -> str:
    with lock:
        return json.dumps({
            "type": "obstacle" if latest["urgency"] != "safe" else "none",
            "direction": latest["direction"],
            "distance": latest["distance"],
            "urgency": latest["urgency"],
            "label": latest["label"],
        })


def client_thread(conn, addr):
    print(f"앱 연결됨: {addr}")
    if not ws_handshake(conn):
        conn.close()
        return
    try:
        while True:
            drain_incoming(conn)
            ws_send_text(conn, make_payload())
            time.sleep(1.0 / SEND_HZ)
    except OSError:
        print(f"앱 연결 종료: {addr}")
    finally:
        conn.close()


def ws_server(host="0.0.0.0", port=8765):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(2)
    print(f"LIMO 융합 서버 시작: ws://{host}:{port}  (종료: Ctrl+C)")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=client_thread, args=(conn, addr),
                         daemon=True).start()


# ---------- ROS 콜백: 최신 데이터만 보관 ----------
def image_callback(msg):
    with img_lock:
        latest_img["data"] = msg
        latest_img["stamp"] = time.time()


def scan_callback(msg):
    with scan_lock:
        latest_scan["msg"] = msg


def msg_to_numpy(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]  # RGB -> BGR (YOLO는 BGR 넘겨도 무방하나 통일)
    return np.ascontiguousarray(img)


def project_scan(scan):
    """LaserScan -> (픽셀 u좌표 배열, 각도 배열(deg), 거리 배열)"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    rr = np.array(scan.ranges, dtype=np.float32)
    ok = np.isfinite(rr) & (rr > max(scan.range_min, 0.05)) & (rr < MAX_RANGE)
    ang, rng = angles[ok], rr[ok]
    # 좌우 ±SECTOR_DEG 밖은 제외 (기존 섹터 범위와 동일한 관심 영역)
    deg = np.degrees(np.arctan2(np.sin(ang), np.cos(ang)))
    sec = np.abs(deg) <= SECTOR_DEG
    ang, rng, deg = ang[sec], rng[sec], deg[sec]
    pts = np.stack([rng * np.cos(ang), rng * np.sin(ang),
                    np.zeros_like(rng)], axis=1)
    cam = (R_EXT @ pts.T).T + T_EXT
    front = cam[:, 2] > 0.05
    cam, deg, rng = cam[front], deg[front], rng[front]
    uv = (K_CAM @ cam.T).T
    u = uv[:, 0] / uv[:, 2] + 75.0  # 수평 정렬 오프셋 보정 (장착 오차 관찰 기반)
    v = uv[:, 1] / uv[:, 2]
    return u, v, deg, rng


def direction_from_deg(a_deg):
    """기존 서버의 섹터 정의와 동일: +20~+60 좌 / -20~+20 정면 / -60~-20 우"""
    if a_deg > 20:
        return "left"
    if a_deg < -20:
        return "right"
    return "front"


def urgency_from_dist(d):
    if d <= EMERGENCY_DIST:
        return "emergency"
    if d <= WARN_DIST:
        return "warn"
    if d <= INFO_DIST:
        return "info"
    return "safe"


# ---------- 융합 루프 ----------
def fusion_loop(model):
    from sensor_msgs.msg import Image
    history = []          # 최근 CONFIRM_N회의 감지 클래스 (2층 방어)
    frame_cnt, t0 = 0, time.time()
    while True:
        with img_lock:
            img_msg = latest_img["data"]
        with scan_lock:
            scan = latest_scan["msg"]
        if img_msg is None or scan is None:
            time.sleep(0.05)
            continue

        img = msg_to_numpy(img_msg)
        res = model(img, conf=CONF_TH, verbose=False)[0]
        u, v, deg, rng = project_scan(scan)

        best = None  # (dist, label, a_med)
        if res.masks is not None and len(res.boxes) > 0 and len(u) > 0:
            for i in range(len(res.boxes)):
                label = model.names[int(res.boxes.cls[i])]
                mask = res.masks.data[i].cpu().numpy()
                if mask.shape != (IMG_H, IMG_W):
                    ys = np.linspace(0, mask.shape[0] - 1, IMG_H).astype(int)
                    xs = np.linspace(0, mask.shape[1] - 1, IMG_W).astype(int)
                    mask = mask[np.ix_(ys, xs)]
                cols = np.where((mask > 0.5).any(axis=0))[0]
                if len(cols) == 0:
                    continue
                inx = (u >= cols.min()) & (u <= cols.max()) \
                      & (u >= 0) & (u < IMG_W)
                if inx.sum() == 0:
                    continue
                # 배경 점 오염 제거: 매칭 점들 중 가까운 절반만 사용
                sel = rng[inx]
                sel = np.sort(sel)[: max(3, len(sel) // 2)]
                d_med = float(np.median(sel))
                a_med = float(np.median(deg[inx]))
                if best is None or d_med < best[0]:
                    best = (d_med, label, a_med)

        # ----- 시각화 영상 발행 (rqt_image_view용) -----
        if viz["pub"] is not None and viz["pub"].get_num_connections() > 0:
            canvas = img.copy()
            if res.boxes is not None:
                for i in range(len(res.boxes)):
                    x1, y1, x2, y2 = map(int, res.boxes.xyxy[i].tolist())
                    name = model.names[int(res.boxes.cls[i])]
                    cf = float(res.boxes.conf[i])
                    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 200, 0), 2)
                    cv2.putText(canvas, f"{name} {cf:.2f}", (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            for (uu, dd) in zip(u, rng):
                col = int(np.clip((dd - 0.5) / 2.5, 0, 1) * 255)
                cv2.circle(canvas, (int(uu), IMG_H // 2), 3, (0, col, 255 - col), -1)
            if best is not None:
                cv2.putText(canvas, f"{best[1]} {best[0]:.2f}m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            out_msg = Image()
            out_msg.height, out_msg.width = IMG_H, IMG_W
            out_msg.encoding = "bgr8"
            out_msg.step = IMG_W * 3
            out_msg.data = canvas.tobytes()
            viz["pub"].publish(out_msg)

        # ----- 2층 방어: 연속 확인 -----
        history.append(best[1] if best else None)
        if len(history) > CONFIRM_N:
            history.pop(0)
        confirmed = (best is not None
                     and len(history) == CONFIRM_N
                     and all(h == best[1] for h in history))

        with lock:
            if confirmed:
                latest["direction"] = direction_from_deg(best[2])
                latest["distance"] = round(best[0], 2)
                latest["urgency"] = urgency_from_dist(best[0])
                latest["label"] = best[1]
            else:
                latest["direction"] = "front"
                latest["distance"] = 99.9
                latest["urgency"] = "safe"
                latest["label"] = ""

        # ----- FPS 로그 (5초마다) -----
        frame_cnt += 1
        now = time.time()
        if now - t0 >= 5.0:
            fps = frame_cnt / (now - t0)
            state = (f"{latest['label']} {latest['distance']}m "
                     f"{latest['direction']}" if confirmed else "감지 없음")
            print(f"[융합 FPS {fps:.1f}] {state}")
            frame_cnt, t0 = 0, now


def main():
    import rospy
    from sensor_msgs.msg import LaserScan, Image
    from ultralytics import YOLO

    print("YOLO 모델 로드 중...")
    model = YOLO("yolov8n-seg.pt")
    print("모델 로드 완료")

    rospy.init_node("limo_fusion_server", disable_signals=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback,
                     queue_size=1, buff_size=2 ** 24)
    # 시각화: 박스·라이다 점이 그려진 영상을 발행 (rqt_image_view에서 /fusion/annotated 선택)
    viz["pub"] = rospy.Publisher("/fusion/annotated", Image, queue_size=1)
    print("/scan + /camera/color/image_raw 구독 시작")
    print("시각화 토픽: /fusion/annotated (rqt_image_view에서 선택)")

    threading.Thread(target=fusion_loop, args=(model,), daemon=True).start()
    try:
        ws_server()
    except KeyboardInterrupt:
        print("\n종료")


if __name__ == "__main__":
    main()
