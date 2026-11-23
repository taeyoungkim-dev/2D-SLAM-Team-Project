"""장애물 발견 보고의 순수 로직 (ROS / YOLO 없이 numpy + OpenCV 만 사용, self_test.py 로 검사).

- ``Detection``            YOLO 가 낸 한 상자(class, 신뢰도, 픽셀 좌표)
- ``is_clearly_visible``   "확실히 보인다" 판정 (신뢰도, 화면 안에 온전히, 충분히 큼)
- ``within_distance``      로봇과 장애물 중심 사이의 거리 조건
- ``Confirmer``            같은 class 가 연속 N 번 확실히 보이면 "발견"으로 확정
- ``annotate``             bounding box 를 그리고 사진 왼쪽 위에 class 를 적은 보고용 이미지
- ``encode_multipart``     서버 ``POST /api/obstacles`` 용 multipart 본문
- ``parse_zones``          /coverage_status 의 obstacle_zones -> {class: (x, y)} (map 좌표)
"""
from __future__ import annotations

import json
import math
import uuid
from dataclasses import dataclass

import cv2
import numpy as np

BOX_COLOR_BGR = (0, 255, 0)  # bounding box 색(초록)


@dataclass(frozen=True)
class Detection:
    cls: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float


def is_clearly_visible(det: Detection, width: int, height: int, min_conf: float,
                       border_px: float, min_area_ratio: float) -> bool:
    """신뢰도가 충분하고, 상자가 화면 가장자리에 걸려 잘리지 않았고(장애물 전체가 보임), 너무 작지 않으면 True."""
    if det.conf < min_conf:
        return False
    if det.x1 < border_px or det.y1 < border_px or det.x2 > width - border_px or det.y2 > height - border_px:
        return False
    return (det.x2 - det.x1) * (det.y2 - det.y1) >= min_area_ratio * width * height


def within_distance(robot_xy: tuple[float, float], obstacle_xy: tuple[float, float], max_distance: float) -> bool:
    """로봇(base)과 장애물 중심 사이의 거리가 max_distance(m) 이하이면 True. 같은 map 좌표여야 한다."""
    return math.hypot(robot_xy[0] - obstacle_xy[0], robot_xy[1] - obstacle_xy[1]) <= max_distance


class Confirmer:
    """프레임마다 update() 를 부르면, class 별로 "연속 min_frames 번 확실히 보였는지"를 판정한다.

    한 프레임이라도 안 보이면 그 class 의 연속 횟수는 0 으로 돌아간다(오탐 한 프레임으로 보고하지 않기 위함).
    확정(연속 횟수가 min_frames 에 도달)된 class 와 그때의 가장 좋은 Detection 을 돌려준다.
    """

    def __init__(self, min_frames: int):
        self.min_frames = max(1, int(min_frames))
        self.streak: dict[str, int] = {}

    def update(self, visible: list[Detection]) -> dict[str, Detection]:
        best: dict[str, Detection] = {}
        for det in visible:
            if det.cls not in best or det.conf > best[det.cls].conf:
                best[det.cls] = det
        self.streak = {cls: self.streak.get(cls, 0) + 1 for cls in best}
        return {cls: det for cls, det in best.items() if self.streak[cls] >= self.min_frames}


def annotate(frame_bgr: np.ndarray, cls: str, det: Detection | None = None) -> np.ndarray:
    """보고용 이미지: 장애물에 bounding box(det)를 그리고, 사진의 왼쪽 위 모서리에 class 이름을 적는다."""
    image = frame_bgr.copy()
    h, w = image.shape[:2]
    scale = max(0.5, min(w, h) / 480.0 * 0.8)
    thickness = max(1, round(scale * 2))
    if det is not None:
        top_left = (max(0, round(det.x1)), max(0, round(det.y1)))
        bottom_right = (min(w - 1, round(det.x2)), min(h - 1, round(det.y2)))
        cv2.rectangle(image, top_left, bottom_right, BOX_COLOR_BGR, thickness, cv2.LINE_AA)
    (tw, th), baseline = cv2.getTextSize(cls, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    pad = max(4, round(scale * 6))
    cv2.rectangle(image, (0, 0), (tw + 2 * pad, th + baseline + 2 * pad), (0, 0, 0), -1)
    cv2.putText(image, cls, (pad, th + pad), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255),
                thickness, cv2.LINE_AA)
    return image


def encode_jpeg(image_bgr: np.ndarray, quality: int = 90) -> bytes:
    ok, buffer = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG 인코딩 실패')
    return buffer.tobytes()


def encode_multipart(fields: dict, filename: str, content: bytes, content_type: str = 'image/jpeg'
                     ) -> tuple[bytes, str]:
    """(본문, Content-Type 헤더 값). 서버의 ``image`` 파일 필드 + form 필드."""
    boundary = uuid.uuid4().hex
    parts = [f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()
             for name, value in fields.items()]
    parts.append((f'--{boundary}\r\nContent-Disposition: form-data; name="image"; filename="{filename}"\r\n'
                  f'Content-Type: {content_type}\r\n\r\n').encode() + content + f'\r\n--{boundary}--\r\n'.encode())
    return b''.join(parts), f'multipart/form-data; boundary={boundary}'


def parse_zones(status_json: str) -> dict[str, tuple[float, float]]:
    """/coverage_status 문자열에서 {class: (x, y)} 를 꺼낸다. 잘못된 입력이면 빈 dict."""
    try:
        zones = json.loads(status_json).get('obstacle_zones', [])
        return {str(z['cls']): (float(z['x']), float(z['y'])) for z in zones}
    except (ValueError, TypeError, KeyError, AttributeError):
        return {}


def image_to_bgr(data: bytes, encoding: str, width: int, height: int, step: int) -> np.ndarray:
    """sensor_msgs/Image 데이터를 BGR 배열로 (cv_bridge 없이). 지원: rgb8, bgr8, rgba8, bgra8, mono8."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'지원하지 않는 이미지 encoding: {encoding}')
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :width * channels]
    image = rows.reshape(height, width, channels)
    if encoding == 'rgb8':
        return np.ascontiguousarray(image[:, :, ::-1])
    if encoding == 'rgba8':
        return np.ascontiguousarray(image[:, :, 2::-1])
    if encoding == 'bgra8':
        return np.ascontiguousarray(image[:, :, :3])
    if encoding == 'mono8':
        return cv2.cvtColor(np.ascontiguousarray(image[:, :, 0]), cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(image)
