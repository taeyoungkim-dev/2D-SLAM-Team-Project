"""reporter_logic 검사. ROS/YOLO 없이 실행: python3 robot_ws/scripts/obstacle_reporter/self_test.py"""
import unittest

import cv2
import numpy as np

from reporter_logic import (BOX_COLOR_BGR, Confirmer, Detection, annotate, encode_jpeg, encode_multipart, image_to_bgr,
                            is_clearly_visible, parse_zones, within_distance)

W, H = 640, 480


def det(cls='power_cable', conf=0.9, box=(100, 100, 200, 200)):
    return Detection(cls, conf, *box)


class VisibilityTests(unittest.TestCase):
    def check(self, d, **kw):
        args = dict(min_conf=0.6, border_px=4, min_area_ratio=0.005)
        args.update(kw)
        return is_clearly_visible(d, W, H, **args)

    def test_clear(self):
        self.assertTrue(self.check(det()))

    def test_low_confidence(self):
        self.assertFalse(self.check(det(conf=0.59)))

    def test_cut_by_border(self):
        for box in ((0, 100, 100, 200), (100, 0, 200, 100), (500, 100, W, 200), (100, 400, 200, H)):
            self.assertFalse(self.check(det(box=box)), box)

    def test_too_small(self):
        self.assertFalse(self.check(det(box=(100, 100, 110, 110))))

    def test_default_min_area_ratio(self):
        """기본값 0.03: 640x480 기준 60x60 상자는 너무 작고 120x120 상자는 충분히 크다."""
        self.assertFalse(self.check(det(box=(200, 200, 260, 260)), min_area_ratio=0.03))
        self.assertTrue(self.check(det(box=(200, 200, 320, 320)), min_area_ratio=0.03))


class DistanceTests(unittest.TestCase):
    def test_distance_to_obstacle_center(self):
        self.assertTrue(within_distance((0.0, 0.0), (0.25, 0.0), 0.25))
        self.assertTrue(within_distance((1.0, 1.0), (1.15, 1.2), 0.25))     # 0.25 (3-4-5)
        self.assertFalse(within_distance((0.0, 0.0), (0.2, 0.2), 0.25))     # 0.283
        self.assertFalse(within_distance((0.0, 0.0), (0.2501, 0.0), 0.25))


class ConfirmerTests(unittest.TestCase):
    def test_needs_consecutive_frames(self):
        c = Confirmer(3)
        self.assertEqual(c.update([det()]), {})
        self.assertEqual(c.update([det()]), {})
        self.assertEqual(list(c.update([det()])), ['power_cable'])
        self.assertEqual(list(c.update([det()])), ['power_cable'])      # 계속 보이면 계속 확정

    def test_one_missed_frame_resets(self):
        c = Confirmer(3)
        c.update([det()])
        c.update([det()])
        c.update([])                                                    # 한 프레임 안 보임
        self.assertEqual(c.update([det()]), {})
        self.assertEqual(c.update([det()]), {})
        self.assertEqual(list(c.update([det()])), ['power_cable'])

    def test_classes_are_independent_and_best_detection_kept(self):
        c = Confirmer(2)
        c.update([det('power_cable'), det('cardboard_box')])
        out = c.update([det('power_cable', 0.7), det('power_cable', 0.95), det('urine_puddle')])
        self.assertEqual(list(out), ['power_cable'])
        self.assertEqual(out['power_cable'].conf, 0.95)


class ImageTests(unittest.TestCase):
    def test_annotate_writes_class_top_left(self):
        frame = np.full((H, W, 3), 128, np.uint8)
        out = annotate(frame, 'cardboard_box')
        self.assertEqual(out.shape, frame.shape)
        self.assertTrue((frame == 128).all())                           # 원본은 그대로
        self.assertTrue((out[:3, :3] == 0).all())                         # 왼쪽 위 모서리의 글자 배경
        self.assertTrue((out[:60, :200] == 255).any())                  # 흰 글자
        self.assertTrue((out[H // 2:, :] == 128).all() and (out[:, W // 2:] == 128).all())   # box 없이 그리면 글자뿐

    def test_annotate_draws_bounding_box(self):
        frame = np.full((H, W, 3), 128, np.uint8)
        d = det(box=(300, 200, 500, 400))
        out = annotate(frame, 'power_cable', d)
        green = (out == np.array(BOX_COLOR_BGR, np.uint8)).all(axis=2)
        ys, xs = np.nonzero(green)
        self.assertTrue(len(ys) > 0)
        for got, want in zip((xs.min(), ys.min(), xs.max(), ys.max()), (300, 200, 500, 400)):
            self.assertLessEqual(abs(int(got) - want), 2)               # 상자 테두리(선 두께 만큼 번짐)
        self.assertTrue((out[250:350, 350:450] == 128).all())           # 상자 안쪽은 건드리지 않음
        self.assertTrue((out[:3, :3] == 0).all())                         # class 글자도 그대로 있음

    def test_jpeg_roundtrip(self):
        frame = np.full((H, W, 3), 90, np.uint8)
        decoded = cv2.imdecode(np.frombuffer(encode_jpeg(frame), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, frame.shape)

    def test_image_to_bgr(self):
        rgb = np.zeros((2, 3, 3), np.uint8)
        rgb[..., 0] = 200                                               # R
        padded = np.zeros((2, 3 * 3 + 5), np.uint8)                     # step 에 여백이 있는 경우
        padded[:, :9] = rgb.reshape(2, 9)
        bgr = image_to_bgr(padded.tobytes(), 'rgb8', 3, 2, 14)
        self.assertEqual(bgr.shape, (2, 3, 3))
        self.assertTrue((bgr[..., 2] == 200).all() and (bgr[..., :2] == 0).all())
        with self.assertRaises(ValueError):
            image_to_bgr(b'', '16UC1', 1, 1, 2)


class ProtocolTests(unittest.TestCase):
    def test_zones(self):
        status = '{"state": "x", "obstacle_zones": [{"cls": "a", "x": 1, "y": 2.5, "radius": 0.1}]}'
        self.assertEqual(parse_zones(status), {'a': (1.0, 2.5)})
        self.assertEqual(parse_zones('{"state": "x"}'), {})
        self.assertEqual(parse_zones('not json'), {})

    def test_multipart(self):
        body, ctype = encode_multipart(dict(type='power_cable', x=1.5, y=-2), 'f.jpg', b'JPEGDATA')
        boundary = ctype.split('boundary=')[1]
        self.assertIn(f'--{boundary}'.encode(), body)
        for part in (b'name="type"\r\n\r\npower_cable', b'name="x"\r\n\r\n1.5', b'name="y"\r\n\r\n-2',
                     b'name="image"; filename="f.jpg"', b'JPEGDATA'):
            self.assertIn(part, body)


if __name__ == '__main__':
    unittest.main(verbosity=2)
