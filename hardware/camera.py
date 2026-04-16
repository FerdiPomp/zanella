import numpy as np
import cv2
import threading
import time

from sklearn.linear_model import LinearRegression, RANSACRegressor

import config as CONFIG
from engine.runtime_utils import print_log, save_jpg_artifact, save_numpy_artifact


def _estimate_plane(points):
    assert points.ndim == 2 and points.shape[1] == 3

    x_coordinates = points[:, :2]
    z_coordinates = points[:, 2]

    ransac = RANSACRegressor(
        estimator=LinearRegression(),
        residual_threshold=CONFIG.PLANE_THRESHOLD,
        min_samples=5000,
        max_trials=100,
    )
    try:
        ransac.fit(x_coordinates, z_coordinates)
        a_xy = ransac.estimator_.coef_
        c_z = ransac.estimator_.intercept_
    except Exception:
        return None

    return np.array([a_xy[0], a_xy[1], -1.0, c_z], dtype=np.float64)


def _point_plane_distance(points, plane):
    a, b, c, d = plane
    num = a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d
    den = np.sqrt(a * a + b * b + c * c)
    return num / den


def _build_height_map(points, roi_mask, image_shape, plane):
    roi_points = points[roi_mask].reshape(-1, 3)
    valid = np.isfinite(roi_points[:, 2])
    roi_points = roi_points[valid]

    distances = _point_plane_distance(roi_points, plane)
    height_map = np.zeros(image_shape, dtype=np.float32)
    roi_indices = np.argwhere(roi_mask)

    for (y, x), height in zip(roi_indices, distances):
        if height > height_map[y, x]:
            height_map[y, x] = height

    return height_map


def _build_aruco_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def _decode_text(code):
    return code.data.decode("utf-8", errors="replace")


def _save_debug_artifact(data, file_name: str, dir_name: str):
    if isinstance(data, np.ndarray) and data.ndim == 3:
        return save_jpg_artifact(data, dir_name, file_name)
    return save_numpy_artifact(data, dir_name, file_name)


class SharedQRState:
    def __init__(self):
        self._lock = threading.Lock()
        self._qr = None
        self._timestamp = time.time()
        self._prev_qr = []
        self._prev_timestamp = []
        self._timeout = 100

    def _remove_old(self, now):
        while (now - self._prev_timestamp[0] > self._timeout) and (len(self._prev_timestamp) > 1):
            self._prev_timestamp.pop(0)
            self._prev_qr.pop(0)

    def update(self, qr_value, timestamp):
        with self._lock:
            self._prev_qr.append(self._qr)
            self._prev_timestamp.append(self._timestamp)
            self._remove_old(timestamp)
            self._qr = qr_value
            self._timestamp = timestamp

    def get(self):
        with self._lock:
            return self._qr, self._timestamp

    def get_prev(self):
        with self.lock:
            return self._prev_qr, self._prev_timestamp


class RealSenseCamera:
    def __init__(self, rgb: bool, file_bag: str = None):
        import pyrealsense2 as rs

        self.rs = rs
        self.ROI = [0, 0, -1, -1]
        self.pipeline = self.rs.pipeline()
        self.config = self.rs.config()

        if file_bag is not None:
            self.config.enable_device_from_file(file_bag, repeat_playback=False)
        else:
            self.config.enable_stream(self.rs.stream.depth, 1280, 720, self.rs.format.z16, 5)
            if rgb:
                self.config.enable_stream(self.rs.stream.color, 1280, 720, self.rs.format.bgr8, 5)

        profile = self.pipeline.start(self.config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.pc = self.rs.pointcloud()
        self.plane = None
        self.roi_mask = None
        self.h = None
        self.w = None

    def __del__(self):
        if getattr(self, "pipeline", None) is not None:
            self.pipeline.stop()

    def _wait_for_frames(self):
        return self.pipeline.wait_for_frames()

    def _warmup_depth_frame(self):
        depth_frame = None
        for _ in range(10):
            frames = self._wait_for_frames()
            depth_frame = frames.get_depth_frame()
        return depth_frame

    def _build_roi_mask(self):
        x0, y0, x1, y1 = self.ROI
        self.roi_mask = np.zeros((self.h, self.w), dtype=bool)
        self.roi_mask[y0:y1, x0:x1] = True

    def _depth_to_points(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32)
        return verts.reshape(-1, 3)

    def _calibrate_plane_from_depth_frame(self, depth_frame):
        depth = np.asanyarray(depth_frame.get_data())
        self.h, self.w = depth.shape
        self._build_roi_mask()

        points = self._depth_to_points(depth_frame).reshape(self.h, self.w, 3)
        roi_points = points[self.roi_mask]
        roi_points = roi_points[np.isfinite(roi_points[:, 2])]
        roi_points = roi_points[roi_points[:, 2] > 0.3]
        self.plane = _estimate_plane(roi_points)
        print_log(f"Piano stimato: {self.plane}")

    def table_calibration(self):
        depth_frame = self._warmup_depth_frame()

        if depth_frame is None:
            frames = self._wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                print_log("No depth frame")
                return None

        self._calibrate_plane_from_depth_frame(depth_frame)

    def _build_height_map_from_depth_frame(self, depth_frame):
        points = self._depth_to_points(depth_frame).reshape(self.h, self.w, 3)
        return _build_height_map(points, self.roi_mask, (self.h, self.w), self.plane)


class ObjCamera(RealSenseCamera):
    def __init__(self, is_enter_node: bool, file_bag: str = None):
        super().__init__(CONFIG.DEBUGGING, file_bag)
        self.ROI = CONFIG.ROI_A if is_enter_node else CONFIG.ROI_B
        self.detect_queue = []
        self.obj_find = False
        self.expected_hu_set = np.load("expected_shape/expected_hu.npy")
        time.sleep(3)
        self.table_calibration()

    def __overlap(self, mask1, mask2):
        inter = np.logical_and(mask1, mask2)
        return np.sum(inter) / max(np.sum(mask1), 1)

    def __there_is_obj(self):
        for index in range(len(self.detect_queue) - 1):
            if not (self.__overlap(self.detect_queue[index], self.detect_queue[index + 1]) > 0.70):
                return False
        return True

    def __shape_validation(self, obj_mask):
        num_labels, labels = cv2.connectedComponents(obj_mask.astype(np.uint8))
        for label in range(1, num_labels):
            component = labels == label
            area = np.sum(component)
            if area < CONFIG.MIN_AREA_PIXELS:
                continue

            mask_uint8 = component.astype(np.uint8) * 255
            moments = cv2.moments(mask_uint8)
            hu = cv2.HuMoments(moments)
            hu = (np.sign(hu) * np.log10(np.abs(hu) + 1e-12)).flatten()
            distances = np.linalg.norm(self.expected_hu_set - hu, axis=1)
            if np.min(distances) < CONFIG.SHAPE_THRESHOLD:
                return True
        return False

    def find_object(self):
        try:
            frames = self._wait_for_frames()
        except RuntimeError as error:
            print_log(f"Frame timeout, restart pipeline: {error}")
            try:
                self.pipeline.stop()
            except Exception:
                pass
            time.sleep(1)
            self.pipeline.start(self.config)
            time.sleep(2)
            frames = self._wait_for_frames()

        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        color_frame = frames.get_color_frame() if CONFIG.DEBUGGING else None
        depth = np.asanyarray(depth_frame.get_data())
        height_map = self._build_height_map_from_depth_frame(depth_frame)
        object_mask = (height_map > CONFIG.MIN_HEIGHT_THRESHOLD) & (height_map < CONFIG.MAX_HEIGHT_THRESHOLD)
        object_detected = np.sum(object_mask) > CONFIG.MIN_AREA_PIXELS
        shape_ok = self.__shape_validation(object_mask)

        if object_detected:
            self.detect_queue.append(object_mask)
            if len(self.detect_queue) > CONFIG.MAX_LEN:
                self.detect_queue.pop(0)
                if not self.obj_find:
                    self.obj_find = self.__there_is_obj() & shape_ok
                if CONFIG.DEBUGGING:
                    _save_debug_artifact(depth, "_not_right_shape", "debug_shape")
                    if color_frame:
                        color = np.asanyarray(color_frame.get_data())
                        _save_debug_artifact(color, "_shape", "debug_shape_img")
        else:
            if self.obj_find:
                self.obj_find = False
            self.detect_queue = []

        return self.obj_find, depth, object_mask


class BaseQrReader:
    def _read_aruco(self, img):
        detector = _build_aruco_detector()
        _, ids, _ = detector.detectMarkers(img)
        if ids is not None:
            return 0 in ids
        return False

    def _decode_codes(self, gray, timeout):
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, 5)
        return self.decode(bw, timeout=timeout, max_count=2)


class QrCamera(RealSenseCamera, BaseQrReader):
    def __init__(self, shared_qr_state: SharedQRState, file_bag: str = None):
        from pylibdmtx.pylibdmtx import decode

        self.decode = decode
        super().__init__(True, file_bag)
        self.ROI = CONFIG.ROI_QR
        self.shared_qr_state = shared_qr_state
        self._last_qr = None
        self.table_calibration()

    def find_occlusion(self):
        depth_frame = self._wait_for_frames().get_depth_frame()
        if not depth_frame:
            return None

        height_map = self._build_height_map_from_depth_frame(depth_frame)
        object_mask = height_map > CONFIG.MIN_HEIGHT_THRESHOLD
        return np.sum(object_mask) > CONFIG.OCCLUSION_THRESHOLD

    def _get_roi_image(self, img):
        x0, y0, x1, y1 = CONFIG.ROI_QR
        return img[y0:y1, x0:x1], (x0, y0, x1, y1)

    def read_qr(self):
        occlusion = self.find_occlusion()
        color_frame = self._wait_for_frames().get_color_frame()
        if not color_frame:
            return None, occlusion, None

        img = np.asanyarray(color_frame.get_data())
        img_roi, (x0, y0, x1, y1) = self._get_roi_image(img)
        gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)

        not_aruco = True
        if CONFIG.ARUCO_MODE and self._read_aruco(gray):
            not_aruco = False

        codes = self._decode_codes(gray, timeout=150)
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 0), 2)

        if codes and len(codes) == 1:
            x, y, _, _ = codes[0].rect
            cv2.putText(img, _decode_text(codes[0]), (x + x0, y + y0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            return _decode_text(codes[0]), occlusion, img
        if codes and len(codes) > 1:
            for code in codes:
                x, y, _, _ = code.rect
                cv2.putText(img, _decode_text(code), (x + x0, y + y0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            return None, True, img

        return None, occlusion or not_aruco, None


class ZEDQrCamera(BaseQrReader):
    def __init__(self, shared_qr_state: SharedQRState, file_bag: str = None):
        import pyzed.sl as sl
        from pylibdmtx.pylibdmtx import decode

        self.sl = sl
        self.decode = decode
        self.shared_qr_state = shared_qr_state
        self._last_qr = None
        self.file_bag = file_bag
        self.zed = None
        self.runtime_params = self.sl.RuntimeParameters()
        print_log("First table calibration")
        self.plane = None
        self.roi_mask = None
        self.h = None
        self.w = None
        time.sleep(1)
        self._open_camera()
        self.table_calibration()

    def __del__(self):
        if getattr(self, "zed", None) is not None:
            self.zed.close()

    def _build_init_params(self):
        init_params = self.sl.InitParameters()
        if self.file_bag is not None:
            init_params.set_from_svo_file(self.file_bag)
        else:
            init_params.camera_resolution = self.sl.RESOLUTION.HD2K
            init_params.depth_mode = self.sl.DEPTH_MODE.NEURAL
            init_params.coordinate_units = self.sl.UNIT.METER
            init_params.sdk_verbose = 1
        return init_params

    def _apply_live_settings(self):
        if self.file_bag is None:
            self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.EXPOSURE, 60)
            self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.GAIN, 20)
            self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.SHARPNESS, 0)
            self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.CONTRAST, 4)

    def _open_camera(self):
        if self.zed is not None:
            try:
                self.zed.close()
            except Exception:
                pass

        self.zed = self.sl.Camera()
        err = self.zed.open(self._build_init_params())
        if err > self.sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failure in opening zed")
        self._apply_live_settings()

    def _grab_frame(self):
        last_error = None
        for attempt in range(CONFIG.ZED_GRAB_RETRY_COUNT):
            error_code = self.zed.grab(self.runtime_params)
            if error_code == self.sl.ERROR_CODE.SUCCESS:
                if attempt > 0:
                    print_log("ZED grab recovered")
                return
            last_error = error_code
            time.sleep(CONFIG.ZED_GRAB_RETRY_DELAY)

        print_log(f"ZED grab failed repeatedly ({last_error}), reopening camera")
        self._open_camera()
        error_code = self.zed.grab(self.runtime_params)
        if error_code != self.sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED grab unrecoverable ({error_code})")

    def _grab_point_cloud(self):
        self._grab_frame()
        point_cloud = self.sl.Mat()
        self.zed.retrieve_measure(point_cloud, self.sl.MEASURE.XYZ)
        return point_cloud

    def _build_roi_mask(self):
        x0, y0, x1, y1 = CONFIG.ROI_QR
        self.roi_mask = np.zeros((self.h, self.w), dtype=bool)
        self.roi_mask[y0:y1, x0:x1] = True

    def _calibrate_plane_from_point_cloud(self, point_cloud):
        depth = np.asanyarray(point_cloud.get_data())[:, :, :3]
        self.h, self.w, _ = depth.shape
        self._build_roi_mask()

        roi_points = depth[self.roi_mask]
        roi_points = roi_points[np.isfinite(roi_points[:, 2])]
        self.plane = _estimate_plane(roi_points)
        print_log(f"Piano stimato: {self.plane}")

    def table_calibration(self):
        point_cloud = self._grab_point_cloud()
        self._calibrate_plane_from_point_cloud(point_cloud)

    def _build_height_map_from_points(self, points):
        return _build_height_map(points, self.roi_mask, (self.h, self.w), self.plane)

    def find_occlusion(self):
        point_cloud = self._grab_point_cloud()
        points = np.asanyarray(point_cloud.get_data())[:, :, :3]
        height_map = self._build_height_map_from_points(points)
        object_mask = height_map > CONFIG.MIN_HEIGHT_THRESHOLD
        return np.sum(object_mask) > CONFIG.OCCLUSION_THRESHOLD

    def _get_roi_image(self, img):
        x0, y0, x1, y1 = CONFIG.ROI_QR
        return img[y0:y1, x0:x1], (x0, y0, x1, y1)

    def read_qr(self):
        self._grab_frame()
        point_cloud = self.sl.Mat()
        image = self.sl.Mat()
        self.zed.retrieve_measure(point_cloud, self.sl.MEASURE.XYZ)
        self.zed.retrieve_image(image, self.sl.VIEW.LEFT)

        points = np.asanyarray(point_cloud.get_data())[:, :, :3]
        height_map = self._build_height_map_from_points(points)
        object_mask = height_map > CONFIG.MIN_HEIGHT_THRESHOLD
        occlusion = np.sum(object_mask) > CONFIG.OCCLUSION_THRESHOLD

        img = np.asanyarray(image.get_data())
        img_roi, _ = self._get_roi_image(img)
        gray = cv2.cvtColor(img_roi, cv2.COLOR_RGBA2GRAY)

        not_aruco = False
        if CONFIG.ARUCO_MODE and self._read_aruco(gray):
            not_aruco = False

        codes = self._decode_codes(gray, timeout=200)
        if codes and len(codes) == 1:
            return _decode_text(codes[0]), occlusion, img
        if codes and len(codes) > 1:
            return None, True, img
        return None, occlusion or not_aruco, None
