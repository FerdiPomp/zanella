import pyrealsense2 as rs
import pyzed.sl as sl
import numpy as np
import cv2
import time

from pylibdmtx.pylibdmtx import decode
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
import threading

import config as CONFIG

class ObjCamera:
    def __init__(self, is_enter_node:bool,file_bag : str = None):
        if is_enter_node:
            self.ROI = CONFIG.ROI_A
        else:
            self.ROI = CONFIG.ROI_B

        self.pipeline = None
        self.pipeline = rs.pipeline()
        config = rs.config()

        if file_bag is not None:
            config.enable_device_from_file(file_bag,  repeat_playback=False)
        else:
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        profile = self.pipeline.start(config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        self.pc = rs.pointcloud()
        time.sleep(1)
        print("First table calibration")
        self.plane = None
        self.roi_mask = None
        self.h = None
        self.w = None
        self.table_calibration()
        self.detect_queue = []
        self.obj_find = False

        self.expected_hu_set = np.load("expected_shape/expected_hu.npy")

    def __del__(self):
        if self.pipeline is not None:
            self.pipeline.stop()

    def table_calibration(self, depth_frame=None):
        if depth_frame is None:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                print("No depth frame")
                return None

        depth = np.asanyarray(depth_frame.get_data())
        self.h, self.w = depth.shape
        x0, y0, x1, y1 = self.ROI

        self.roi_mask = np.zeros((self.h, self.w), dtype=bool)
        self.roi_mask[y0:y1, x0:x1] = True

        points = self.__depth_to_points(depth_frame)
        points = points.reshape(self.h, self.w, 3)

        roi_points = points[self.roi_mask]
        roi_points = roi_points[np.isfinite(roi_points[:,2])]

        #TODO: check for not None output
        self.plane = self.__estimate_plane(roi_points)
        print("Piano stimato:", self.plane)

    #TODO: magari trovare un modo più sofisticato di fare sta cosa...
    def __estimate_plane(self, points):
        assert points.ndim == 2 and points.shape[1] == 3

        X = points[:, :2]   # x, y
        y = points[:, 2]    # z

        ransac = RANSACRegressor(
            estimator=LinearRegression(),
            residual_threshold=CONFIG.PLANE_THRESHOLD,
            min_samples=5000,
            max_trials=100
        )
        try:
            ransac.fit(X, y)

            # Piano: z = ax + by + c
            a_xy = ransac.estimator_.coef_
            c_z = ransac.estimator_.intercept_
        except:
            return None

        return np.array([a_xy[0], a_xy[1], -1.0, c_z], dtype=np.float64)

    def __depth_to_points(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32)
        return verts.reshape(-1, 3)

    def __point_plane_distance(self, points, plane = None):
        if plane is None:
            plane = self.plane
        a, b, c, d = plane
        num = a*points[:,0] + b*points[:,1] + c*points[:,2] + d
        den = np.sqrt(a*a + b*b + c*c)
        return num / den

    def __overlap(slef, mask1, mask2):
        inter = np.logical_and(mask1, mask2)
        return np.sum(inter) / max(np.sum(mask1), 1)

    def __there_is_obj(self):
        obj_list = self.detect_queue
        for i in range(len(obj_list)-1):
            if not (self.__overlap(obj_list[i], obj_list[i+1]) >0.90):
                return False
        return True

    def __shape_validation(self, obj_mask):
        num_labels, labels = cv2.connectedComponents(obj_mask.astype(np.uint8))
        for label in range(1, num_labels):
            component = (labels == label)
            area = np.sum(component)
            if area < CONFIG.MIN_AREA_PIXELS:
                continue

            mask_uint8 = (component.astype(np.uint8)) * 255
            moments = cv2.moments(mask_uint8)
            hu = cv2.HuMoments(moments)
            hu = (np.sign(hu) * np.log10(np.abs(hu) + 1e-12)).flatten()
            distances = np.linalg.norm(self.expected_hu_set - hu, axis=1)
            min_distance = np.min(distances)
            if min_distance < CONFIG.SHAPE_THRESHOLD:
                return True
        return False

    def find_object(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        depth = np.asanyarray(depth_frame.get_data())
        
        points = self.__depth_to_points(depth_frame).reshape(self.h, self.w, 3)

        roi_points = points[self.roi_mask].reshape(-1, 3)
        valid = np.isfinite(roi_points[:,2])
        roi_points = roi_points[valid]

        #Table re-calibration
        # new_plane = self.__estimate_plane(roi_points)
        # if new_plane is not None:
        #     new_distances = self.__point_plane_distance(roi_points, new_plane)
        #     std_height = np.std(new_distances)
        #     if std_height < CONFIG.THRESH_STD:
        #         self.plane = new_plane

        distances = self.__point_plane_distance(roi_points)

        height_map = np.zeros((self.h, self.w), dtype=np.float32)
        roi_indices = np.argwhere(self.roi_mask)
        roi_heights = distances
        
        for (y, x), hgt in zip(roi_indices, roi_heights):
            if hgt > height_map[y, x]:
                height_map[y, x] = hgt

        object_mask = (height_map > CONFIG.MIN_HEIGHT_THRESHOLD) & (height_map < CONFIG.MAX_HEIGHT_THRESHOLD)
        object_detected = np.sum(object_mask) > CONFIG.MIN_AREA_PIXELS
        shape_ok = self.__shape_validation(object_mask)
        
        if object_detected:
            self.detect_queue.append(object_mask)
            if len(self.detect_queue)>CONFIG.MAX_LEN:
                self.detect_queue.pop(0)
                if not self.obj_find:
                    self.obj_find = self.__there_is_obj() & shape_ok
        else:
            if self.obj_find:
                self.obj_find = False   
            self.detect_queue = []


        return self.obj_find, depth, object_mask

#TODO: rimpiazzare questa classe con una normale queue
class SharedQRState:
    def __init__(self):
        self._lock = threading.Lock()
        self._qr = None
        self._timestamp = time.time()
        self._prev_qr = []
        self._prev_timestamp = []
        self._timeout = 100

    def _remove_old(self, now):
        for i in range(len(self._prev_timestamp)):
            if now-self._prev_timestamp[i] < self._timeout:
                break
            self._prev_timestamp.pop(i)
            self._prev_qr.pop(i)

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

#TODO: ricontrollare logica occlusioni + vedere se fondere camere in classi e sottoclassi
class QrCamera:
    def __init__(self, shared_qr_state:SharedQRState, file_bag : str = None):
        self.shared_qr_state = shared_qr_state
        self._last_qr = None
        self.pipeline = None
        self.pipeline = rs.pipeline()
        config = rs.config()

        if file_bag is not None:
            config.enable_device_from_file(file_bag, repeat_playback=False)
        else:
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            
        profile = self.pipeline.start(config)
        align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.pc = rs.pointcloud()
        time.sleep(1)
        print("First table calibration")
        self.plane = None
        self.roi_mask = None
        self.h = None
        self.w = None
        self.table_calibration()

    def __del__(self):
        if self.pipeline is not None:
            self.pipeline.stop()

    def __depth_to_points(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32)
        return verts.reshape(-1, 3)

    def __estimate_plane(self, points):
        assert points.ndim == 2 and points.shape[1] == 3

        X = points[:, :2]   # x, y
        y = points[:, 2]    # z

        ransac = RANSACRegressor(
            estimator=LinearRegression(),
            residual_threshold=CONFIG.PLANE_THRESHOLD,
            min_samples=5000,
            max_trials=100
        )
        try:
            ransac.fit(X, y)

            # Piano: z = ax + by + c
            a_xy = ransac.estimator_.coef_
            c_z = ransac.estimator_.intercept_
        except:
            return None

        return np.array([a_xy[0], a_xy[1], -1.0, c_z], dtype=np.float64)


    def table_calibration(self, depth_frame = None):
        if depth_frame is None:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                print("No depth frame")
                return None

        depth = np.asanyarray(depth_frame.get_data())
        self.h, self.w = depth.shape
        
        x0, y0, x1, y1 = CONFIG.ROI_QR

        self.roi_mask = np.zeros((self.h, self.w), dtype=bool)
        self.roi_mask[y0:y1, x0:x1] = True

        points = self.__depth_to_points(depth_frame)
        points = points.reshape(self.h, self.w, 3)

        roi_points = points[self.roi_mask]
        roi_points = roi_points[np.isfinite(roi_points[:,2])]

        self.plane = self.__estimate_plane(roi_points)
        print("Piano stimato:", self.plane)

    def __point_plane_distance(self, points, plane = None):
        if plane is None:
            plane = self.plane
        a, b, c, d = self.plane
        num = a*points[:,0] + b*points[:,1] + c*points[:,2] + d
        den = np.sqrt(a*a + b*b + c*c)
        return num / den

    def find_occlusion(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        depth = np.asanyarray(depth_frame.get_data())
        
        points = self.__depth_to_points(depth_frame).reshape(self.h, self.w, 3)

        roi_points = points[self.roi_mask].reshape(-1, 3)
        valid = np.isfinite(roi_points[:,2])
        roi_points = roi_points[valid]

        #Table re-calibration
        # new_plane = self.__estimate_plane(roi_points)
        # if new_plane is not None:
        #     new_distances = self.__point_plane_distance(roi_points, new_plane)
        #     std_height = np.std(new_distances)
        #     if std_height < CONFIG.THRESH_STD:
        #         self.plane = new_plane


        distances = self.__point_plane_distance(roi_points)

        height_map = np.zeros((self.h, self.w), dtype=np.float32)
        roi_indices = np.argwhere(self.roi_mask)
        roi_heights = distances
        
        for (y, x), hgt in zip(roi_indices, roi_heights):
            if hgt > height_map[y, x]:
                height_map[y, x] = hgt

        object_mask = (height_map > CONFIG.MIN_HEIGHT_THRESHOLD)
        occlusion = np.sum(object_mask) > CONFIG.OCCLUSION_THRESHOLD
        
        return occlusion

    def read_qr(self):

        occlusion = self.find_occlusion()
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        if not color_frame:
            return None, occlusion

        img = np.asanyarray(color_frame.get_data())
        x0, y0, x1, y1 = CONFIG.ROI_QR
        img_roi = img[y0:y1, x0:x1]

        gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        #gray = cv2.equalizeHist(gray)
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, 5)

        codes = decode(bw, timeout=150, max_count=2)

        cv2.rectangle(
            img,
            (x0, y0),
            (x1, y1),
            (255, 0, 0),   # blu
            2
        )

        if codes and len(codes)==1:
            x, y, _, _= codes[0].rect

            cv2.putText(
                img,
                codes[0].data.decode("utf-8",  errors="replace"),
                (x + x0, y + y0 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            return codes[0].data.decode("utf-8",  errors="replace"), occlusion, img
        elif codes and len(codes)>1:
            for code in codes:
                x, y, _, _= code.rect
                cv2.putText(
                    img,
                    code.data.decode("utf-8",  errors="replace"),
                    (x + x0, y + y0 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )
            return None, True, img

        return None, occlusion, None

# TODO: questa può diventare sottoclasse di QrCamera
class ZEDQrCamera():
    def __init__(self, shared_qr_state:SharedQRState, file_bag : str = None):
        self.shared_qr_state = shared_qr_state
        self._last_qr = None
        self.zed = None
        self.zed = sl.Camera()

        init_params = sl.InitParameters()

        if file_bag is not None:
            init_parameters.set_from_svo_file(file_bag)
        else:
            init_params.camera_resolution = sl.RESOLUTION.HD2K
            init_params.depth_mode = sl.DEPTH_MODE.NEURAL
            init_params.coordinate_units = sl.UNIT.METER
            init_params.sdk_verbose = 1
            
        err = zed.open(init_params)
        if err > sl.ERROR_CODE.SUCCESS:
            print('Failure in opening zed')
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        time.sleep(1)
        print("First table calibration")
        self.plane = None
        self.roi_mask = None
        self.h = None
        self.w = None
        self.table_calibration()

    def __del__(self):
        if self.zed is not None:
            self.zed.close()
            

    def __depth_to_points(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32)
        return verts.reshape(-1, 3)

    def __estimate_plane(self, points):
        assert points.ndim == 2 and points.shape[1] == 3

        X = points[:, :2]   # x, y
        y = points[:, 2]    # z

        ransac = RANSACRegressor(
            estimator=LinearRegression(),
            residual_threshold=CONFIG.PLANE_THRESHOLD,
            min_samples=5000,
            max_trials=100
        )
        try:
            ransac.fit(X, y)

            # Piano: z = ax + by + c
            a_xy = ransac.estimator_.coef_
            c_z = ransac.estimator_.intercept_
        except:
            return None

        return np.array([a_xy[0], a_xy[1], -1.0, c_z], dtype=np.float64)


    def table_calibration(self, depth_frame = None):
        if zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            point_cloud = sl.Mat()
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
        else:
            print('Failure in grabbing ZED during table calibration')
            exit(1)

        depth = np.asanyarray(point_cloud.get_data())[:,:,:3]

        self.h, self.w, _ = depth.shape
        
        x0, y0, x1, y1 = CONFIG.ROI_QR

        self.roi_mask = np.zeros((self.h, self.w), dtype=bool)
        self.roi_mask[y0:y1, x0:x1] = True

        roi_points = depth[self.roi_mask]
        roi_points = roi_points[np.isfinite(roi_points[:,2])]

        self.plane = self.__estimate_plane(roi_points)
        print("Piano stimato:", self.plane)

    def __point_plane_distance(self, points, plane = None):
        if plane is None:
            plane = self.plane
        a, b, c, d = self.plane
        num = a*points[:,0] + b*points[:,1] + c*points[:,2] + d
        den = np.sqrt(a*a + b*b + c*c)
        return num / den

    def find_occlusion(self):
        if zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            point_cloud = sl.Mat()
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
        else:
            exit(1)

        points = np.asanyarray(point_cloud.get_data())[:,:,:3]

        roi_points = points[self.roi_mask].reshape(-1, 3)
        valid = np.isfinite(roi_points[:,2])
        roi_points = roi_points[valid]

        #Table re-calibration
        # new_plane = self.__estimate_plane(roi_points)
        # if new_plane is not None:
        #     new_distances = self.__point_plane_distance(roi_points, new_plane)
        #     std_height = np.std(new_distances)
        #     if std_height < CONFIG.THRESH_STD:
        #         self.plane = new_plane


        distances = self.__point_plane_distance(roi_points)

        height_map = np.zeros((self.h, self.w), dtype=np.float32)
        roi_indices = np.argwhere(self.roi_mask)
        roi_heights = distances
        
        for (y, x), hgt in zip(roi_indices, roi_heights):
            if hgt > height_map[y, x]:
                height_map[y, x] = hgt

        object_mask = (height_map > CONFIG.MIN_HEIGHT_THRESHOLD)
        
        occlusion = np.sum(object_mask) > CONFIG.OCCLUSION_THRESHOLD
        
        return occlusion

    def read_qr(self):
        occlusion = self.find_occlusion()
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            image = sl.Mat()
            zed.retrieve_image(image, sl.VIEW.LEFT)
        else:
            exit(1)

        img = np.asanyarray(image.get_data())[:,:,:3]
        x0, y0, x1, y1 = CONFIG.ROI_QR
        img_roi = img[y0:y1, x0:x1]

        gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        #gray = cv2.equalizeHist(gray)
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, 5)

        codes = decode(bw, timeout=200, max_count=2)

        if codes and len(codes)==1:
            return codes[0].data.decode("utf-8",  errors="replace"), occlusion, img
        elif codes and len(codes)>1:
            return None, True, img

        return None, occlusion, None