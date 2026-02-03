import pyrealsense2 as rs
import numpy as np
import cv2
import time
#from pyzbar.pyzbar import decode
from pylibdmtx.pylibdmtx import decode
import pyzed.sl as sl

#from hardware.camera import QrCamera


def depth_to_points(depth_frame):
    points = pc.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32)
    return verts.reshape(-1, 3)


zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD2K
init_params.depth_mode = sl.DEPTH_MODE.NEURAL
init_params.coordinate_units = sl.UNIT.METER
init_params.sdk_verbose = 1
# Chat gpt suggested settings for Datamatrix decode
zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 60)
zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 20)
zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 0)
zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)

err = zed.open(init_params)
if err > sl.ERROR_CODE.SUCCESS:
    exit(1)


# ================== ROI (MODIFICA QUI) ==================
# Esempio: area centrale
ROI_X = 500
ROI_Y = 300
ROI_W = 400
ROI_H = 300

try:
    while True:
        runtime_params = sl.RuntimeParameters()

        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            point_cloud = sl.Mat()
            image = sl.Mat()
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            zed.retrieve_image(image, sl.VIEW.LEFT)

        """ depth = np.asanyarray(point_cloud.get_data())[:,:,:3]
        roi_depth = depth[roi_mask].reshape(-1, 3)
        valid = np.isfinite(roi_depth[:,2])
        roi_depth = roi_depth[valid] """


        img = np.asanyarray(image.get_data())

        # ================== Crop ROI ==================
        roi = img[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

        # ================== Preprocessing ==================
        gray = cv2.cvtColor(roi, cv2.COLOR_RGBA2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.equalizeHist(gray)
        bw = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            31,
            5
        )
        # ================== Decode QR ==================
        codes = decode(bw, timeout=200, max_count=2)

        for code in codes:
            try:
                data = code.data.decode("utf-8", errors="replace")
                print("QR:", data)
                
            

                # Bounding box RELATIVA ALLA ROI
                x, y, w, h = code.rect

                # Trasforma in coordinate GLOBALI
                xg = ROI_X + x
                yg = ROI_Y + y

                # cv2.rectangle(
                #     img,
                #     (xg, yg-h),
                #     (xg + w, yg),
                #     (0, 255, 0),
                #     3
                # )

                cv2.putText(
                    img,
                    data,
                    (xg, yg - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )
            except:
                print('Error')

        cv2.imshow("QR distante - ROI visibile", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()