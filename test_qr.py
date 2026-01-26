import pyrealsense2 as rs
import numpy as np
import cv2
import time
#from pyzbar.pyzbar import decode
from pylibdmtx.pylibdmtx import decode

from hardware.camera import QrCamera


def depth_to_points(depth_frame):
    points = pc.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32)
    return verts.reshape(-1, 3)


USE_BAG = True
# ================== RealSense ==================
pipeline = rs.pipeline()
config = rs.config()

if USE_BAG:
    config.enable_device_from_file("./prove/dati_test/test_qr_3.bag",  repeat_playback=False)
    #config.enable_stream(rs.stream.color, rs.format.bgr8)
else:
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
pc = rs.pointcloud()
print("Lettura QR industriale (pyzbar) - premi 'q' per uscire")

# ================== ROI (MODIFICA QUI) ==================
# Esempio: area centrale
ROI_X = 300
ROI_Y = 300
ROI_W = 300
ROI_H = 300

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame:
            continue
        depth = np.asanyarray(depth_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        h_d, w_d = depth.shape

        points = depth_to_points(depth_frame)
        points = points.reshape(h_d, w_d, 3)

        img = np.asanyarray(color_frame.get_data())

        # ================== Disegna ROI ==================
        cv2.rectangle(
            img,
            (ROI_X, ROI_Y),
            (ROI_X + ROI_W, ROI_Y + ROI_H),
            (255, 0, 0),   # blu
            2
        )

        # ================== Crop ROI ==================
        roi = img[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

        # ================== Preprocessing ==================
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        #gray = cv2.equalizeHist(gray)
        bw = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            31,
            5
        )
        # ================== Decode QR ==================
        codes = decode(bw, timeout=150, max_count=2)

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