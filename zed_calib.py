import pyzed.sl as sl
import cv2
import numpy as np

print("Calibration ZED2 - premi 'q' per uscire")

# ================== ZED Init ==================
zed = sl.Camera()

# init_params = sl.InitParameters()
# init_params.camera_resolution = sl.RESOLUTION.HD2K
# init_params.camera_fps = 30
init_params.set_from_svo_file('./calibration.svo2')

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Errore apertura ZED:", err)
    exit(1)

image = sl.Mat()
runtime_params = sl.RuntimeParameters()

# ================== ROI iniziale ==================
ROI_X0 = 300
ROI_Y0 = 200
ROI_X1 = 900
ROI_Y1 = 600

WIN_NAME = "ZED2 - ROI live"
cv2.namedWindow(WIN_NAME)

# ================== Trackbar ==================
def nothing(x):
    pass

cv2.createTrackbar("X0", WIN_NAME, ROI_X0, 1279, nothing)
cv2.createTrackbar("Y0", WIN_NAME, ROI_Y0, 719,  nothing)
cv2.createTrackbar("X1", WIN_NAME, ROI_X1, 1279, nothing)
cv2.createTrackbar("Y1", WIN_NAME, ROI_Y1, 719,  nothing)

# ================== Loop ==================
try:
    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img = image.get_data()

            # BGR per OpenCV
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

            # Leggi ROI dalle trackbar
            x0 = cv2.getTrackbarPos("X0", WIN_NAME)
            y0 = cv2.getTrackbarPos("Y0", WIN_NAME)
            x1 = cv2.getTrackbarPos("X1", WIN_NAME)
            y1 = cv2.getTrackbarPos("Y1", WIN_NAME)

            # Disegna ROI solo se valida
            if x1 > x0 and y1 > y0:
                cv2.rectangle(
                    img,
                    (x0, y0),
                    (x1, y1),
                    (255, 0, 0),
                    2
                )

            cv2.imshow(WIN_NAME, img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    zed.close()
    cv2.destroyAllWindows()