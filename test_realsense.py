import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2
import time
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
# =====================
# PARAMETRI
# =====================

# ROI del tavolo (da regolare)
ROI = (150, 90, 440, 400)  # x0, y0, x1, y1

# RANSAC
PLANE_THRESH = 0.005  

SHAPE_THRESHOLD = 1.5
# MIN_POINTS_PER_SLICE = 1
# OVERLAP_THRESHOLD = 0.01

MIN_HEIGHT_THRESHOLD = 0.05
MAX_HEIGHT_THRESHOLD = 0.3
MIN_AREA_PIXELS = 5000        # area minima occupata
MAX_OCCUPATION = 100
THRESH_STD = 0.1


USE_BAG = True

# =====================
# REALSENSE SETUP
# =====================

pipeline = rs.pipeline()
config = rs.config()

if USE_BAG:
    config.enable_device_from_file("./prove/dati_test/test_obj_3.bag",  repeat_playback=False)
else:
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

pc = rs.pointcloud()

# =====================
# FUNZIONI
# =====================

def depth_to_points(depth_frame):
    points = pc.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32)
    return verts.reshape(-1, 3)

def estimate_plane(points):
    assert points.ndim == 2 and points.shape[1] == 3

    X = points[:, :2]   # x, y
    y = points[:, 2]    # z

    ransac = RANSACRegressor(
        estimator=LinearRegression(),
        residual_threshold=PLANE_THRESH,
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

def point_plane_distance(points, plane):
    a, b, c, d = plane
    num = a*points[:,0] + b*points[:,1] + c*points[:,2] + d
    den = np.sqrt(a*a + b*b + c*c)
    return num / den

def overlap(mask1, mask2):
    inter = np.logical_and(mask1, mask2)
    return np.sum(inter) / max(np.sum(mask1), 1)

def there_is_obj(obj_list):
    for i in range(len(obj_list)-1):
        if not (overlap(obj_list[i],obj_list[i+1])>0.90):
            return False
    return True

def shape_validation(obj_mask, expected_hu_set):
    shape_ok_raw = False
    num_labels, labels = cv2.connectedComponents(obj_mask.astype(np.uint8))
    for label in range(1, num_labels):
        component = (labels == label)
        area = np.sum(component)
        if area < MIN_AREA_PIXELS:
            continue

        mask_uint8 = (component.astype(np.uint8)) * 255
        moments = cv2.moments(mask_uint8)
        hu = cv2.HuMoments(moments)
        hu = (np.sign(hu) * np.log10(np.abs(hu) + 1e-12)).flatten()
        distances = np.linalg.norm(expected_hu_set - hu, axis=1)
        min_distance = np.min(distances)
        #print(min_distance)
        shape_ok_raw = min_distance < SHAPE_THRESHOLD

    return shape_ok_raw



# =====================
# STIMA PIANO DEL TAVOLO
# =====================

expected_hu_set = np.load("expected_shape/expected_hu.npy")

print("Calibrazione piano del tavolo...")

time.sleep(1)
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
if not depth_frame:
    exit()

depth = np.asanyarray(depth_frame.get_data())
h, w = depth.shape
x0, y0, x1, y1 = ROI

roi_mask = np.zeros((h, w), dtype=bool)
roi_mask[y0:y1, x0:x1] = True

points = depth_to_points(depth_frame)
points = points.reshape(h, w, 3)

roi_points = points[roi_mask]
roi_points = roi_points[np.isfinite(roi_points[:,2])]


plane = estimate_plane(roi_points)
print("Piano stimato:", plane)

# =====================
# LOOP PRINCIPALE
# =====================

print("Avvio rilevamento oggetti...")

detect_queue = []
MAX_LEN = 2
obj_find = False
while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        continue

    depth = np.asanyarray(depth_frame.get_data())
    points = depth_to_points(depth_frame).reshape(h, w, 3)

    roi_points = points[roi_mask].reshape(-1, 3)
    valid = np.isfinite(roi_points[:,2])
    roi_points = roi_points[valid]

    new_plane = estimate_plane(roi_points)
    if new_plane is not None:
        new_distances = point_plane_distance(roi_points, new_plane)
        std_height = np.std(new_distances)
        print(std_height)
        if std_height < THRESH_STD:
            plane = new_plane
            print("Piano ricalibrato:", plane)

    distances = point_plane_distance(roi_points, plane)
    
    # Height map inizializzata a zero
    height_map = np.zeros((h, w), dtype=np.float32)
    roi_indices = np.argwhere(roi_mask)
    roi_heights = distances
    
    for (y, x), hgt in zip(roi_indices, roi_heights):
        if hgt > height_map[y, x]:
            height_map[y, x] = hgt

    object_mask = (height_map > MIN_HEIGHT_THRESHOLD) & (height_map < MAX_HEIGHT_THRESHOLD)
    object_detected = np.sum(object_mask) > MIN_AREA_PIXELS
    shape_ok = shape_validation(object_mask, expected_hu_set)
    
    if object_detected:
        detect_queue.append(object_mask)
        if len(detect_queue)>MAX_LEN:
            detect_queue.pop(0)
            if not obj_find:
                obj_find = (there_is_obj(detect_queue)) & shape_ok
    else:
        if obj_find:
            obj_find = False
            #time.sleep(5)
        detect_queue = []

    


    # =====================
    # VISUALIZZAZIONE
    # =====================

    vis = cv2.applyColorMap(
        cv2.convertScaleAbs(depth, alpha=0.03),
        cv2.COLORMAP_JET
    )

    cv2.rectangle(vis, (x0,y0), (x1,y1), (0,255,0), 2)

    text = "OGGETTO PRESENTE" if obj_find else "NESSUN OGGETTO"
    color = (0,0,255) if obj_find else (0,255,0)
    cv2.putText(vis, text, (20,40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    vis_height = np.zeros((h, w, 3), dtype=np.uint8)
    vis_height[object_mask] = (0, 0, 255)
    if obj_find:
        vis = cv2.addWeighted(vis, 1.0, vis_height, 1.0, 0)


    cv2.imshow("Depth", vis)

    if cv2.waitKey(1) == 27:
        break

pipeline.stop()
cv2.destroyAllWindows()