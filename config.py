
#----------------Node Parameters------------------#
#----------------End Node Parameters------------------#


#----------------Camera Parameters------------------#
IS_ZED = False
ARUCO_MODE = False
DEBUGGING = False

ROI = (390, 30, 930, 420) # x0, y0, x1, y1
#ROI for enter obj detection camera
ROI_A = (360, 0, 950, 395) # x0, y0, x1, y1
#ROI for exit obj detection camera
ROI_B = (390, 30, 930, 420) # x0, y0, x1, y1
#ROI for env camera
ROI_QR = (370,290,440,415) # x0, y0, x1, y1

ROI_QR_ZED = (513, 386, 696, 616)

#Threshold for plane calibration
PLANE_THRESHOLD = 0.005  #5mm

#min sigma for table ri-calibration
THRESH_STD = 0.1

#Max distance in shape validation
SHAPE_THRESHOLD = 1.5

#Max-Min heigh from plane in object detection
MIN_HEIGHT_THRESHOLD = 0.05
MAX_HEIGHT_THRESHOLD = 0.30

#Min occupancy area (in px) for object detection
MIN_AREA_PIXELS = 5000
#Max occupancy area for no object detection
MAX_OCCUPATION = 100

#Number of frame for debouncing object detection
MAX_LEN = 5

#Max number of pixel for detect occlusion on qr reading
OCCLUSION_THRESHOLD = 1000
#----------------End Camera Parameters------------------#

#----------------GPIO Parameters------------------#

BUTTON_PIN = 85 
LED_PIN = 106 
#----------------End GPIO Parameters------------------#

#----------------Handler Parameters------------------#
THERE_IS_LED = False
THERE_IS_BUTTON = False
IS_JETSON = False

#States for FSM.
NO_OBJECT = 0
OBJECT_PRESENT = 1

NO_PRESS = 0
PRESSED = 1

NO_QR = 0
QR = 1

#Debouncing times for the various FSM (seconds)
OBJ_MIN_OFF_TIME = 2

BUT_MIN_ON_TIME = 0.5
BUT_MIN_OFF_TIME = 2

QR_MIN_ON_TIME = 1
QR_MIN_OFF_TIME = 3
QR_MIN_CHANGE_TIME = 5
#----------------End Handler Parameters------------------#

#----------------Node Parameters------------------#
ONLINE_SENDER = True
ONLINE_RECIEVER = True
ONLINE_SENDER_ENV = False
#----------------End Node Parameters------------------#

#----------------Network Parameters------------------#
SERVER_URL = "192.168.0.233"
SERVER_PORT = 9000
BROKER_IP = "mqtt.192.168.0.60.nip.io"
MQTT_PORT = 443
#----------------End Network Parameters------------------#
