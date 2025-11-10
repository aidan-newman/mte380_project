import cv2
import serial
import numpy as np
import time

class Calibrator:

    def __init__(self):
        # ballancer settings
        self.BALANCER_DIAMETER = 0.15

        # camera settings
        self.CAMERA_INDEX = 0
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480

        # state calibration settings
        self.current_frame = None
        self.phase = "colour"

        # colour calibration settings
        self.hsv_samples = []
        self.hsv_lower = None
        self.hsv_upper = None

        # geometric calibration settings
        self.endpoints = []
        self.pixels_per_meter = None

        # hardware settings
        self.servo = None
        self.servo_port = "COM3"
        self.servo_baudrate = 9600
        self.neutral_angle = 70


    def connect_servo(self):
        try:
            self.servo = serial.Serial(self.servo_port, self.servo_baudrate)
            time.sleep(2)  # wait for servo to initialize
            print("Servo connected successfully.")
            return True
        except Exception as e:
            print(f"Failed to connect to servo: {e}")
            return False

    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "colour":
                self.sample_colour(x,y)
            elif self.phase == "geometry":
                self.endpoints.append((x, y))
                print(f"Selected endpoint: {(x, y)}")


    def sample_colour(self, x, y):
        if self.current_frame is not None:
            hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    sample = hsv[y + dy, x + dx]
                    self.hsv_samples.append(sample)

            if self.hsv_samples:
                self.hsv_lower = np.min(self.hsv_samples, axis=0)
                self.hsv_upper = np.max(self.hsv_samples, axis=0)
                print(f"Calibrated HSV range: {self.hsv_lower} - {self.hsv_upper}")

    def calibrate(self):
        # init camera
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Calibration", self.frame_width, self.frame_height)
        cv2.setMouseCallback("Calibration", self.mouse_event)

