from datetime import datetime
import math
import cv2
import numpy as np
import time
import json
from pupil_apriltags import Detector

class Calibrator:

    def __init__(self):
        self.CAMERA_INDEX = 0
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        self.current_frame = None # Current camera frame

        # color calibration settings
        self.hsv_samples = [] # Collected HSV samples
        self.lower_hsv = None # Lower HSV bounds
        self.upper_hsv = None # Upper HSV bounds


    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.sample_color(x, y)

    def sample_color(self, x, y):
        if self.current_frame is None:
                return
            
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
    
        # Sample 5x5 region around click point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x + dx, y + dy
                # Check bounds and collect valid samples
                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])
        
        # Update HSV bounds based on collected samples
        if self.hsv_samples:
            samples = np.array(self.hsv_samples)
            
            # Calculate adaptive margins for each HSV channel
            h_margin = max(5, (np.max(samples[:, 0]) - np.min(samples[:, 0])) * 0.1)
            s_margin = max(10, (np.max(samples[:, 1]) - np.min(samples[:, 1])) * 0.15)
            v_margin = max(10, (np.max(samples[:, 2]) - np.min(samples[:, 2])) * 0.15)
            
            # Convert samples to int to avoid overflow
            samples_int = samples.astype(int)

            # Set lower bounds with margin
            self.lower_hsv = [
                int(max(0, np.min(samples_int[:, 0]) - h_margin)),
                int(max(0, np.min(samples_int[:, 1]) - s_margin)),
                int(max(0, np.min(samples_int[:, 2]) - v_margin))
            ]

            # Set upper bounds with margin
            self.upper_hsv = [
                int(min(179, np.max(samples_int[:, 0]) + h_margin)),
                int(min(255, np.max(samples_int[:, 1]) + s_margin)),
                int(min(255, np.max(samples_int[:, 2]) + v_margin))
            ]

            print(f"[COLOR] Samples: {len(self.hsv_samples)}")


    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame."""
        overlay = frame.copy()

        # Instructions
        cv2.putText(overlay, "Calibration Mode", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(overlay, "Click on ball to sample colors", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(overlay, "Press 's' to complete calibration", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Detect ball if color calibration is done
        self.ball_position = None
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    self.ball_position = (int(x), int(y))
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)

        return overlay

    def update_config(self, calibration_data, file_path='config.json'):
        try:
            with open(file_path, "r") as f:
                config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            print("[WARN] config.json missing or invalid, creating new one...")
            config = {}

        # merge values
        for key, value in calibration_data.items():
            if key in config and isinstance(config[key], dict) and isinstance(value, dict):
                config[key].update(value)
            else:
                config[key] = value

        # save
        with open(file_path, "w") as f:
            json.dump(config, f, indent=4)

        print("[INFO] Calibration saved successfully!")

    
    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Simple Calibration")
        cv2.setMouseCallback("Simple Calibration", self.mouse_event)
        
        # Display instructions
        print("[INFO] Simple Calibration")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Simple Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('s'):
                # Save configuration and exit
                json_data = {
                    "ball_detection": {
                        "lower_hsv": self.lower_hsv,
                        "upper_hsv": self.upper_hsv
                    }
                }
                self.update_config(json_data, 'config.json')
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = Calibrator()
    calibrator.run()