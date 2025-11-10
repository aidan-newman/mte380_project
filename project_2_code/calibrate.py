from datetime import datetime
import math
import cv2
import serial
import numpy as np
import time
import json

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

    def send_servo_angle(self, angle1, angle2, angle3):
        if self.servo:
            # Clip angle to safe range and send as byte
            angle1 = int(np.clip(angle1, 0, 140))
            angle2 = int(np.clip(angle2, 0, 140))
            angle3 = int(np.clip(angle3, 0, 140))
            data = [angle1, angle2, angle3]
            self.servo.write(bytes(data))

    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "colour":
                self.sample_colour(x,y)
            elif self.phase == "geometry" and len(self.endpoints) < 6:
                self.endpoints.append((x, y))
                print(f"[GEO] Endpoint {len(self.endpoints)} selected")
                if len(self.endpoints) == 6:
                    self.calculate_geometry()


    def sample_colour(self, x, y):
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
            
            # Set lower bounds with margin
            self.lower_hsv = [
                max(0, np.min(samples[:, 0]) - h_margin),
                max(0, np.min(samples[:, 1]) - s_margin),
                max(0, np.min(samples[:, 2]) - v_margin)
            ]
            
            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples[:, 0]) + h_margin),
                min(255, np.max(samples[:, 1]) + s_margin),
                min(255, np.max(samples[:, 2]) + v_margin)
            ]
            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")


    def calculate_geometry(self):
        """Calculate pixel-to-meter conversion ratio from beam endpoint coordinates."""
        p1, p2, p3, p4, p5, p6 = self.endpoints
        
        # Calculate pixel distance between beam endpoints
        distance1 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        distance2 = math.sqrt((p4[0] - p3[0])**2 + (p4[1] - p3[1])**2)
        distance3 = math.sqrt((p6[0] - p5[0])**2 + (p6[1] - p5[1])**2)

        # Convert to meters using known balancer diameter
        self.pixel_to_meter_ratio = self.BALANCER_DIAMETER / distance1
        print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f}")
        
        # Advance to limits calibration phase
        self.phase = "limits"


    def detect_ball_position(self, frame):
        if not self.lower_hsv:
            return None
        
        # Convert to HSV and create color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Get largest contour (assumed to be ball)
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        
        # Filter out very small detections
        if radius < 5:
            return None
        
        # Convert pixel position to meters from center
        center_x = frame.shape[1] // 2
        pixel_offset = x - center_x
        meters_offset = pixel_offset * self.pixel_to_meter_ratio
        
        return meters_offset


    def save_config(self):
        """Save all calibration results to config.json file."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "beam_length_m": float(self.BEAM_LENGTH_M),
            "camera": {
                "index": int(self.CAM_INDEX),
                "frame_width": int(self.FRAME_W),
                "frame_height": int(self.FRAME_H)
            },
            "ball_detection": {
                "lower_hsv": [float(x) for x in self.lower_hsv] if self.lower_hsv else None,
                "upper_hsv": [float(x) for x in self.upper_hsv] if self.upper_hsv else None
            },
            "calibration": {
                "pixel_to_meter_ratio": float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None,
                "position_min_m": float(self.position_min) if self.position_min else None,
                "position_max_m": float(self.position_max) if self.position_max else None
            },
            "servo": {
                "port": str(self.servo_port),
                "neutral_angle": int(self.neutral_angle)
            }
        }
        
        # Write configuration to JSON file
        with open("config.json", "w") as f:
            json.dump(config, f, indent=2)
        print("[SAVE] Configuration saved to config.json")



    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            numpy.ndarray: Frame with overlay graphics and text
        """
        overlay = frame.copy()
        
        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "geometry": "Click on beam endpoints (2 points)",
            "limits": "Press 'l' to find limits automatically",
            "complete": "Calibration complete! Press 's' to save"
        }
        
        # Draw current phase and instructions
        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, phase_text[self.phase], (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show geometry calibration points
        for i, peg in enumerate(self.peg_points):
            cv2.circle(overlay, peg, 8, (0, 255, 0), -1)
            cv2.putText(overlay, f"Peg {i+1}", (peg[0]+10, peg[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw line between beam endpoints if both are selected
        if len(self.peg_points) == 2:
            cv2.line(overlay, self.peg_points[0], self.peg_points[1], (255, 0, 0), 2)
        
        # Show real-time ball detection if color calibration is complete
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find and draw detected ball
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)
                    
                    # Show position if geometry calibration is complete
                    if self.pixel_to_meter_ratio:
                        pos = self.detect_ball_position(frame)
                        if pos is not None:
                            cv2.putText(overlay, f"Pos: {pos:.4f}m",
                                       (int(x)+20, int(y)+20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show final results if limit calibration is complete
        if self.position_min is not None and self.position_max is not None:
            cv2.putText(overlay, f"Limits: {self.position_min:.4f}m to {self.position_max:.4f}m",
                       (10, overlay.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return overlay



    def run(self):
        # init camera
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Calibration", self.frame_width, self.frame_height)
        cv2.setMouseCallback("Calibration", self.mouse_event)

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = Calibrator()
    calibrator.run()