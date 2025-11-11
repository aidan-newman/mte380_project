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
        self.circle_center = None # Known diameter length in meters
        self.circle_radius = None
        self.drawing_circle = False

        # camera settings
        self.CAMERA_INDEX = 0 # Default camera index
        self.FRAME_WIDTH = 640 # Default frame width
        self.FRAME_HEIGHT = 480 # Default frame height

        # state calibration settings
        self.current_frame = None # Current camera frame
        self.phase = 'color' # Calibration phase

        # color calibration settings
        self.hsv_samples = [] # Collected HSV samples
        self.lower_hsv = None # Lower HSV bounds
        self.upper_hsv = None # Upper HSV bounds

        # geometric calibration settings
        self.endpoints = [] # Selected beam endpoints
        self.pixels_per_meter = None # Pixel-to-meter ratio
        self.pixel_to_meter_ratio = None


        # hardware settings
        self.servo = None # Servo connection
        self.servo_port = "COM3" # Servo port
        self.servo_baudrate = 9600 # Servo baudrate
        self.neutral_angle = 65 # Neutral servo angle

        self.position_max = None # Max position limit
        self.position_min = None # Min position limit

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
        if self.phase == "color":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.sample_color(x, y)
        elif self.phase == "circle":
            if event == cv2.EVENT_LBUTTONDOWN:
                # Start drawing
                self.circle_center = (x, y)
                self.drawing_circle = True

            elif event == cv2.EVENT_MOUSEMOVE and self.drawing_circle:
                # Update radius dynamically
                dx = x - self.circle_center[0]
                dy = y - self.circle_center[1]
                self.circle_radius = int(math.sqrt(dx**2 + dy**2))

            elif event == cv2.EVENT_LBUTTONUP:
                # Stop drawing
                self.drawing_circle = False

        elif self.phase == "geometry":
            if event == cv2.EVENT_LBUTTONDOWN and len(self.endpoints) < 2:
                self.endpoints.append((x, y))
                print(f"[GEO] Endpoint {len(self.endpoints)} selected")
                if len(self.endpoints) == 2:
                    self.calculate_geometry()

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
                max(0, np.min(samples_int[:, 0]) - h_margin),
                max(0, np.min(samples_int[:, 1]) - s_margin),
                max(0, np.min(samples_int[:, 2]) - v_margin)
            ]

            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples_int[:, 0]) + h_margin),
                min(255, np.max(samples_int[:, 1]) + s_margin),
                min(255, np.max(samples_int[:, 2]) + v_margin)
            ]

            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")


    def calculate_geometry(self):
        """Calculate pixel-to-meter conversion ratio from beam endpoint coordinates."""
        p1, p2 = self.endpoints
        
        # Calculate pixel distance between beam endpoints
        distance1 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

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

    def find_limits_automatically(self):
        # Estimate limits without servo if connection failed
        self.position_min = -self.BEAM_LENGTH_M / 2
        self.position_max = self.BEAM_LENGTH_M / 2
        print("[LIMITS] Estimated without servo")
        return


    def save_config(self):
        """Save all calibration results to config.json file."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "beam_length_m": float(self.BEAM_LENGTH_M),
            "camera": {
                "index": int(self.CAMERA_INDEX),
                "frame_width": int(self.FRAME_WIDTH),
                "frame_height": int(self.FRAME_HEIGHT)
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

    def get_ball_polar_position(self, ball_x, ball_y):
        """Compute distance (m) and angle (deg) of ball relative to circle center."""
        if self.circle_center is None or self.pixel_to_meter_ratio is None:
            return None, None

        dx_pixels = ball_x - self.circle_center[0]
        dy_pixels = ball_y - self.circle_center[1]

        # Convert to meters
        dx_m = dx_pixels * self.pixel_to_meter_ratio
        dy_m = dy_pixels * self.pixel_to_meter_ratio

        # Distance from center
        distance = math.sqrt(dx_m**2 + dy_m**2)

        # Angle relative to positive x-axis (to the right)
        angle_rad = math.atan2(dy_m, dx_m)
        angle_deg = math.degrees(angle_rad)

        return distance, angle_deg

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
            "circle": "Draw and size circle around platform. Press 'n' when done.",
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
            
        if self.phase == "circle" and self.circle_center and self.circle_radius:
            if isinstance(self.circle_center, tuple) and len(self.circle_center) == 2:
                cv2.circle(overlay, (int(self.circle_center[0]), int(self.circle_center[1])), 
                        int(self.circle_radius), (0, 255, 0), 2)
        
        # Show geometry calibration points
        for i, endpoint in enumerate(self.endpoints):
            cv2.circle(overlay, endpoint, 8, (0, 255, 0), -1)
            cv2.putText(overlay, f"Endpoint {i+1}", (endpoint[0]+10, endpoint[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw line between beam endpoints if both are selected
        if len(self.endpoints) == 2:
            cv2.line(overlay, self.endpoints[0], self.endpoints[1], (255, 0, 0), 2)

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
            # Inside draw_overlay, replace the ball detection part with:

            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)

                    # Draw line from circle center to ball
                    if self.circle_center:
                        cv2.line(overlay, (int(self.circle_center[0]), int(self.circle_center[1])),
                                (int(x), int(y)), (255, 255, 0), 2)

                    # Compute distance and angle relative to circle center
                    distance, angle_deg = self.get_ball_polar_position(x, y)
                    if distance is not None:
                        cv2.putText(overlay, f"Dist: {distance:.3f} m", (10, 120),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        cv2.putText(overlay, f"Angle: {angle_deg:.1f} deg", (10, 140),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Keep your circle drawing code as is:
            if self.phase == "circle" and self.circle_center and self.circle_radius:
                cv2.circle(overlay, (int(self.circle_center[0]), int(self.circle_center[1])), 
                        int(self.circle_radius), (0, 255, 0), 2)

                    
                    # # Show position if geometry calibration is complete
                    # if self.pixel_to_meter_ratio:
                    #     pos = self.detect_ball_position(frame)
                    #     if pos is not None:
                    #         cv2.putText(overlay, f"Pos: {pos:.4f}m",
                    #                    (int(x)+20, int(y)+20),
                    #                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # # Show final results if limit calibration is complete
        # if self.position_min is not None and self.position_max is not None:
        #     cv2.putText(overlay, f"Limits: {self.position_min:.4f}m to {self.position_max:.4f}m",
        #                (10, overlay.shape[0] - 20),
        #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return overlay

    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Auto Calibration")
        cv2.setMouseCallback("Auto Calibration", self.mouse_event)
        
        # Attempt servo connection
        self.connect_servo()
        
        # Display instructions
        print("[INFO] Simple Auto Calibration")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        print("Phase 2: Click on beam endpoints")
        print("Phase 3: Press 'l' to find limits")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Auto Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('c') and self.phase == "color":
                # Complete color calibration phase
                if self.hsv_samples:
                    self.phase = "circle"
                    print("[INFO] Color calibration complete. Draw and resize circle over the platform.")
                    
                    #self.phase = "geometry"
                    #print("[INFO] Color calibration complete. Click on beam endpoints.")
            elif key == ord('n') and self.phase == "circle":
                if self.circle_center and self.circle_radius > 0:
                    self.pixel_to_meter_ratio = (self.BALANCER_DIAMETER / 2) / self.circle_radius
                    print(f"[CIRCLE] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f}")
                    self.phase = "geometry"
                    print("[INFO] Circle calibration complete. Click on beam endpoints.")
            elif key == ord('l') and self.phase == "limits":
                # Start automatic limit finding
                self.find_limits_automatically()
                self.phase = "complete"
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
        if self.servo:
            self.servo.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = Calibrator()
    calibrator.run()