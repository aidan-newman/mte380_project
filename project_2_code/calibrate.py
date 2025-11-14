from datetime import datetime
import math
import cv2
import numpy as np
import time
import json

class Calibrator:

    def __init__(self):
        # ballancer settings
        self.BALANCER_DIAMETER = 0.3
        self.position_max = self.BALANCER_DIAMETER / 2 # Max position limit
        self.position_min = -self.BALANCER_DIAMETER / 2 # Min position limit
        self.circle_center = None # Known diameter length in meters
        self.circle_radius = None
        self.drawing_circle = False
        self.motor_points = [] # Selected motor points

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
        self.pixels_per_meter = None # Pixel-to-meter ratio
        self.pixel_to_meter_ratio = None

        # hardware settings
        self.servo_port = "COM4" # Servo port
        self.servo_baudrate = 9600 # Servo baudrate
        self.neutral_angle = 65 # Neutral servo angle


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
                self.pixel_to_meter_ratio = self.BALANCER_DIAMETER / (2 * self.circle_radius)
            elif event == cv2.EVENT_LBUTTONUP:
                # Stop drawing
                self.drawing_circle = False
        elif self.phase == "motor_select":
            if event == cv2.EVENT_LBUTTONDOWN and len(self.motor_points) < 3:
                self.motor_points.append((x, y))

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

    def get_ball_position(self, ball_x, ball_y):
        """Compute distance (m) and angle (deg) of ball relative to circle center."""
        if self.circle_center is None or self.pixel_to_meter_ratio is None:
            return None, None

        dx_pixels = ball_x - self.circle_center[0]
        dy_pixels = ball_y - self.circle_center[1]

        # Convert to meters
        dx_m = dx_pixels * self.pixel_to_meter_ratio
        dy_m = dy_pixels * self.pixel_to_meter_ratio

        return dx_m, dy_m
    
    def get_motor_unit_vectors(self):
        """Compute unit vectors from circle center to motor points in meters."""
        if self.circle_center is None or self.pixel_to_meter_ratio is None:
            return None

        unit_vectors = []
        cx, cy = self.circle_center

        for (mx, my) in self.motor_points:
            dx_pixels = mx - cx
            dy_pixels = cy - my # Invert y-axis

            # Convert to meters
            dx_m = dx_pixels * self.pixel_to_meter_ratio
            dy_m = dy_pixels * self.pixel_to_meter_ratio

            norm = math.sqrt(dx_m**2 + dy_m**2)
            unit_vector = (dx_m / norm, dy_m / norm)

            unit_vectors.append(unit_vector)

        return unit_vectors

    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame."""
        overlay = frame.copy()

        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "circle": "Draw and size circle around platform. Press 'n' when done.",
            "motor_select": "Click on all 3 motors to select them. Press 'm' when done.",
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

        # Draw circle if defined
        if self.circle_center and self.circle_radius:
            cx, cy = int(self.circle_center[0]), int(self.circle_center[1])
            cv2.circle(overlay, (cx, cy), int(self.circle_radius), (0, 255, 0), 2)
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(overlay, f"Center: ({cx}, {cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw motor points and lines
        if self.motor_points and len(self.motor_points) > 0:
            for i, (mx, my) in enumerate(self.motor_points, start=1):

                # Draw motor point dot
                cv2.circle(overlay, (int(mx), int(my)), 5, (0, 255, 255), -1)

                # Draw line to circle center
                if self.circle_center:

                    if self.pixel_to_meter_ratio:
                        cx, cy = int(self.circle_center[0]), int(self.circle_center[1])
                        dx_m = (mx - cx) * self.pixel_to_meter_ratio
                        dy_m = (cy - my) * self.pixel_to_meter_ratio

                        mx, my = int(mx), int(my)


                        cv2.line(overlay, (int(mx), int(my)), (cx, cy), (255, 0, 0), 2)
                        # Display info near motor point
                        cv2.putText(overlay, f"M{i}: X: {dx_m:.3f} m", (int(mx)+10, int(my)-20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        cv2.putText(overlay, f"Y: {dy_m:.3f} m", (int(mx)+10, int(my)-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                    
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

                    # Draw line and metrics relative to circle
                    if self.circle_center and self.pixel_to_meter_ratio:
                        cx, cy = self.circle_center
                        cv2.line(overlay, (int(cx), int(cy)), (int(x), int(y)), (255, 255, 0), 2)
                        dx_m = (x - cx) * self.pixel_to_meter_ratio
                        dy_m = (cy - y) * self.pixel_to_meter_ratio
                        cv2.putText(overlay, f"x: {dx_m:.3f} m", (int(x)+10, int(y)-20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        cv2.putText(overlay, f"y: {dy_m:.3f} m", (int(x)+10, int(y)-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                        ball_x, ball_y = self.get_ball_position(x, y)
                        if ball_x is not None and ball_y is not None:
                            cv2.putText(overlay, f"X: {ball_x:.3f} m", (10, 120),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                            cv2.putText(overlay, f"Y: {ball_y:.3f} m", (10, 140),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return overlay

    def save_config(self):
        """Save all calibration results to config.json file."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "balancer_diameter": float(self.BALANCER_DIAMETER),
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
                "position_min_m": self.position_min,
                "position_max_m": self.position_max,
                "center_x": self.circle_center[0] if self.circle_center else None,
                "center_y": self.circle_center[1] if self.circle_center else None
            },
            "servo": {
                "port": str(self.servo_port),
                "neutral_angle": int(self.neutral_angle)
            },
            "motor": {
                "pixels": {
                    "motor0": [float(x) for x in self.motor_points[0]] if len(self.motor_points) > 0 else None,
                    "motor1": [float(x) for x in self.motor_points[1]] if len(self.motor_points) > 1 else None,
                    "motor2": [float(x) for x in self.motor_points[2]] if len(self.motor_points) > 2 else None        
                },
                "unit_vector_m": {
                    "motor0": self.motor_unit_vectors[0] if self.motor_unit_vectors and len(self.motor_unit_vectors) > 0 else None,
                    "motor1": self.motor_unit_vectors[1] if self.motor_unit_vectors and len(self.motor_unit_vectors) > 1 else None,
                    "motor2": self.motor_unit_vectors[2] if self.motor_unit_vectors and len(self.motor_unit_vectors) > 2 else None        
                }
            }
        }
        
        # Write configuration to JSON file
        with open("config.json", "w") as f:
            json.dump(config, f, indent=2)
        print("[SAVE] Configuration saved to config.json")


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
            elif key == ord('n') and self.phase == "circle":
                if self.circle_center and self.circle_radius > 0:
                    print(f"[CIRCLE] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f}")
                    self.phase = "motor_select"
            elif key == ord('m') and self.phase == "motor_select":
                if len(self.motor_points) == 3:
                    self.motor_unit_vectors = self.get_motor_unit_vectors()
                    self.phase = "complete"
                    print("[INFO] Motor selection complete. Calibration finished.")
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = Calibrator()
    calibrator.run()