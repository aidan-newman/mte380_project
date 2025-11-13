# Ball Detection Module for Computer Vision Ball Tracking System
# Detects colored balls in video frames using HSV color space filtering
# Provides both class-based and legacy function interfaces

import cv2
import numpy as np
import json
import os

class BallDetector:
    
    def __init__(self, config_file="config.json"):
        # Default HSV bounds for orange ball detection
        self.lower_hsv = np.array([5, 150, 150], dtype=np.uint8)  # Orange lower bound
        self.upper_hsv = np.array([20, 255, 255], dtype=np.uint8)  # Orange upper bound
        self.scale_factor = 1.0  # Conversion factor from normalized coords to meters
        
        # Load configuration from file if it exists
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)

                # Extract HSV color bounds from config
                if 'ball_detection' in config:
                    if config['ball_detection']['lower_hsv']:
                        self.lower_hsv = np.array(config['ball_detection']['lower_hsv'], dtype=np.uint8)
                    if config['ball_detection']['upper_hsv']:
                        self.upper_hsv = np.array(config['ball_detection']['upper_hsv'], dtype=np.uint8)
                
                # Extract scale factor for position conversion from pixels to meters
                if 'calibration' in config and 'pixel_to_meter_ratio' in config['calibration']:
                    if config['calibration']['pixel_to_meter_ratio']:
                        frame_width = config.get('camera', {}).get('frame_width', 640)
                        frame_height = config.get('camera', {}).get('frame_height', 480)
                        self.center_x = config['calibration'].get('center_x', frame_width // 2)
                        self.center_y = config['calibration'].get('center_y', frame_height // 2)
                        self.scale_factor = config['calibration']['pixel_to_meter_ratio'] * (frame_width / 2)
                
                print(f"[BALL_DETECT] Loaded HSV bounds: {self.lower_hsv} to {self.upper_hsv}")
                print(f"[BALL_DETECT] Scale factor: {self.scale_factor:.6f} m/normalized_unit")
                
            except Exception as e:
                print(f"[BALL_DETECT] Config load error: {e}, using defaults")
        else:
            print("[BALL_DETECT] No config file found, using default HSV bounds")

    def detect_ball(self, frame):
        # Convert frame from BGR to HSV color space for better color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create binary mask using HSV color bounds
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # Clean up mask using morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find all contours in the cleaned mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, (0.0, 0.0)
        
        largest_contour = max(contours, key=cv2.contourArea)
        ((ball_x_pixels, ball_y_pixels), ball_radius) = cv2.minEnclosingCircle(largest_contour)
        
        if ball_radius < 5 or ball_radius > 100:
            return False, None, None, (0.0, 0.0)
        
        # Convert pixel position to meters from center
        center_x = self.center_x
        center_y = self.center_y
        
        normalized_x = (ball_x_pixels - center_x) / center_x
        normalized_y = (center_y - ball_x_pixels) / center_y
        
        position_m_x = normalized_x * self.scale_factor
        position_m_y = normalized_y * self.scale_factor
        
        return True, (int(ball_x_pixels), int(ball_y_pixels)), ball_radius, (position_m_x, position_m_y)


    def draw_detection(self, frame, show_info=True):
        found, ball_center, ball_radius, ball_relative = self.detect_ball(frame)
        ball_x, ball_y = ball_relative[0], ball_relative[1]

        overlay = frame.copy()
        height, width = frame.shape[:2]
        center_x = self.center_x
        center_y = self.center_y

        # Draw center cross
        cv2.line(overlay, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        cv2.putText(overlay, "Center", (center_x + 5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.line(overlay, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.putText(overlay, "Center", (5, center_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if found:
            # Draw the ball
            cv2.circle(overlay, ball_center, int(ball_radius), (0, 255, 0), 2)
            cv2.circle(overlay, ball_center, 3, (0, 255, 0), -1)

            # ---------- Draw Vector from Center to Ball ----------
            cv2.arrowedLine(
                overlay,
                (center_x, center_y),     # start at image center
                ball_center,              # end at ball location
                (0, 0, 255),              # red arrow
                2,                        # thickness
                tipLength=0.15            # arrow head size
            )

            # Draw coordinate info
            if show_info:
                cv2.putText(overlay, f"x: {ball_x:.4f} m", (ball_center[0] - 40, ball_center[1] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(overlay, f"y: {ball_y:.4f} m", (ball_center[0] - 40, ball_center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return overlay

    

# For testing/calibration when run directly
def main():
    """Test ball detection with current config."""
    detector = BallDetector()
    print("here")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use default camera

    if not cap.isOpened():
        print("❌ Could not open camera.")
        return
    else:
        print("✅ Camera opened successfully.")
    
    print("Ball Detection Test")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Resize frame for consistent processing
        frame = cv2.resize(frame, (640, 480))
        
        vis_frame = detector.draw_detection(frame)
        found, ball_center, ball_radius, ball_relative = detector.detect_ball(frame)
        x, y = ball_relative
        
        # Show detection info in console
        if found:
            print(f"Ball detected — x = {x} -y = {y}")
        else:
            print("No ball detected.")
        
        # Display frame with detection overlay
        cv2.imshow("Ball Detection Test", vis_frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()