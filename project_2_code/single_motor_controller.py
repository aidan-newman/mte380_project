import cv2
import numpy as np
import json
import serial
print(serial.__file__)
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from vector import Vector
from ball_detection import BallDetector

class BasicPIDController:
    def __init__(self, config_file="config.json"):


        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        # PID gains (controlled by sliders in GUI)
        self.Kp = 2.0
        self.Ki = 1.5
        self.Kd = 1.0
        # Scale factor for converting from pixels to meters
        self.scale_factor = self.config['calibration']['pixel_to_meter_ratio'] * self.config['camera']['frame_width'] / 2
        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None
        # Controller-internal state
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        # Data logs for plotting results
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown

        self.curr_motor = None # 0 1 2
        self.error_multiplier = 100

    def connect_servo(self):
        """Try to open serial connection to servo, return True if success."""
        try:
            self.servo = serial.Serial(self.servo_port, 115200)
            time.sleep(2)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            print(f"[SERVO] Failed: {e}")
            return False


    def send_servo_angle(self, angle):
        """Send angle command to servo motor (clipped for safety)."""
        if not self.servo:
            return

        # Initialize timestamp if not present
        if not hasattr(self, "_last_servo_write"):
            self._last_servo_write = 0
        
        # Rate limit to ~15 Hz (every 67 ms)
        now = time.time()
        if now - self._last_servo_write < 0.067:
            return
    
        self._last_servo_write = now
        
        match self.curr_motor.get():
            case 0:
                angle_data = str(int(self.neutral_angle - angle)) + "," + str(int(self.neutral_angle)) + "," + str(int(self.neutral_angle)) + "\n"
            case 1:
                angle_data = str(int(self.neutral_angle)) + "," + str(int(self.neutral_angle - angle)) + "," + str(int(self.neutral_angle)) + "\n"
            case 2:
                angle_data = str(int(self.neutral_angle)) + "," + str(int(self.neutral_angle)) + "," + str(int(self.neutral_angle - angle)) + "\n"
            case _:
                print("ERROR: Invalid motor value")

        try:
            self.servo.write(bytes(angle_data, 'utf-8'))
            print(f"[SERVO] Sent angles: {angle_data}")
        except Exception as e:
            print(f"[SERVO] Send failed: {e}")

    def update_pid(self, error, dt=0.033):

        """Perform PID calculation and return control output."""
        error *= self.error_multiplier
        print("Error Value: " + str(error))

        # Proportional term
        P = self.Kp * error
        # Integral term accumulation
        self.integral += error * dt
        # Limit integral term
        self.integral = np.clip(self.integral, -10, 10)
        I = self.Ki * self.integral
        # Derivative term calculation
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = error
        # PID output (limit to safe beam range)
        output = P + I + D
        output = np.clip(output, -20, 20)
        return output

    def camera_thread(self):
        self.ball_detector = BallDetector()

        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue

            # Detect ball position in frame
            found, ball_center, ball_radius, coords = self.ball_detector.detect_ball(frame)
            vis_frame = self.ball_detector.draw_detection(frame)
            x, y = coords
            vec = Vector(x, y)

            # print("[CAMERA] Ball detected at: " + str(vec))

            if found:
                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait(vec)
                except Exception:
                    pass
            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        cap.release()
        cv2.destroyAllWindows()


    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")
        
        x, y = self.config['motor']['unit_vector_m']["motor0"]
        u0 = Vector(x, y)
        x, y = self.config['motor']['unit_vector_m']["motor1"]
        u1 = Vector(x, y)
        x, y = self.config['motor']['unit_vector_m']["motor2"]
        u2 = Vector(x, y)
        
        print("u0: " + str(u0))
        print("u1: " + str(u1))
        print("u2: " + str(u2))

        self.start_time = time.time()
        while self.running:
            try:
                # Wait for latest ball position from camera
                coords = self.position_queue.get(timeout=0.1)

                print(f"coords: {coords.dot(u0)}, {coords.dot(u1)}, {coords.dot(u2)}")
                m0_dist = (coords.dot(u0) - self.setpoint)
                m1_dist = (coords.dot(u1) - self.setpoint)
                m2_dist = (coords.dot(u2) - self.setpoint)

                # Compute control output using PID
                control_output = 0
                match self.curr_motor.get():
                    case 0:
                        control_output = self.update_pid(m0_dist)
                    case 1:
                        control_output = self.update_pid(m1_dist)
                    case 2:
                        control_output = self.update_pid(m2_dist)
                    case _:
                        print("ERROR: Invalid motor value")

                # Send control command to servo (real or simulated)
                self.send_servo_angle(control_output)

                # Log results for plotting
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.setpoint_log.append(self.setpoint)
                self.control_log.append(control_output)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break

        if self.servo:
            # Return to neutral on exit
            self.send_servo_angle(0)
            self.servo.close()

    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Basic PID Controller")
        self.root.geometry("520x400")

        # Title label
        ttk.Label(self.root, text="PID Gains", font=("Arial", 18, "bold")).pack(pady=10)

        # Kp slider
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 12)).pack()
        self.kp_var = tk.DoubleVar(value=self.Kp)
        kp_slider = ttk.Scale(self.root, from_=0, to=50, variable=self.kp_var,
                              orient=tk.HORIZONTAL, length=500)
        kp_slider.pack(pady=5)
        self.kp_label = ttk.Label(self.root, text=f"Kp: {self.Kp:.1f}", font=("Arial", 11))
        self.kp_label.pack()

        # Ki slider
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 12)).pack()
        self.ki_var = tk.DoubleVar(value=self.Ki)
        ki_slider = ttk.Scale(self.root, from_=0, to=5, variable=self.ki_var,
                              orient=tk.HORIZONTAL, length=500)
        ki_slider.pack(pady=5)
        self.ki_label = ttk.Label(self.root, text=f"Ki: {self.Ki:.1f}", font=("Arial", 11))
        self.ki_label.pack()

        # Kd slider
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 12)).pack()
        self.kd_var = tk.DoubleVar(value=self.Kd)
        kd_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.kd_var,
                              orient=tk.HORIZONTAL, length=500)
        kd_slider.pack(pady=5)
        self.kd_label = ttk.Label(self.root, text=f"Kd: {self.Kd:.1f}", font=("Arial", 11))
        self.kd_label.pack()

        # Setpoint slider
        ttk.Label(self.root, text="Setpoint (meters)", font=("Arial", 12)).pack()
        pos_min = self.config['calibration']['position_min_m']
        pos_max = self.config['calibration']['position_max_m']
        self.setpoint_var = tk.DoubleVar(value=self.setpoint)
        setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                   variable=self.setpoint_var,
                                   orient=tk.HORIZONTAL, length=500)
        setpoint_slider.pack(pady=5)
        self.setpoint_label = ttk.Label(self.root, text=f"Setpoint: {self.setpoint:.3f}m", font=("Arial", 11))
        self.setpoint_label.pack()

        # motor selection
        self.curr_motor = tk.IntVar(value=0)

        ttk.Label(self.root, text="Select Motor", font=("Arial", 12)).pack()
        for i, name in enumerate(["Motor 0", "Motor 1", "Motor 2"]):
            ttk.Radiobutton(
                self.root,
                text=name,
                value=i,
                variable=self.curr_motor
            ).pack(anchor="w")

        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
        ttk.Button(button_frame, text="Reset Integral",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        # Schedule periodic GUI update
        self.update_gui()

    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters
            self.Kp = self.kp_var.get()
            self.Ki = self.ki_var.get()
            self.Kd = self.kd_var.get()
            self.setpoint = self.setpoint_var.get()
            # Update displayed values
            self.kp_label.config(text=f"Kp: {self.Kp:.1f}")
            self.ki_label.config(text=f"Ki: {self.Ki:.1f}")
            self.kd_label.config(text=f"Kd: {self.Kd:.1f}")
            self.setpoint_label.config(text=f"Setpoint: {self.setpoint:.3f}m")
            # Call again after 50 ms (if not stopped)
            self.root.after(50, self.update_gui)

    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral = 0.0
        print("[RESET] Integral term reset")

    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        # Ball position trace
        ax1.plot(self.time_log, self.position_log, label="Ball Position", linewidth=2)
        ax1.plot(self.time_log, self.setpoint_log, label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("Position (m)")
        ax1.set_title(f"Basic PID Control (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        # Control output trace
        ax2.plot(self.time_log, self.control_log, label="Control Output",
                 color="orange", linewidth=2)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Beam Angle (degrees)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False
        # Try to safely close all windows/resources
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()

        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_autocal.py first.")
    except Exception as e:
        print(f"[ERROR] {e}")
