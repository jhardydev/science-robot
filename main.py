#!/usr/bin/env python3
"""
Duckiebot Science Fair Robot - Main Control Loop
Detects waving, steers toward wavers, performs dance on gesture, and handles treat dispensing
Enhanced with NVIDIA GPU acceleration and ROS integration
"""
import rospy
import cv2
import time
import signal
import sys
import config
from src.camera import Camera
from src.gesture_detector import GestureDetector
from src.wave_detector import WaveDetector
from src.motor_controller import MotorController
from src.navigation import NavigationController
from src.dance import DanceController
from src.treat_dispenser import TreatDispenser

# Conditionally import VPI processor
if config.USE_VPI_ACCELERATION:
    try:
        from src.vpi_processor import VPIProcessor
        vpi_processor = VPIProcessor(backend=config.VPI_BACKEND)
    except ImportError:
        print("VPI not available, continuing without GPU preprocessing")
        vpi_processor = None
else:
    vpi_processor = None


class RobotController:
    """Main robot control system"""
    
    def __init__(self):
        """Initialize all robot subsystems"""
        print("Initializing Duckiebot Science Fair Robot...")
        print("NVIDIA acceleration: ", end="")
        if config.USE_CUDA_ACCELERATION or config.USE_VPI_ACCELERATION:
            print("ENABLED")
        else:
            print("DISABLED")
        
        # Initialize components
        self.camera = Camera()
        self.gesture_detector = GestureDetector(
            min_detection_confidence=config.GESTURE_CONFIDENCE_THRESHOLD,
            min_tracking_confidence=config.GESTURE_CONFIDENCE_THRESHOLD
        )
        self.wave_detector = WaveDetector()
        self.motor_controller = MotorController()
        self.navigation = NavigationController()
        self.dance_controller = DanceController(self.motor_controller)
        self.treat_dispenser = TreatDispenser()
        
        # VPI processor for GPU-accelerated preprocessing
        self.vpi_processor = vpi_processor
        
        # State management
        self.running = False
        self.state = 'idle'  # idle, tracking, dancing
        self.frame_count = 0
        
        # Performance monitoring
        self.frame_times = []
        self.processing_times = []
        
        # Gesture tracking
        self.last_dance_gesture_time = 0
        self.last_treat_gesture_time = 0
        self.current_gesture_hold_time = 0
        self.current_gesture = None
        
        print("Robot initialized successfully!")
        if self.camera.has_cuda():
            print("  - CUDA acceleration: Available")
        if self.vpi_processor and self.vpi_processor.is_available():
            print("  - VPI acceleration: Available")
    
    def initialize(self):
        """Initialize camera and hardware"""
        if not self.camera.initialize():
            print("Failed to initialize camera")
            return False
        
        if not self.treat_dispenser.initialize():
            print("Warning: Treat dispenser initialization failed")
        
        return True
    
    def _preprocess_frame(self, frame):
        """
        Preprocess frame using GPU if available
        
        Args:
            frame: Input frame
            
        Returns:
            Preprocessed frame
        """
        if config.GPU_PREPROCESSING and self.vpi_processor and self.vpi_processor.is_available():
            # Optionally resize for faster processing (if needed)
            # frame = self.vpi_processor.resize_gpu(frame, self.camera.width, self.camera.height)
            pass  # Preprocessing can be added here if needed
        
        return frame
    
    def run(self):
        """Main control loop"""
        self.running = True
        frame_time_target = 1.0 / config.MAIN_LOOP_FPS
        
        print("Starting main control loop...")
        print("Press 'q' to quit, 's' to emergency stop")
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Capture frame
                success, frame = self.camera.read_frame()
                if not success:
                    print("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                self.frame_count += 1
                
                # GPU preprocessing (if enabled)
                if config.GPU_PREPROCESSING:
                    frame = self._preprocess_frame(frame)
                
                # Detect hands and gestures
                detection_start = time.time()
                hands_data, mp_results = self.gesture_detector.detect_hands(frame)
                detection_time = time.time() - detection_start
                
                # Update wave detector
                is_waving, wave_position = self.wave_detector.update(hands_data)
                
                # Handle state machine
                self._update_state(is_waving, wave_position, hands_data, frame)
                
                # Performance monitoring
                if config.ENABLE_PERFORMANCE_MONITORING:
                    loop_time = time.time() - loop_start
                    self.frame_times.append(loop_time)
                    self.processing_times.append(detection_time)
                    
                    if self.frame_count % 30 == 0:  # Every 30 frames
                        avg_frame_time = sum(self.frame_times[-30:]) / len(self.frame_times[-30:])
                        avg_detection_time = sum(self.processing_times[-30:]) / len(self.processing_times[-30:])
                        print(f"Performance: {1.0/avg_frame_time:.1f} FPS, Detection: {avg_detection_time*1000:.1f}ms")
                
                # Display output if enabled
                if config.DISPLAY_OUTPUT:
                    self._draw_overlay(frame, mp_results, is_waving, wave_position)
                    cv2.imshow('Duckiebot Science Fair Robot', frame)
                    
                    # Handle key presses
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("Quit requested")
                        break
                    elif key == ord('s'):
                        print("Emergency stop!")
                        self.motor_controller.emergency_stop()
                        self.state = 'idle'
                
                # Maintain target FPS
                elapsed = time.time() - loop_start
                if elapsed < frame_time_target:
                    time.sleep(frame_time_target - elapsed)
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.shutdown()
    
    def _update_state(self, is_waving, wave_position, hands_data, frame):
        """Update robot state based on sensor input"""
        
        # Check for dance gesture
        dance_gesture_detected = False
        treat_gesture_detected = False
        
        if hands_data:
            gesture = self.gesture_detector.classify_gesture(hands_data)
            
            if gesture == 'dance':
                if self.current_gesture == 'dance':
                    self.current_gesture_hold_time += (1.0 / config.MAIN_LOOP_FPS)
                    if self.current_gesture_hold_time >= config.DANCE_GESTURE_HOLD_TIME:
                        dance_gesture_detected = True
                else:
                    self.current_gesture = 'dance'
                    self.current_gesture_hold_time = 0
            elif gesture == 'treat':
                if self.current_gesture == 'treat':
                    self.current_gesture_hold_time += (1.0 / config.MAIN_LOOP_FPS)
                    if self.current_gesture_hold_time >= config.TREAT_GESTURE_HOLD_TIME:
                        treat_gesture_detected = True
                else:
                    self.current_gesture = 'treat'
                    self.current_gesture_hold_time = 0
            else:
                self.current_gesture = None
                self.current_gesture_hold_time = 0
        else:
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        
        # State machine logic
        if self.state == 'dancing':
            # Don't interrupt dance
            if not self.dance_controller.is_dance_in_progress():
                self.state = 'idle'
        elif treat_gesture_detected:
            # Handle treat gesture (future feature)
            if config.LOG_GESTURES:
                print("Secret treat gesture detected!")
            self.treat_dispenser.dispense_treat()
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        elif dance_gesture_detected:
            # Start dance
            self.state = 'dancing'
            self.motor_controller.stop()
            self.dance_controller.execute_dance()
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        elif is_waving and wave_position:
            # Track and navigate toward wave
            self.state = 'tracking'
            left_speed, right_speed = self.navigation.calculate_steering(wave_position)
            
            # Check if we should stop (target is close)
            if self.navigation.should_stop(wave_position):
                self.motor_controller.stop()
            else:
                self.motor_controller.set_differential_speed(left_speed, right_speed)
        else:
            # No target - stop and return to idle
            self.state = 'idle'
            self.motor_controller.stop()
    
    def _draw_overlay(self, frame, mp_results, is_waving, wave_position):
        """Draw overlay information on frame"""
        # Draw hand landmarks
        self.gesture_detector.draw_landmarks(frame, mp_results)
        
        # Draw state and information
        height, width = frame.shape[:2]
        
        # State indicator
        state_colors = {
            'idle': (128, 128, 128),
            'tracking': (0, 255, 0),
            'dancing': (255, 165, 0)
        }
        state_color = state_colors.get(self.state, (255, 255, 255))
        cv2.putText(frame, f"State: {self.state.upper()}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        
        # Acceleration indicator
        if self.camera.has_cuda():
            cv2.putText(frame, "CUDA", (width - 200, height - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if self.vpi_processor and self.vpi_processor.is_available():
            cv2.putText(frame, "VPI", (width - 200, height - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Wave detection indicator
        if is_waving:
            cv2.putText(frame, "WAVING DETECTED!", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw target position
            if wave_position:
                x, y = wave_position
                pixel_x = int(x * width)
                pixel_y = int(y * height)
                cv2.circle(frame, (pixel_x, pixel_y), 20, (0, 255, 0), 3)
        
        # Gesture indicator
        if self.current_gesture:
            gesture_text = f"Gesture: {self.current_gesture.upper()} ({self.current_gesture_hold_time:.1f}s)"
            cv2.putText(frame, gesture_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Frame count
        cv2.putText(frame, f"Frame: {self.frame_count}", (width - 150, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Instructions
        if not config.DISPLAY_OUTPUT:
            cv2.putText(frame, "Press 'q' to quit, 's' to stop", (10, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def shutdown(self):
        """Gracefully shutdown robot"""
        print("Shutting down robot...")
        self.running = False
        self.motor_controller.stop()
        time.sleep(0.2)
        self.motor_controller.cleanup()
        self.camera.release()
        self.gesture_detector.close()
        self.treat_dispenser.cleanup()
        if config.DISPLAY_OUTPUT:
            cv2.destroyAllWindows()
        print("Shutdown complete")


def signal_handler(sig, frame):
    """Handle interrupt signals"""
    print("\nReceived interrupt signal")
    rospy.signal_shutdown("Interrupted by user")
    sys.exit(0)


def main():
    """Main entry point"""
    # Initialize ROS node FIRST
    rospy.init_node('science_robot_controller', anonymous=True)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run robot controller
    robot = RobotController()
    
    if not robot.initialize():
        rospy.logerr("Failed to initialize robot")
        return 1
    
    try:
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

