import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32
import json
import traceback
from datetime import datetime
from custom_interfaces.msg import (
    DetectionStatus, PositionStatus, DetectionSetup, SystemStatus, BagNumber, StateTimings
)


class LogicNode(Node):
    """
    Tracks the active machine stage, accumulates timing data for the current cycle,
    and republishes structured status messages for the UI and logging components.
    Orchestrates state transitions based on hardware feedback and detection results.
    """

    def __init__(self):
        """
        Sets up all topic subscriptions for hardware/detection feedback, creates
        output publishers for control commands and status messages, and initializes
        state tracking variables for the current inspection cycle.
        """
        super().__init__('logic_node')

        # Subscribe to hardware and detection feedback
        self.create_subscription(String, '/loading_status', self.loading_callback, 10)
        self.create_subscription(String, '/unloading_status', self.unloading_callback, 10)
        self.create_subscription(Float32, '/vision_system/processing_time', self.vision_callback, 10)
        self.create_subscription(String, '/bag_contamination_status', self.detection_callback, 10)
        self.create_subscription(Float32, '/current_position', self.spinning_callback, 10)

        # Publishers for control commands
        self.platform_trigger_pub = self.create_publisher(Int32, '/trigger_platform_move', 10)
        self.spinner_trigger_pub = self.create_publisher(Int32, '/spinner', 10)
        self.reset_simulation_pub = self.create_publisher(Int32, '/reset_simulation', 10)

        # Publishers for status and state information
        self.state_pub = self.create_publisher(SystemStatus, '/system_state', 10)
        self.detection_status_pub = self.create_publisher(DetectionStatus, '/detection_status', 10)
        self.detection_setup_pub = self.create_publisher(DetectionSetup, '/detection_setup', 10)
        self.bag_number_pub = self.create_publisher(BagNumber, '/bag_number', 10)
        self.state_timings_pub = self.create_publisher(StateTimings, '/state_timings', 10)
        self.test_result_pub = self.create_publisher(Int32, '/test_result', 10)
        self.error_pub = self.create_publisher(String, '/system_errors', 10)

        # Detection state tracking
        self.current_detection_state = DetectionStatus()
        self.current_detection_setup = DetectionSetup()
        self.current_detection_state.position1 = PositionStatus()
        self.current_detection_state.position2 = PositionStatus()

        # System state tracking
        self.current_system_state = SystemStatus()
        self.current_state_timings = StateTimings()

        # Timing and sequence state
        self.last_state_change = self.get_clock().now()
        self.process_start_time = None
        self.active_state_name = None
        self.total_bags = None
        self.prev_status = None
        self.last_unloaded_bag = None

    def _handle_error(self, error_code, message, exception=None):
        """Log and publish an error consistently.
        Args:
            error_code: Error identifier (e.g., 'JSON_DECODE_ERROR')
            message: Human-readable error message
            exception: Optional exception object for traceback extraction
        """
        self.get_logger().error(f"{message}" + (f": {exception}" if exception else ""))
        try:
            details = traceback.format_exc() if exception else None
            self.publish_system_error(error_code, message, details=details)
        except Exception as e:
            self.get_logger().error(f"Failed to publish error {error_code}: {e}")

    def _publish_bag_message(self, current_bag):
        """Publish current bag number and total bags count.
        
        Args:
            current_bag: Current bag number being processed
        """
        bags_msg = BagNumber()
        bags_msg.current_bag = current_bag
        bags_msg.total_bags = self.total_bags if self.total_bags is not None else 0
        self.bag_number_pub.publish(bags_msg)

    def _parse_position_data(self, position_key, position_data):
        """Extract and set detection data for a single position.
        
        Args:
            position_key: 'position1' or 'position2'
            position_data: Dictionary containing position-level detection results
        """
        position_obj = (self.current_detection_state.position1 
                       if position_key == "position_1" 
                       else self.current_detection_state.position2)
        
        position_obj.contaminated = position_data.get("is_contaminated", False)
        position_obj.needs_review = position_data.get("needs_review", False)
        position_obj.contamination_level = position_data.get("contamination_level", "Unknown")
        position_obj.total_particles = position_data.get("total_particles", 0)
        position_obj.total_bubbles = position_data.get("total_bubbles", 0)
        position_obj.avg_confidence = float(position_data.get("avg_confidence", 0.0))
        position_obj.analysis_duration = position_data.get("analysis_duration", 0.0)
        position_obj.timestamp = position_data.get("timestamp", "0")
        position_obj.statement = position_data.get("contamination_logic", "Unknown")
        
        # Extract thresholds if present
        if "thresholds_used" in position_data:
            self.current_detection_setup.particle_threshold = (
                position_data["thresholds_used"].get("particle_threshold", 0)
            )
            self.current_detection_setup.confidence_threshold = (
                position_data["thresholds_used"].get("confidence_threshold", 0.0)
            )



    def publish_system_error(self, code, message, severity='WARN', details=None):
        """Publish a structured JSON error to `/system_errors` topic.
        
        Args:
            code: Error code identifier
            message: Human-readable error message
            severity: Severity level (default: 'WARN')
            details: Optional detailed traceback or context (truncated to 1024 chars)
        """
        try:
            payload = {
                'timestamp': datetime.utcnow().isoformat() + 'Z',
                'source': 'logic_node',
                'code': code,
                'message': message,
                'severity': severity,
            }
            if details is not None:
                payload['details'] = str(details)[-1024:]

            msg = String()
            msg.data = json.dumps(payload)
            self.error_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish system error: {e}')


    def unloading_callback(self, msg):
        """
        Tracks bag unloading progress, publishes test results and bag numbers,
        and resets the system state when unloading completes. Handles cycle
        transitions by recording final timings and clearing state.
        """
        status = self.prev_status

        try:
            data = json.loads(msg.data)
            current_bag = data.get("current_bag")
            total_bags = data.get("total_bags")
            text_message = data.get("message", "")
            unloading_status = data.get("status", "")

            # Process in-progress bag unloading
            if (current_bag is not None and current_bag != 0 and 
                current_bag != self.last_unloaded_bag and unloading_status == "in_progress"):
                
                status = "unload"
                
                # Publish contamination status as test result
                test_result_msg = Int32()
                test_result_msg.data = 1 if "CONTAMINATED" in text_message.upper() else 0
                self.test_result_pub.publish(test_result_msg)

                # Update total bags if provided
                if total_bags is not None:
                    self.total_bags = total_bags

                # Publish bag information
                self._publish_bag_message(current_bag)
                self.last_unloaded_bag = current_bag

            # Handle cycle completion
            if data.get("status") == "complete":
                self.get_logger().info("Cycle complete - Resetting system logic state.")
                self.reset_simulation_pub.publish(Int32(data=1))

                # Record final timing for active state if one exists
                if self.active_state_name is not None:
                    now = self.get_clock().now()
                    diff = now - self.last_state_change
                    duration = diff.nanoseconds / 1e9
                    timing_field = f"{self.active_state_name}_time"

                    if hasattr(self.current_state_timings, timing_field):
                        current_val = getattr(self.current_state_timings, timing_field)
                        setattr(self.current_state_timings, timing_field, current_val + duration)

                    self.state_timings_pub.publish(self.current_state_timings)

                # Transition to idle (None) to properly reset all state flags
                self.update_system_state(None)
                
                # Reset all state variables
                self.active_state_name = None
                self.process_start_time = None
                self.prev_status = None
                self.total_bags = None
                self.last_unloaded_bag = None
                self.current_state_timings = StateTimings()
            
            # Update system state if status changed (only for non-complete transitions)
            elif status != self.prev_status:
                self.update_system_state("unload")
                self.prev_status = "unload"

        except json.JSONDecodeError as e:
            self._handle_error('JSON_DECODE_ERROR', 'Invalid JSON in unloading_callback', e)
        except Exception as e:
            self._handle_error('UNHANDLED_EXCEPTION', 'Error in unloading_callback', e)

    def loading_callback(self, msg):
        """
        Maps loading phase transitions to platform control commands and updates
        bag tracking. Handles state transitions from loading through conveyor
        movement and spinner triggering.
        """
        try:
            data = json.loads(msg.data)
            phase = data.get("phase")
            status = self.prev_status
            cmd = 0

            # Map loading phases to control commands and state transitions
            if phase == "motor_advancing":
                cmd = 1
                status = "conveyor"
            elif phase == "positioning":
                status = "load"
            elif phase == "so101_loading":
                status = "load"
            elif phase == "bag_complete":
                cmd = 2
                status = "load"
                self.spinner_trigger_pub.publish(Int32(data=1))
            elif phase == "complete":
                status = None
            else:
                cmd = 0

            # Send platform control command
            out_msg = Int32()
            out_msg.data = cmd
            self.platform_trigger_pub.publish(out_msg)

            # Update total bags on first occurrence
            if self.total_bags is None and "total_bags" in data:
                self.total_bags = data["total_bags"]

            # Publish bag information
            self._publish_bag_message(data.get("current_bag", 0))

            # Update system state if status changed
            if status != self.prev_status:
                self.update_system_state(status)
                self.prev_status = status
                
        except json.JSONDecodeError as e:
            self._handle_error('JSON_DECODE_ERROR', 'Invalid JSON in loading_callback', e)

    def vision_callback(self, msg):
        """
        Transitions to detection phase when the vision system has finished
        processing (indicated by non-zero processing time).
        """
        try:
            val = msg.data
            # Transition to detect phase when vision processing is complete (non-zero time)
            if val > 0.0 and self.active_state_name == "spin":
                self.update_system_state("detect")
        except Exception as e:
            self._handle_error('UNHANDLED_EXCEPTION', 'Error in vision_callback', e)
            
                
    def detection_callback(self, msg):
        """
        Processes contamination detection output from the vision system,
        updates all detection metrics and setup parameters, and publishes
        structured detection status for downstream consumers.
        """
        try:
            data = json.loads(msg.data)

            # Update bag number
            self.current_detection_state.iv_bag_number = data.get("iv_bag_number", 0)
            self._publish_bag_message(data.get("iv_bag_number", 0))

            # If this is the last bag, transition to idle
            if self.total_bags is not None and data.get("iv_bag_number") == self.total_bags:
                self.update_system_state(None)

            # Update detection model at system level
            self.current_detection_setup.detection_model = data.get("detection_model", "Unknown")

            # Update overall detection status
            self.current_detection_state.overall_status = data.get("overall_status", "Unknown")
            self.current_detection_state.contamination_level = data.get("contamination_level", "Unknown")
            self.current_detection_state.recommendation = data.get("recommendation", "Unknown")
            self.current_detection_state.needs_review = data.get("needs_review", False)
            self.current_detection_state.total_particles = data.get("total_particles", 0)
            self.current_detection_state.total_bubbles = data.get("total_bubbles", 0)

            # Parse position-specific detection results
            if "position_1" in data:
                self._parse_position_data("position_1", data["position_1"])
            if "position_2" in data:
                self._parse_position_data("position_2", data["position_2"])

            # Publish detection results
            self.detection_status_pub.publish(self.current_detection_state)
            self.detection_setup_pub.publish(self.current_detection_setup)

        except json.JSONDecodeError as e:
            self._handle_error('JSON_DECODE_ERROR', 'Invalid JSON in detection_callback', e)
        except Exception as e:
            self._handle_error('UNHANDLED_EXCEPTION', 'Error in detection_callback', e)

    def spinning_callback(self, msg):
        """Transition to spin phase when spinner starts moving.
        
        Triggered when the spinner begins rotating. Initiates the spin phase
        unless already in a later phase (spin, detect, or unload).
        """
        try:
            position = msg.data
            # Only transition to spin if not already in or past this phase
            if position > 0.0 and self.active_state_name not in ["spin", "detect", "unload"]:
                self.update_system_state("spin")
        except Exception as e:
            self._handle_error('UNHANDLED_EXCEPTION', 'Error in spinning_callback', e)

    def update_system_state(self, active_key):
        """
        Calculates duration of the ending state, updates timing totals, transitions
        to the new state, and publishes status updates. Handles state sequence
        validation and ensures monotonic time tracking.
        
        Args:
            active_key: New state name (e.g., 'load', 'spin', 'detect', 'unload')
                       or None to indicate idle/complete state
        """
        now = self.get_clock().now()
        # Normalise state name to lowercase
        new_state = active_key.lower() if isinstance(active_key, str) else None

        # Ignore redundant state transitions
        if new_state == self.active_state_name:
            return

        # Record duration of the state that is ending
        if self.active_state_name is not None and self.last_state_change is not None:
            diff = now - self.last_state_change
            duration = diff.nanoseconds / 1e9
            timing_field = f"{self.active_state_name}_time"
            
            if hasattr(self.current_state_timings, timing_field):
                current_total = getattr(self.current_state_timings, timing_field)
                setattr(self.current_state_timings, timing_field, current_total + duration)
                self.get_logger().info(f"Finished {self.active_state_name}: {duration:.2f}s")

        # Start process timer on first non-idle state
        if self.process_start_time is None and new_state is not None:
            self.process_start_time = now
        
        # Update total process/cycle time
        if self.process_start_time is not None:
            total_duration = (now - self.process_start_time).nanoseconds / 1e9
            self.current_state_timings.process_time = total_duration

        # Update state tracking variables
        self.active_state_name = new_state
        self.last_state_change = now

        # Update SystemStatus boolean flags (only one state active at a time)
        valid_fields = self.current_system_state.get_fields_and_field_types()
        for field in valid_fields:
            setattr(self.current_system_state, field, (field == new_state))

        # Publish state updates
        self.state_timings_pub.publish(self.current_state_timings)
        self.state_pub.publish(self.current_system_state)
        
        # Log state transition
        if new_state:
            self.get_logger().info(f"System transitioning to: {new_state.upper()}")
        else:
            self.get_logger().info("System entering IDLE (None)")

    def reset_internal_state(self):
        """
        Resets all detection state, system state, and timing variables to default
        values and publishes these resets to ensure downstream systems receive
        consistent initialisation signals on restart. Safe to call even if
        partial initialisation has occurred.
        """
        try:
            # Reset detection state objects
            self.current_detection_state = DetectionStatus()
            self.current_detection_setup = DetectionSetup()
            self.current_detection_state.position1 = PositionStatus()
            self.current_detection_state.position2 = PositionStatus()

            # Reset system state to all-false by transitioning to idle
            self.current_system_state = SystemStatus()
            self.update_system_state(None)

            # Reset timing state
            self.current_state_timings = StateTimings()

            # Reset tracking variables
            self.last_state_change = None
            self.process_start_time = None
            self.active_state_name = None
            self.total_bags = None
            self.prev_status = None
            self.last_unloaded_bag = None

            # Publish reset state
            try:
                self.state_timings_pub.publish(self.current_state_timings)
            except Exception:
                pass
            try:
                self.state_pub.publish(self.current_system_state)
            except Exception:
                pass
            self.get_logger().info('Internal state reset for shutdown.')
        except Exception as e:
            self.get_logger().error(f'Failed to reset internal state: {e}')
        

def main(args=None):
    """
    Initialises the ROS 2 node, starts the spin loop to process messages,
    and ensures clean shutdown with state reset on termination.
    
    Args:
        args: Optional command-line arguments passed to rclpy.init()
    """
    rclpy.init(args=args)
    node = LogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up state before shutdown
        try:
            node.reset_internal_state()
        except Exception:
            pass
        try:
            node.publish_system_error('NODE_SHUTDOWN', 'LogicNode shutting down', severity='FATAL')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

