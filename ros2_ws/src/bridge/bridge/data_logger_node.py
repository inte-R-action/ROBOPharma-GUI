import base64
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sqlite3
import traceback
import os
from datetime import datetime
import json
import re
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from custom_interfaces.msg import DetectionStatus, StateTimings, DetectionSetup


class DataLoggerNode(Node):
    """Logs detection results, timing summaries, and analysis images to SQLite database."""

    def __init__(self):
        """Initialise storage paths, subscriptions, and states."""
        super().__init__('data_logger_node')
        
        # Cache for latest received data
        self.latest_image = None
        self.latest_setup = None
        
        # Directory and database configuration
        self.log_dir = os.path.join(os.getcwd(), 'logs')
        self.image_dir = os.path.join(self.log_dir, 'images')
        os.makedirs(self.image_dir, exist_ok=True)
        self.db_path = os.path.join(self.log_dir, 'results.db')

        # State tracking for cycles and bags
        self.current_cycle = 1
        self.last_processed_bag = None
        self.last_logged_times = None
        self.current_bag_image_paths = {}

        # Initialise database schema and restore current cycle
        self.init_db()

        # Create QoS profile for best-effort image streaming
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to all relevant ROS 2 topics
        self.create_subscription(DetectionStatus, '/detection_status', self.detection_callback, 10)
        self.create_subscription(StateTimings, '/state_timings', self.timing_callback, 10)
        self.create_subscription(CompressedImage, '/vision_system/processed_image/compressed', 
                               self.save_image_callback, custom_qos)
        self.create_subscription(DetectionSetup, '/detection_setup', self.setup_callback, 10)
        self.create_subscription(String, '/current_position_step', self.analysis_callback, 10)

        # Error reporting publisher
        self.error_pub = self.create_publisher(String, '/system_errors', 10)
        self.get_logger().info(f"Data Logger Node started. Starting at Cycle: {self.current_cycle}")
    
    def _handle_error(self, error_code, message, exception):
        """Helper method to log and publish errors consistently.
        Args:
            error_code: Error identifier (e.g., 'DB_ERROR', 'IMAGE_CONVERT_ERROR')
            message: Human-readable error message
            exception: The exception that was caught
        """
        self.get_logger().error(f"{message}: {exception}")
        try:
            tb = traceback.format_exc()
            self.publish_system_error(error_code, message, details=tb)
        except Exception as e:
            self.get_logger().error(f"Failed to publish error {error_code}: {e}")
    
    def analysis_callback(self, msg):
        """
        Extracts bag and position numbers from the analysis message and saves the
        latest captured image with a descriptive filename.
        """
        self.get_logger().info(f"Received analysis message: {msg.data}")
        try:
            # Parse the analysis message JSON
            data = json.loads(msg.data)
            state = data.get("state")
            description = data.get("description")
            self.get_logger().info(f"Parsed state: '{state}', description: '{description}'")
            
            if state == "analyzing":
                # Ensure there is an image to save
                if self.latest_image is None:
                    self.get_logger().error("State is 'analyzing', but latest_image is NONE! (Is the camera topic running?)")
                    return

                # Extract bag number and position from description
                match = re.search(r'IV Bag (\d+) - Position (\d+)', description)
                if not match:
                    self.get_logger().warn(f"REGEX FAILED! Could not parse bag/position from: {description}")
                    return

                bag_num = int(match.group(1))
                pos_num = match.group(2)

                # Create safe filename from description
                safe_desc = description.replace(" - ", "_").replace(" ", "_").lower()
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"cycle_{self.current_cycle}_{safe_desc}_{timestamp}.jpg"
                filepath = os.path.join(self.image_dir, filename)

                # Decode base64 image data and save to file
                b64_data = self.latest_image.split(',')[1] if ',' in self.latest_image else self.latest_image
                image_bytes = base64.b64decode(b64_data)
                with open(filepath, 'wb') as image_file:
                    image_file.write(image_bytes)

                # Track image paths for database insertion
                if bag_num not in self.current_bag_image_paths:
                    self.current_bag_image_paths[bag_num] = {}
                self.current_bag_image_paths[bag_num][f"pos{pos_num}"] = filepath
                self.get_logger().info(f"SUCCESS: Saved analysis image for Bag {bag_num}, Pos {pos_num}: {filename}")
            else:
                self.get_logger().info("State is not 'analyzing', ignoring message.")
                
        except Exception as e:
            self._handle_error('UNHANDLED_EXCEPTION', 'Failed to process analysis message', e)

    def save_image_callback(self, msg):
        """
        Converts incoming compressed image messages to base64 for easy storage
        and later retrieval by the analysis callback.
        """
        try:
            b64_str = base64.b64encode(msg.data).decode('utf-8')
            self.latest_image = f'data:image/jpeg;base64,{b64_str}'
        except Exception as e:
            self._handle_error('IMAGE_CONVERT_ERROR', 'IV Bag Detection Image conversion failed', e)

    def init_db(self):
        """
        Initialises the SQLite schema with 'detections' and 'cycles' tables
        if they don't exist, and restores the current cycle counter from the
        maximum cycle present in the database.
        """
        with sqlite3.connect(self.db_path) as conn:
            cursor = conn.cursor()

            # Create detections table to store per-bag detection results
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    cycle INTEGER,
                    bag_number INTEGER,
                    timestamp TEXT,
                    status TEXT,
                    contamination_level TEXT,
                    recommendation TEXT,
                    particles INTEGER,
                    bubbles INTEGER,
                    confidence REAL,
                    image_path_pos1 TEXT,
                    image_path_pos2 TEXT,
                    detection_model TEXT,
                    setup_particle_threshold INTEGER,
                    setup_confidence_threshold REAL
                )
            ''')

            # Create cycles table to store aggregate timing metrics per cycle
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS cycles (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    load_time REAL,
                    conveyor_time REAL,
                    spin_time REAL,
                    detect_time REAL,
                    unload_time REAL,
                    process_time REAL
                )
            ''')

            # Restore cycle counter from max cycle in database
            cursor.execute("SELECT MAX(cycle) FROM detections")
            result = cursor.fetchone()
            self.current_cycle = result[0] + 1 if result and result[0] is not None else 1

    def publish_system_error(self, code, message, severity='WARN', details=None):
        """Publish compact JSON errors to `/system_errors` topic.
        Args:
            code: Error code identifier
            message: Human-readable error message
            severity: Severity level (default: 'WARN')
            details: Optional detailed traceback or additional context
        """
        try:
            time_str = datetime.now().isoformat() + 'Z'
            info_parts = [str(code) if code else '']
            if severity:
                info_parts.append(str(severity))
            if details is not None:
                details_str = str(details)
                info_parts.append(details_str[-1024:])  # Limit details to 1024 chars

            payload = {
                'time': time_str,
                'node': 'data_logger',
                'error': str(message),
                'information': ' | '.join([p for p in info_parts if p])
            }

            msg = String()
            msg.data = json.dumps(payload)
            self.error_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish system error: {e}')

    def timing_callback(self, msg):
        """
        Inserts cycle timing data into the database when all timing values are
        positive (indicating a completed cycle). Uses last_logged_times to avoid
        duplicate entries from repeated messages.
        """
        # Only log if all timing stages have positive values
        if (msg.load_time > 0.0 and msg.conveyor_time > 0.0 and 
            msg.spin_time > 0.0 and msg.detect_time > 0.0 and msg.unload_time > 0.0):

            current_times = (msg.load_time, msg.conveyor_time, msg.spin_time, 
                           msg.detect_time, msg.unload_time, msg.process_time)

            # Avoid duplicate entries from repeated identical messages
            if self.last_logged_times == current_times:
                return

            self.last_logged_times = current_times
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            try:
                # Insert cycle timing data into database
                with sqlite3.connect(self.db_path) as conn:
                    conn.execute(
                        '''
                        INSERT INTO cycles (
                            timestamp, load_time, conveyor_time,
                            spin_time, detect_time, unload_time, process_time
                        )
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                        ''',
                        (
                            timestamp,
                            msg.load_time,
                            msg.conveyor_time,
                            msg.spin_time,
                            msg.detect_time,
                            msg.unload_time,
                            msg.process_time,
                        ),
                    )

            except Exception as e:
                self._handle_error('DB_ERROR', 'Failed to log cycle timings', e)

    def setup_callback(self, msg):
        """
        Stores the current detection model and threshold settings to include
        in detection records for reproducibility.
        """
        self.latest_setup = msg

    def detection_callback(self, msg):
        """
        Inserts new detection records or updates existing ones with the latest
        analysis results, including particle/bubble counts, confidence scores,
        and image paths. Tracks cycle transitions when bag_number resets.
        """
        bag_number = msg.iv_bag_number
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Detect cycle transitions: when bag_number rolls back to 1
        if self.last_processed_bag is not None:
            if bag_number == 1 and self.last_processed_bag > 1:
                self.current_cycle += 1
                self.current_bag_image_paths.clear()

        self.last_processed_bag = bag_number

        # Calculate average confidence from both positions
        avg_conf = (msg.position1.avg_confidence + msg.position2.avg_confidence) / 2.0
        
        # Use latest setup values or defaults if not yet received
        model = self.latest_setup.detection_model if self.latest_setup else "Unknown"
        p_thresh = self.latest_setup.particle_threshold if self.latest_setup else 1
        c_thresh = self.latest_setup.confidence_threshold if self.latest_setup else 0.9985

        # Retrieve previously saved analysis images for this bag
        saved_paths = self.current_bag_image_paths.get(bag_number, {})
        image_path_1 = saved_paths.get("pos1", "")
        image_path_2 = saved_paths.get("pos2", "")

        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                # Check if detection record already exists for this cycle/bag
                cursor.execute(
                    'SELECT id FROM detections WHERE cycle = ? AND bag_number = ?',
                    (self.current_cycle, bag_number),
                )
                existing_row = cursor.fetchone()

                if existing_row:
                    # Update existing record with latest detection results
                    cursor.execute(
                        '''
                        UPDATE detections SET
                            timestamp = ?, status = ?, particles = ?, bubbles = ?,
                            confidence = ?, image_path_pos1 = ?, image_path_pos2 = ?,
                            contamination_level = ?, recommendation = ?, detection_model = ?,
                            setup_particle_threshold = ?, setup_confidence_threshold = ?
                        WHERE id = ?
                        ''',
                        (
                            timestamp,
                            msg.overall_status,
                            msg.total_particles,
                            msg.total_bubbles,
                            avg_conf,
                            image_path_1,
                            image_path_2,
                            msg.contamination_level,
                            msg.recommendation,
                            model,
                            p_thresh,
                            c_thresh,
                            existing_row[0],
                        ),
                    )
                else:
                    # Insert new detection record
                    cursor.execute(
                        '''
                        INSERT INTO detections (
                            cycle, bag_number, timestamp, status,
                            particles, bubbles, confidence, image_path_pos1, image_path_pos2,
                            contamination_level, recommendation, detection_model,
                            setup_particle_threshold, setup_confidence_threshold
                        )
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        ''',
                        (
                            self.current_cycle,
                            bag_number,
                            timestamp,
                            msg.overall_status,
                            msg.total_particles,
                            msg.total_bubbles,
                            avg_conf,
                            image_path_1,
                            image_path_2,
                            msg.contamination_level,
                            msg.recommendation,
                            model,
                            p_thresh,
                            c_thresh,
                        ),
                    )

        except Exception as e:
            self._handle_error('DB_ERROR', 'Failed to log bag detection', e)

def main(args=None):
    """Initialise and run the DataLoggerNode.
    
    Initialises the ROS 2 node, starts the spin loop, and cleanly shuts down
    on KeyboardInterrupt.
    
    Args:
        args: Optional command-line arguments passed to rclpy.init()
    """
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()