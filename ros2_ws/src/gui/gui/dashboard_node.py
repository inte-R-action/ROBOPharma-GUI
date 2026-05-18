"""NiceGUI dashboard for the ROS 2 inspection system.

This module bridges ROS 2 topics, services, and the web UI. The dashboard
displays live system state, detection metrics, simulation imagery, and report
downloads. 

Architecture:
1. ROS callbacks populate shared state on the ros_node instance
2. Periodic UI timers read that state and refresh page widgets
3. User actions call ROS services or generate reports from SQLite records

State is centralised in one node instance so the UI can poll it without
additional networking.
"""

import threading
import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from nicegui import app, ui
import os
import json
import time
import csv
from pathlib import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import base64
from custom_interfaces.msg import SystemStatus, DetectionStatus, DetectionSetup, BagNumber, StateTimings
import sqlite3
from datetime import datetime, timedelta
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Image as RLImage, Spacer
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib import colors
from std_srvs.srv import Trigger


static_dir = '/home/roberta/ros2_ws/src/gui/static'
script_dir = os.path.dirname(os.path.abspath(__file__))

if not os.path.exists(static_dir):
    static_dir = os.path.join(os.path.dirname(script_dir), 'static')

if not os.path.exists(static_dir):
    print(f"CRITICAL: Static directory not found. Checked: {static_dir}")
else:
    print(f"Success: Mapping /static to {static_dir}")
    app.add_static_files('/static', static_dir)

ros_node = None

class MyGuiNode(Node):
    """
    The node subscribes to system status, detection results, images, timing
    updates, and system errors. Callbacks populate attributes that the NiceGUI
    timers read periodically for UI refresh without blocking on callbacks.
    """

    def __init__(self):
        """
        Sets up topic subscriptions for all system feedback channels (hardware state,
        detection results, timing, errors) and creates publishers for model selection.
        Caches for UI state are initialised as None, and pass/fail counters are reset to 0.
        """
        super().__init__('dashboard_node')

        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(CompressedImage, '/vision_system/processed_image/compressed', self.detection_image_callback, custom_qos)
        self.create_subscription(SystemStatus, '/system_state', self.system_state_callback, 10)
        self.create_subscription(DetectionStatus, '/detection_status', self.detection_callback, 10)
        self.create_subscription(DetectionSetup, '/detection_setup', self.detection_setup_callback, 10)
        self.create_subscription(BagNumber, '/bag_number', self.bag_number_callback, 10)
        self.create_subscription(StateTimings, '/state_timings', self.state_timing_callback, 10)
        self.create_subscription(CompressedImage, '/simulation_view', self.compressed_image_callback, custom_qos)
        self.create_subscription(String, '/vision_system/current_model', self.model_callback, 10)
        self.create_subscription(String, '/system_errors', self.system_error_callback, 10)

        self.cycle_client = self.create_client(Trigger, '/switch_model')

        # Image and state caching
        self.latest_image_b64 = None
        self.latest_sim_b64 = None
        self.latest_det = None
        self.latest_det_arrival = None
        
        # Error tracking
        self.error_messages = deque(maxlen=20)
        self.latest_system_error = None
        self.system_error_seq = 0
        self.system_error_visible_until = 0.0
        self.errors_txt_path = get_errors_txt_path()
        
        # System state tracking
        self.current_state = None
        self.total_pass = 0
        self.total_fail = 0
        self.last_bag_id = -1
        self.latest_setup = None
        self.timings = None
        self.current_model = "faster_rcnn"
        self.bag_msg = None
        
        # Detection callback timing tracking
        self.csv_filename = os.path.expanduser('~/gui_detection_latencies.csv')
        self.setup_csv()
        
        self.get_logger().info("ROS 2 GUI Node initialised")


    def model_callback(self, msg):
        """Track the currently active detection model for the settings page."""
        self.current_model = msg.data

    def setup_csv(self):
        """
        Initialises a CSV with headers tracking pipeline timing from detection
        through UI rendering for performance analysis.
        """
        file_exists = os.path.isfile(self.csv_filename)
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow([
                    'Bag_ID',
                    'Total_Time_To_UI_Render',
                    'Network_Logic_to_GUI',
                    'GUI_Render_Time',
                    'Detection_Status',
                    'Timestamp'
                ])
        self.get_logger().info(f"Detection latency logging: {self.csv_filename}")

    def trigger_parameter_cycle(self):
        """
        Debug note: this call is intentionally guarded by ``service_is_ready``
        so the UI can report a clean failure instead of hanging on a call to an
        unavailable service.
        """
        # Check if the service is actually online before calling
        if not self.cycle_client.service_is_ready():
            self.get_logger().warn('Cannot cycle parameter: /topic service is offline.')
            return False

        self.get_logger().info('Sending request to cycle parameter...')
        req = Trigger.Request()
        future = self.cycle_client.call_async(req)
        future.add_done_callback(self.handle_cycle_response)
        return True

    def handle_cycle_response(self, future):
        """Log the result of the asynchronous cycle request."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Parameter cycled! Server said: "{response.message}"')
            else:
                self.get_logger().warn(f'Service failed: "{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Service call completely failed: {e}')

    def compressed_image_callback(self, msg):
        """Convert a compressed simulation frame into a browser-friendly URI."""
        try:
            b64_str = base64.b64encode(msg.data).decode('utf-8')
            self.latest_sim_b64 = f'data:image/jpeg;base64,{b64_str}'
        except Exception as e:
            self.get_logger().error(f'Sim image conversion failed: {e}')

    def detection_image_callback(self, msg):
        """Convert the processed detection image into a browser-friendly URI."""
        try:
            b64_str = base64.b64encode(msg.data).decode('utf-8')
            self.latest_image_b64 = f'data:image/jpeg;base64,{b64_str}'
        except Exception as e:
            self.get_logger().error(f'IV Bag Image conversion failed: {e}')


    def system_state_callback(self, msg):
        """
        Extracts the currently active state from the SystemStatus message by
        finding the single True-valued boolean field. Resets counters when
        returning to idle (all false).
        """
        fields = msg.get_fields_and_field_types()
        active_state_found = False

        # Find the active (True) state
        for field in fields:
            if getattr(msg, field) is True:
                self.current_state = field.lower()
                active_state_found = True
                break

        # If no state is active, system is idle
        if not active_state_found:
            self.current_state = None
            self.last_bag_id = -1

        self.get_logger().info(f"System state: {self.current_state or 'IDLE'}")

    def detection_callback(self, msg):
        """
        Records the latest detection output, updates pass/fail counters on new bags,
        and logs pipeline timing for performance analysis. Avoids updating counters
        for duplicate messages from the same bag.
        """
        self.latest_det = msg
        # Update pass/fail counters only once per bag
        if msg.iv_bag_number != self.last_bag_id:
            if msg.overall_status.lower() == 'contaminated':
                self.total_fail += 1
            elif msg.overall_status.lower() == 'clean':
                self.total_pass += 1
            self.last_bag_id = msg.iv_bag_number         

    def detection_setup_callback(self, msg):
        """Cache the current detection setup parameters for UI display."""
        self.latest_setup = msg
        self.get_logger().info(f"Detection Setup: Model={msg.detection_model}")

    def bag_number_callback(self, msg):
        """Store the active bag index and total bag count."""
        self.bag_msg = msg
        self.get_logger().info(
            f"Bag: {msg.current_bag}/{msg.total_bags}"
        )

    def state_timing_callback(self, msg):
        """Cache the latest timing summary for the home dashboard bars."""
        self.timings = msg

    def system_error_callback(self, msg):
        """
        Logs errors to a text file for archival and updates the latest error
        for display in the status indicator. Errors are cached with a visibility
        timeout for the UI to display them.
        """
        # Save error in text file
        try:
            errors_dir = os.path.dirname(self.errors_txt_path)
            os.makedirs(errors_dir, exist_ok=True)
            with open(self.errors_txt_path, 'a', encoding='utf-8') as error_file:
                error_file.write(msg.data)
                error_file.write('\n')
        except Exception as e:
            self.get_logger().error(f'Failed to write /system_errors to text file: {e}')

        # Cache error for UI display
        try:
            payload = json.loads(msg.data)
            severity = str(payload.get('severity', 'None')).upper()

            if severity == 'NONE':
                self.latest_system_error = None
                self.system_error_visible_until = 0.0
            else:
                self.latest_system_error = payload
                self.system_error_visible_until = time.time() + 5.0
        except Exception:
            # Fallback for malformed error messages
            self.latest_system_error = {
                'time': datetime.now().isoformat() + 'Z',
                'node': 'unknown',
                'error': 'Failed to parse /system_errors payload',
                'information': str(msg.data),
            }
            self.system_error_visible_until = time.time() + 5.0

        self.system_error_seq += 1


def build_sidebar():
    """Create the navigation bar used on every page."""
    with ui.left_drawer(value=True, fixed=True).classes('custom-sidebar').props('width=120'):
        ui.button(icon='home', on_click=lambda: ui.navigate.to('/')).props('flat').classes('nav-btn q-mb-md')
        ui.button(icon='camera', on_click=lambda: ui.navigate.to('/detection')).props('flat').classes('nav-btn q-mb-md')
        ui.button(icon='insert_chart', on_click=lambda: ui.navigate.to('/metrics')).props('flat').classes('nav-btn q-mb-md')
        ui.button(icon='precision_manufacturing', on_click=lambda: ui.navigate.to('/simulation')).props('flat').classes('nav-btn q-mb-md')
        ui.button(icon='download', on_click=lambda: ui.navigate.to('/downloads')).props('flat').classes('nav-btn q-mb-md')
        ui.element('div').classes('flex-grow') 
        ui.button(icon='settings', on_click=lambda: ui.navigate.to('/settings')).props('flat').classes('nav-btn')

def build_status_indicator(node=None):
    """Create the top-right status indicator."""
    node = node or ros_node

    with ui.element('div').classes('fixed top-4 right-4 z-50 flex items-center gap-2'):
        error_label = ui.label('').classes('text-red-600 font-extrabold text-6xl bg-white px-3 py-2 border-2 border-red-600 rounded')
        error_label.set_visibility(False)

        indicator = ui.element('div').classes('rounded-full shadow-lg border-2 border-black') \
            .style('width: 65px; height: 65px; background-color: #9e9e9e; cursor: pointer;')

    last_seen_error_seq = {'value': -1}
    current_error = {'payload': None}
    error_pinned = {'value': False}

    def get_indicator_color(payload):
        severity = str(payload.get('severity', 'WARN')).upper()
        if severity == 'ERROR' or severity == 'FATAL':
            return '#f44336'
        return '#facc15'

    def show_error(payload, pin=False):
        current_error['payload'] = payload
        error_pinned['value'] = pin

        severity = str(payload.get('severity', 'WARN')).upper()
        error_text = str(payload.get('error') or payload.get('message') or 'System error')
        prefix = 'Warning' if severity == 'WARN' else 'ERROR'
        error_label.set_text(f"{prefix}: {error_text}")
        error_label.set_visibility(True)
        indicator.style(f"background-color: {get_indicator_color(payload)};")

    def hide_error():
        error_label.set_visibility(False)
        error_pinned['value'] = False

    def update_status():
        """Maintains connectivity status."""
        is_connected = rclpy.ok() 

        if not is_connected:
            error_label.set_text("ERROR: No connection to rig")
            error_label.set_visibility(True)
            indicator.style('background-color: #f44336;')
        elif node and getattr(node, 'latest_system_error', None):
            current_error['payload'] = node.latest_system_error
            indicator.style(f"background-color: {get_indicator_color(node.latest_system_error)};")

            if node.system_error_seq != last_seen_error_seq['value']:
                last_seen_error_seq['value'] = node.system_error_seq
                show_error(node.latest_system_error, pin=False)
            elif error_pinned['value']:
                if not error_label.visible:
                    show_error(node.latest_system_error, pin=True)
            elif time.time() < getattr(node, 'system_error_visible_until', 0.0):
                if not error_label.visible:
                    show_error(node.latest_system_error, pin=False)
            else:
                hide_error()
        else:
            current_error['payload'] = None
            hide_error()
            indicator.style('background-color: #4caf50;')

    def toggle_error_visibility():
        """Show or hide the most recent system error from the status circle."""
        if not rclpy.ok():
            return

        if error_label.visible:
            hide_error()
            return

        if current_error['payload'] is not None:
            show_error(current_error['payload'], pin=True)

    indicator.on('click', toggle_error_visibility)

    ui.timer(0.2, update_status)


def get_results_db_path():
    """Return the SQLite database used for detection and cycle history."""
    candidates = [
        Path.home() / 'ros2_ws' / 'logs' / 'results.db',
        Path.home() / 'logs' / 'results.db',
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return str(candidates[0])


def get_errors_txt_path():
    """Return the text file used to append raw /system_errors messages."""
    candidates = [
        Path.home() / 'ros2_ws' / 'logs' / 'errors.txt',
        Path.home() / 'logs' / 'errors.txt',
    ]

    for candidate in candidates:
        log_dir = candidate.parent
        if log_dir.exists():
            return str(candidate)

    return str(candidates[0])


# Home page
@ui.page('/')
def main_page():
    """
    The left side focuses on performance and stage timing, while the right
    side shows the current stage, bag progress, and live simulation imagery.
    """
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    ui.add_head_html('<style>.nicegui-content { padding: 0 !important; }</style>')
    build_sidebar()
    build_status_indicator()

    timing_ui_elements = {}
    stage_elements = {}

    # Main Layout
    with ui.row().classes('w-full h-screen max-h-screen no-wrap p-6 bg-white gap-8 overflow-hidden'):
    
        # Left column - Performance and Times 
        with ui.column().classes('w-[40%] h-full gap-6'):
            ui.label('Performance').classes('text-6xl font-bold self-center mb-2')
            
            # 2x2 Grid for metrics - Bags Processed, Pass Rate, Process Time, Failed
            with ui.grid(columns=2).classes('w-full gap-6'):
                with ui.element('div').classes('metric-block'):
                    ui.label('Bags Processed').classes('text-3xl')
                    bags_processed_label = ui.label('0').classes('text-6xl font-bold mt-2')
                with ui.element('div').classes('metric-block'):
                    ui.label('Pass Rate').classes('text-3xl')
                    pass_rate_label = ui.label('0%').classes('text-6xl font-bold mt-2')
                with ui.element('div').classes('metric-block'):
                    ui.label('Process Time').classes('text-3xl')
                    total_process_time_label = ui.label('0s').classes('text-6xl font-bold mt-2')
                with ui.element('div').classes('metric-block'):
                    ui.label('Failed').classes('text-3xl')
                    failed_label = ui.label('0').classes('text-6xl font-bold mt-2')
            
            # Times
            with ui.element('div').classes('metric-block flex-grow w-full items-start justify-start p-4'):
                ui.label('Times').classes('text-6xl font-bold w-full text-center mb-2')
                
                def adaptive_time_row(name):
                    """Build one timing row with a bar, absolute time, and percentage."""
                    with ui.column().classes('w-full gap-0 mb-4'):
                        with ui.row().classes('w-full items-center justify-between no-wrap'):
                            ui.label(name).classes('font-bold text-4xl')
                            
                            with ui.row().classes('items-center gap-2'):
                                val_label = ui.label('0.0s').classes('text-3xl font-bold text-[#0d0d5c]')
                                percent_label = ui.label('(0%)').classes('text-3xl')
                    
                        bar = ui.linear_progress(value=0, show_value=False).classes('w-full h-6').props('rounded color=light-blue-4')
                            
                        timing_ui_elements[name.lower()] = {
                            'bar': bar, 
                            'label': val_label,
                            'percent': percent_label
                        }

                for state in ['Load', 'Conveyor', 'Spin', 'Detect', 'Unload']:
                    adaptive_time_row(state)
        # Right column
        with ui.column().classes('flex-grow h-full gap-6'):

            with ui.row().classes('w-full justify-between items-center px-2 pr-16'):
                ui.label('Stage').classes('text-6xl font-bold')
                bag_count_display = ui.label('IV Bag 0/0').classes('text-6xl font-bold')

            with ui.row().classes('w-full justify-between items-center bg-transparent py-4 no-wrap gap-2'):
                stages = ['Load', 'Conveyor', 'Spin', 'Detect', 'Unload']
                
                for i, name in enumerate(stages):
                    stage_elements[name.lower()] = ui.label(name).classes('stage-box flex-1 flex-shrink-0 text-center') \
                        .style('font-size: 38px !important; color: #000;') 
                    
                    if i < len(stages) - 1:
                        ui.icon('arrow_forward').classes('text-4xl flex-none')

            # Simulation view
            with ui.element('div').classes('sim-placeholder flex-grow w-full overflow-hidden bg-black'):
                # Change scale to adjust zoom of image
                sim_view = ui.interactive_image().classes('w-full h-full object-contain scale-160')
    
    def update_home_ui():
        """Refresh the home dashboard from the latest ROS node snapshot."""
        if not ros_node: return
       
        if ros_node.timings is not None:
            t = ros_node.timings
            mapping = {
                'load':     t.load_time,
                'conveyor': t.conveyor_time,
                'spin':     t.spin_time,
                'detect':   t.detect_time,
                'unload':   t.unload_time
            }

            total_process_time = t.process_time
            total_process_time_label.set_text(f"{total_process_time:.1f}s")

            for key, stage_time in mapping.items():
                if key in timing_ui_elements:
                    # Calculate the proportion only after the total is known to avoid division errors.
                    progress = (stage_time / total_process_time) if total_process_time > 0 else 0
                    
                    # Update the bar 
                    timing_ui_elements[key]['bar'].set_value(progress)
                    
                    # Update the time label
                    timing_ui_elements[key]['label'].set_text(f"{stage_time:.1f}s")
                    
                    # Update the percentage label 
                    percentage_int = int(progress * 100)
                    timing_ui_elements[key]['percent'].set_text(f"({percentage_int}%)")
        
        # Set state colours
        active_state = getattr(ros_node, 'current_state', 'load')
        for name, label_el in stage_elements.items():
            if name == active_state:
                # Active: Brighter Blue 
                label_el.style('background-color: #5BC0F8 !important; color: #000 !important; font-size: 38px !important;')
                label_el.classes('scale-105 shadow-md')
            else:
                # Inactive : Light Blue 
                label_el.style('background-color: #CCEEFF !important; color: #000 !important; font-size: 38px !important;')
                label_el.classes(remove='scale-105 shadow-md')
    
        if ros_node.latest_sim_b64:
            sim_view.set_source(ros_node.latest_sim_b64)
            ros_node.latest_sim_b64 = None 
        
        # Update metrics
        total_bags = ros_node.total_pass + ros_node.total_fail
        bags_processed_label.set_text(str(total_bags))
        failed_label.set_text(str(ros_node.total_fail))
        rate = (ros_node.total_pass / total_bags * 100) if total_bags > 0 else 0
        pass_rate_label.set_text(f"{int(rate)}%")

        if hasattr(ros_node, 'bag_msg'):
            bag_count_display.set_text(f"IV Bag {ros_node.bag_msg.current_bag}/{ros_node.bag_msg.total_bags}")

    # Poll ROS state frequently so the dashboard feels live without requiring manual refresh.
    ui.timer(0.1, update_home_ui)



@ui.page('/detection')
def detection_page():
    """
    This page focuses on detection results for each IV bag: summary status, metrics,
    setup parameters, and the live camera feed used during detection.
    """
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    
    # Sidebar + status indicator
    build_sidebar()
    build_status_indicator()
    
    # Main layout
    with ui.row().classes('w-full h-screen max-h-screen no-wrap p-6 bg-white gap-4 overflow-hidden'):
        
        # Left side
        with ui.column().classes('w-2/5 h-full no-wrap gap-4'):
            
            # Detection metrics 
            with ui.column().classes('metrics-panel w-full p-6 shadow-md'):
                with ui.row().classes('w-full justify-between items-center mb-4'):
                    id_label = ui.label('ID : --').classes('text-6xl font-bold')
                    status_badge = ui.label('WAIT').classes('text-6xl fail-badge')
                
                # Pie chart for pass/fail
                with ui.row().classes('w-full items-center mb-6'):  
                    # ECharts is fed with live counts below; the chart starts empty and is updated by the timer.
                    chart = ui.echart({
                            'tooltip': {'trigger': 'item'},
                            'series': [{
                                'type': 'pie',
                                'radius': ['40%', '80%'], 
                                'avoidLabelOverlap': False,
                                'label': {'show': False},
                                'emphasis': {'label': {'show': False}},
                                'data': [
                                    {'value': 0, 'name': 'Pass', 'itemStyle': {'color': '#22C55E'}},
                                    {'value': 0, 'name': 'Fail', 'itemStyle': {'color': '#FF0000'}},
                                ]
                            }]
                            }).style('width: 150px; height: 150px;')    
                    with ui.column().classes('ml-auto items-end'):
                        with ui.row().classes('gap-4'):
                            with ui.column().classes('items-center'):
                                ui.label('Pass:').classes('text-3xl')
                                pass_count = ui.label('0').classes('text-5xl font-bold')
                            with ui.column().classes('items-center'):
                                ui.label('Fail:').classes('text-3xl')
                                fail_count = ui.label('0').classes('text-5xl font-bold')

                def create_metric(label_text):
                    with ui.row().classes('w-full justify-between items-baseline mb-2'):
                        ui.label(label_text).classes('text-3xl')
                        return ui.label('--').classes('text-5xl font-bold')
                
                # Create all metric labels and store references for updates
                conf_val = create_metric('Confidence:')
                contam_val = create_metric('Contamination:')
                part_val = create_metric('Particles:')
                bub_val = create_metric('Bubbles:')
                time_val = create_metric('Analysis Time:')

            # Setup information 
            with ui.column().classes('metrics-panel w-full p-6 rounded-2xl shadow-md'):
                ui.label('Information').classes('text-6xl font-bold mb-4')
                
                with ui.row().classes('w-full justify-between items-center mb-2'):
                    ui.label('Model:').classes('text-3xl')
                    model_val = ui.label('--').classes('text-4xl font-bold')
                    
                with ui.row().classes('w-full justify-between items-center mb-2'):
                    ui.label('Particle Threshold:').classes('text-3xl')
                    part_thresh_val = ui.label('--').classes('text-4xl font-bold')
                    
                with ui.row().classes('w-full justify-between items-center'):
                    ui.label('Confidence Threshold:').classes('text-3xl')
                    conf_thresh_val = ui.label('--').classes('text-4xl font-bold')
        

        # Right detection camera panel
        with ui.column().classes('w-full h-full items-center justify-center bg-transparent overflow-hidden'):
            with ui.card().classes('card-camera shadow-0 w-full flex-grow overflow-hidden'):
                    camera_display = ui.interactive_image().classes('w-full h-full')
                    ui.html('<img id="camera_stream" style="width:100%; height:100%; object-fit:contain;">')    
            
         

    def update_det_ui():
        """Refresh the detection page from the latest ROS node data."""
        if not ros_node:
            return
        
        # Update global counters
        pass_count.set_text(str(ros_node.total_pass))
        fail_count.set_text(str(ros_node.total_fail))
        
        # Update the pie chart data if any new bags have been processed
        if (ros_node.total_pass + ros_node.total_fail) > 0:
            chart.options['series'][0]['data'][0]['value'] = ros_node.total_pass
            chart.options['series'][0]['data'][1]['value'] = ros_node.total_fail
            chart.update()

        # Update bag ID from bag_number_callback
        if hasattr(ros_node, 'bag_msg'):
            id_label.set_text(f"ID : {ros_node.bag_msg.current_bag}")

        # Update detection metrics
        if hasattr(ros_node, 'latest_det') and ros_node.latest_det:
            d = ros_node.latest_det
            
            # Overall status 
            is_clean = d.overall_status.lower() == 'clean'

            if is_clean:
                status_badge.set_text('PASS')
                # Remove the fail color, add the pass color
                status_badge.classes(remove='fail-badge', add='pass-badge')
            else:
                status_badge.set_text('FAIL')
                # Remove the pass color, add the fail color
                status_badge.classes(remove='pass-badge', add='fail-badge')
            
            # Contamination data
            part_val.set_text(str(d.total_particles))
            bub_val.set_text(str(d.total_bubbles))
            contam_val.set_text(d.contamination_level.upper()) 
            avg_time = (d.position1.analysis_duration + d.position2.analysis_duration) / 2
            time_val.set_text(f"{avg_time:.1f}s")
            
            # Confidence calculation (average of both positions)
            avg_conf = (d.position1.avg_confidence + d.position2.avg_confidence) / 2
            conf_val.set_text(f"{avg_conf:.3f}%")
        
        if hasattr(ros_node, 'latest_setup') and ros_node.latest_setup:
            s = ros_node.latest_setup
            model_val.set_text(s.detection_model)
            part_thresh_val.set_text(str(s.particle_threshold))
            conf_thresh_val.set_text(f"{s.confidence_threshold:.4f}")

        # Update camera 
        if ros_node.latest_image_b64:
            camera_display.set_source(ros_node.latest_image_b64)
            ros_node.latest_image_b64 = None

    # Refresh the UI every 100ms
    ui.timer(0.1, update_det_ui)




@ui.page('/metrics')
def metrics_page():
    """Render the detailed metrics and analysis dashboard."""
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    ui.add_head_html('<style>.nicegui-content { padding: 0 !important; }</style>')
    
    build_sidebar()
    build_status_indicator()

    # Main Layout 
    with ui.column().classes('w-full h-screen max-h-screen no-wrap p-6 bg-white gap-6 overflow-hidden'):
    
        # Header Area with the new Time Filter
        with ui.row().classes('w-full shrink-0 justify-between items-center'): 
            ui.label('Metrics').classes('text-6xl font-bold')
            
            with ui.row().classes('items-center gap-4 bg-[#cceeff] px-8 py-2 rounded-2xl shadow-sm mr-20'):
                ui.label('Timeframe:').classes('text-4xl font-bold')
                
                time_filter = ui.toggle(
                    {1: '1 Day', 7: '1 Week', 30: '1 Month'}, 
                    value=7, 
                    on_change=lambda: fetch_and_update_chart()
                ).classes('font-bold').props('rounded size="28px" unelevated dense color="white" text-color="black" active-color="primary"')
                
        # Grid Area
        with ui.grid(columns=2).classes('w-full flex-grow gap-6 grid-rows-2'):
            
            # Top-Left Quadrant: Pass/Fail Line Chart
            with ui.element('div').classes('metric-block flex flex-col p-2 border-8 border-[#cceeff] rounded-xl bg-white shadow-sm h-full w-full overflow-hidden min-w-0'):
                ui.label('Pass/Fail Trend per Cycle').classes('text-5xl font-bold mb-2 text-gray-800 shrink-0')
                with ui.element('div').classes('relative w-full flex-grow min-h-0 min-w-0'):
                    
                    pass_fail_chart = ui.echart({
                        'tooltip': {'trigger': 'axis', 'textStyle': {'fontSize': 30, 'fontWeight': 'bold'}}, 
                        'legend': {'data': ['Pass', 'Fail'], 'top': 0, 'right': 10, 'textStyle': {'fontSize': 35, 'fontWeight': 'bold'}},
                        'grid': {'left': '6%', 'right': '5%', 'bottom': '8%', 'top': '15%', 'containLabel': True},
                        'dataZoom': [{'type': 'inside'}], 
                        'xAxis': {
                            'type': 'category', 'name': 'Time', 'nameLocation': 'middle', 
                            'nameGap': 100, 
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 24, 'fontWeight': 'bold', 'lineHeight': 30}, 
                            'data': [] 
                        },
                        'yAxis': {
                            'type': 'value', 'name': 'Bags', 'nameLocation': 'middle', 'nameGap': 30,
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 30, 'fontWeight': 'bold'}, 'minInterval': 1 
                        },
                        'series': [
                            {'name': 'Pass', 'type': 'line', 'data': [], 'itemStyle': {'color': '#22C55E'}, 'smooth': True, 'symbolSize': 12, 'lineStyle': {'width': 5}},
                            {'name': 'Fail', 'type': 'line', 'data': [], 'itemStyle': {'color': '#FF0000'}, 'smooth': True, 'symbolSize': 12, 'lineStyle': {'width': 5}}
                        ]
                    }).classes('absolute inset-0 w-full h-full')

            # Top-Right Quadrant: Particles Over Time
            with ui.element('div').classes('metric-block flex flex-col p-2 border-8 border-[#cceeff] rounded-xl bg-white shadow-sm h-full w-full overflow-hidden min-w-0'):
                ui.label('Particles per Bag').classes('text-5xl font-bold mb-2 text-gray-800 shrink-0')
                with ui.element('div').classes('relative w-full flex-grow min-h-0 min-w-0'):
                    
                    particles_chart = ui.echart({
                        'tooltip': {'trigger': 'axis', 'textStyle': {'fontSize': 30, 'fontWeight': 'bold'}},
                        'grid': {'left': '6%', 'right': '5%', 'bottom': '8%', 'top': '15%', 'containLabel': True},
                        'dataZoom': [{'type': 'inside'}], 
                        'xAxis': {
                            'type': 'category', 'name': 'Time', 'nameLocation': 'middle', 
                            'nameGap': 100, 
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 24, 'fontWeight': 'bold', 'lineHeight': 30}, 
                            'data': [] 
                        },
                        'yAxis': {
                            'type': 'value', 'name': 'Particles', 'nameLocation': 'middle', 'nameGap': 50,
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 30, 'fontWeight': 'bold'}, 'minInterval': 1 
                        },
                        'series': [
                            {'name': 'Particles', 'type': 'line', 'data': [], 'itemStyle': {'color': '#3B82F6'}, 'smooth': True, 'symbolSize': 12, 'lineStyle': {'width': 5}}
                        ]
                    }).classes('absolute inset-0 w-full h-full')

            # Bottom-Left Quadrant: Stage Timings
            with ui.element('div').classes('metric-block flex flex-col p-2 border-8 border-[#cceeff] rounded-xl bg-white shadow-sm h-full w-full overflow-hidden min-w-0'):
                ui.label('Stage Timings per Cycle').classes('text-5xl font-bold mb-2 text-gray-800 shrink-0')
                with ui.element('div').classes('relative w-full flex-grow min-h-0 min-w-0'):
                    
                    timings_chart = ui.echart({
                        'tooltip': {'trigger': 'axis', 'textStyle': {'fontSize': 30, 'fontWeight': 'bold'}},
                        'legend': {
                            'data': ['Load', 'Conveyor', 'Spin', 'Detect', 'Unload'], 'top': 0, 'right': 10,
                            'textStyle': {'fontSize': 30, 'fontWeight': 'bold'} 
                        },
                        'grid': {'left': '6%', 'right': '5%', 'bottom': '8%', 'top': '20%', 'containLabel': True},
                        'dataZoom': [{'type': 'inside'}], 
                        'xAxis': {
                            'type': 'category', 'name': 'Time', 'nameLocation': 'middle', 
                            'nameGap': 100, 
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 24, 'fontWeight': 'bold', 'lineHeight': 30}, 
                            'data': [] 
                        },
                        'yAxis': {
                            'type': 'value', 'name': 'Seconds', 'nameLocation': 'middle', 'nameGap': 60, 
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 30, 'fontWeight': 'bold'}
                        },
                        'series': [
                            {'name': 'Load', 'type': 'line', 'data': [], 'itemStyle': {'color': '#8B5CF6'}, 'smooth': True, 'symbolSize': 10, 'lineStyle': {'width': 5}},
                            {'name': 'Conveyor', 'type': 'line', 'data': [], 'itemStyle': {'color': '#F97316'}, 'smooth': True, 'symbolSize': 10, 'lineStyle': {'width': 5}},
                            {'name': 'Spin', 'type': 'line', 'data': [], 'itemStyle': {'color': '#EAB308'}, 'smooth': True, 'symbolSize': 10, 'lineStyle': {'width': 5}},
                            {'name': 'Detect', 'type': 'line', 'data': [], 'itemStyle': {'color': '#EC4899'}, 'smooth': True, 'symbolSize': 10, 'lineStyle': {'width': 5}},
                            {'name': 'Unload', 'type': 'line', 'data': [], 'itemStyle': {'color': '#14B8A6'}, 'smooth': True, 'symbolSize': 10, 'lineStyle': {'width': 5}}
                        ]
                    }).classes('absolute inset-0 w-full h-full')

            # Bottom-Right Quadrant: Bubbles Over Time
            with ui.element('div').classes('metric-block flex flex-col p-2 border-8 border-[#cceeff] rounded-xl bg-white shadow-sm h-full w-full overflow-hidden min-w-0'):
                ui.label('Bubbles per Bag').classes('text-5xl font-bold mb-2 text-gray-800 shrink-0')
                with ui.element('div').classes('relative w-full flex-grow min-h-0 min-w-0'):
                    
                    bubbles_chart = ui.echart({
                        'tooltip': {'trigger': 'axis', 'textStyle': {'fontSize': 30, 'fontWeight': 'bold'}},
                        'grid': {'left': '6%', 'right': '5%', 'bottom': '8%', 'top': '15%', 'containLabel': True},
                        'dataZoom': [{'type': 'inside'}], 
                        'xAxis': {
                            'type': 'category', 'name': 'Time', 'nameLocation': 'middle', 
                            'nameGap': 100, 
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 24, 'fontWeight': 'bold', 'lineHeight': 30}, 
                            'data': [] 
                        },
                        'yAxis': {
                            'type': 'value', 'name': 'Bubbles', 'nameLocation': 'middle', 'nameGap': 50,
                            'nameTextStyle': {'fontSize': 35, 'fontWeight': 'bold'}, 
                            'axisLabel': {'fontSize': 30, 'fontWeight': 'bold'}, 'minInterval': 1 
                        },
                        'series': [
                            {'name': 'Bubbles', 'type': 'line', 'data': [], 'itemStyle': {'color': '#06B6D4'}, 'smooth': True, 'symbolSize': 12, 'lineStyle': {'width': 5}}
                        ]
                    }).classes('absolute inset-0 w-full h-full')

         

    def fetch_and_update_chart():
        """Fetch cycle data from SQLite and update the line charts."""
        db_path = get_results_db_path()
        if not os.path.exists(db_path): return
        
        def generate_long_label(prefix, num, raw_time):
            try:
                dt = datetime.strptime(raw_time, "%Y-%m-%d %H:%M:%S")
                return f"{prefix} {num}\n{dt.strftime('%b %d, %Y')}\n{dt.strftime('%H:%M:%S')}"
            except Exception:
                return f"{prefix} {num}\n{raw_time}"
        
        try:
            with sqlite3.connect(db_path) as conn:
                cursor = conn.cursor()
                
                # Calculate time threshold
                cutoff_date = (datetime.now() - timedelta(days=time_filter.value)).strftime("%Y-%m-%d %H:%M:%S")
                
                # Fetch pass/fail data
                cursor.execute('''
                    SELECT cycle, MAX(timestamp) as cycle_time,
                           SUM(CASE WHEN LOWER(status) = 'clean' THEN 1 ELSE 0 END) as pass_count,
                           SUM(CASE WHEN LOWER(status) != 'clean' THEN 1 ELSE 0 END) as fail_count
                    FROM detections 
                    WHERE timestamp >= ?
                    GROUP BY cycle ORDER BY cycle ASC
                ''', (cutoff_date,))
                
                rows = cursor.fetchall()
                x_times, y_passes, y_fails = [], [], []
                
                for row in rows:
                    x_times.append(generate_long_label("Cycle", row[0], row[1]))
                    y_passes.append(row[2])
                    y_fails.append(row[3])
                    
                pass_fail_chart.options['xAxis']['data'] = x_times
                pass_fail_chart.options['series'][0]['data'] = y_passes
                pass_fail_chart.options['series'][1]['data'] = y_fails
                pass_fail_chart.update()
                
                # Fetch particles and bubbles
                try:
                    cursor.execute('''
                        SELECT bag_number, timestamp, particles, bubbles
                        FROM detections 
                        WHERE timestamp >= ?
                        ORDER BY id ASC
                    ''', (cutoff_date,))
                    
                    particle_rows = cursor.fetchall()
                    part_x_times, part_y_vals, bubb_y_vals = [], [], []
                    
                    for row in particle_rows:
                        part_x_times.append(generate_long_label("Bag", row[0], row[1]))
                        part_y_vals.append(row[2])
                        bubb_y_vals.append(row[3]) 
                        
                    particles_chart.options['xAxis']['data'] = part_x_times
                    particles_chart.options['series'][0]['data'] = part_y_vals
                    particles_chart.update()

                    bubbles_chart.options['xAxis']['data'] = part_x_times
                    bubbles_chart.options['series'][0]['data'] = bubb_y_vals
                    bubbles_chart.update()
                except sqlite3.OperationalError as e:
                    print(f"Particles/Bubbles DB Error: {e} - Make sure 'bubbles' exists in your DB schema.")
                
                # Stage timings data
                try:
                    cursor.execute('''
                        SELECT rowid, timestamp, load_time, conveyor_time, spin_time, detect_time, unload_time
                        FROM cycles 
                        WHERE timestamp >= ?
                        ORDER BY timestamp ASC
                    ''', (cutoff_date,))
                    
                    timing_rows = cursor.fetchall()
                    t_x_times = []
                    t_load, t_conveyor, t_spin, t_detect, t_unload = [], [], [], [], []
                    
                    for row in timing_rows:
                        t_x_times.append(generate_long_label("Cycle", row[0], row[1]))
                        t_load.append(round(row[2]))
                        t_conveyor.append(round(row[3]))
                        t_spin.append(round(row[4]))
                        t_detect.append(round(row[5]))
                        t_unload.append(round(row[6]))
                        
                    timings_chart.options['xAxis']['data'] = t_x_times
                    timings_chart.options['series'][0]['data'] = t_load
                    timings_chart.options['series'][1]['data'] = t_conveyor
                    timings_chart.options['series'][2]['data'] = t_spin
                    timings_chart.options['series'][3]['data'] = t_detect
                    timings_chart.options['series'][4]['data'] = t_unload
                    timings_chart.update()
                    
                except sqlite3.OperationalError:
                    pass
                except Exception as e:
                    print(f"Timings DB Error: {e}")
                    
        except Exception as e:
            print(f"Metrics chart main DB Error: {e}")
    # Poll the database every 10 seconds
    ui.timer(10.0, fetch_and_update_chart)

@ui.page('/simulation')
def simulation_page():
    """Simulation page: stage row across the top, simulation fills remainder."""
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    ui.add_head_html('<style>.nicegui-content { padding: 0 !important; }</style>')
    build_sidebar()
    build_status_indicator()

    with ui.row().classes('w-full h-screen max-h-screen no-wrap p-6 bg-white gap-8 overflow-hidden'):
        with ui.column().classes('w-full h-full p-0 m-0'):
            # Simulation area fills space
            with ui.element('div').classes('flex-grow w-full h-full m-0 p-0'):
                with ui.element('div').classes('sim-placeholder w-full h-full overflow-hidden'):
                    sim_view = ui.interactive_image().classes('w-full h-full object-cover')
    
    def update_sim_ui():
        if not ros_node:
            return
        # Update simulation image if available
        if getattr(ros_node, 'latest_sim_b64', None):
            sim_view.set_source(ros_node.latest_sim_b64)

    ui.timer(0.1, update_sim_ui)

    
@ui.page('/downloads')
def downloads_page():
    """Render the report browser and PDF export page."""
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    build_sidebar()
    build_status_indicator()

    ui.add_head_html('''
        <style>
            .downloads-shell {
                height: 100vh;
                overflow: hidden;
                min-height: 0;
            }
            .downloads-table-wrap {
                flex: 1 1 auto;
                min-height: 0;
                overflow-y: auto;
                overflow-x: hidden;
            }
            .custom-table thead tr th {
                background-color: #cceeff !important;
                font-size: 1.875rem !important; 
                font-weight: bold !important;
                color: black !important;
                height: 80px !important; 
            }
            .custom-table tbody tr:nth-child(even) {
                background-color: #e6f7ff !important; 
            }
            .custom-table tbody tr:nth-child(odd) {
                background-color: #cceeff !important; 
            }
            .custom-table tbody td {
                font-size: 1.875rem !important; 
                color: black !important;
                height: 80px !important; 
                cursor: pointer; 
            }
            .q-table__card { box-shadow: none !important; border: none !important; }
        </style>
    ''')

    with ui.row().classes('downloads-shell w-full flex-nowrap p-6 bg-white gap-8'):
        with ui.column().classes('w-full h-full min-h-0'):
            
            with ui.row().classes('w-full justify-between items-center mb-6 pr-18'):
                ui.label('Reports').classes('text-6xl font-bold')
                download_btn = ui.button('Download Selected PDFs', icon='file_download', color='primary').classes('text-3xl py-4 px-6 rounded-xl shadow-md')

            columns = [
                {'name': 'id', 'label': 'ID', 'field': 'id', 'sortable': True, 'align': 'left'},
                {'name': 'date', 'label': 'Date', 'field': 'date', 'sortable': True, 'align': 'left'},
                {'name': 'time', 'label': 'Time', 'field': 'time', 'sortable': True, 'align': 'left'},
                {'name': 'cycle', 'label': 'Cycle No.', 'field': 'cycle', 'sortable': True, 'align': 'center'},
                {'name': 'bag_number', 'label': 'Bag No.', 'field': 'bag_number', 'sortable': True, 'align': 'center'},
                {'name': 'result', 'label': 'Result', 'field': 'result', 'sortable': True, 'align': 'center'},
                {'name': 'action', 'label': 'Select', 'field': 'action', 'align': 'center'},
            ]

            db_path = get_results_db_path()
            
            with ui.element('div').classes('downloads-table-wrap w-full'):
                table = ui.table(columns=columns, rows=[], row_key='id').classes('w-full custom-table')

            table.add_slot('body-cell-result', '''
                <q-td :props="props">
                    <q-badge 
                        :style="props.value.toLowerCase() === 'clean' ? 
                            'background-color: #00C853; color: black; font-size: 1.875rem; padding: 12px 24px; border-radius: 30px;' : 
                            'background-color: #E53935; color: white; font-size: 1.875rem; padding: 12px 24px; border-radius: 30px;'"
                        :label="props.value.toLowerCase() === 'clean' ? 'Pass' : 'Fail'"
                    />
                </q-td>
            ''')

            table.add_slot('body-cell-action', '''
                <q-td :props="props" @click.stop>
                    <q-checkbox v-model="props.row.selected" size="xl" color="primary"
                                @update:model-value="(val) => $parent.$emit('toggle_select', {id: props.row.id, val: val})" />
                </q-td>
            ''')

            selected_ids = set()

            def handle_toggle(e):
                """Maintain the set of selected database ids from checkbox events."""
                record_id = e.args['id']
                is_selected = e.args['val']
                if is_selected:
                    selected_ids.add(record_id)
                else:
                    selected_ids.discard(record_id)

            table.on('toggle_select', handle_toggle)

            def refresh_table_data():
                """Load the latest report records from SQLite into the table."""
                if not os.path.exists(db_path):
                    return
                
                try:
                    conn = sqlite3.connect(db_path)
                    cursor = conn.cursor()
                    
                    cursor.execute("SELECT id, timestamp, cycle, bag_number, status FROM detections ORDER BY id DESC")
                    
                    new_rows_data = []
                    for r in cursor.fetchall():
                        raw_date = r[1]
                        try:
                            dt = datetime.strptime(raw_date, "%Y-%m-%d %H:%M:%S")
                            formatted_date = dt.strftime("%d/%m/%y")
                            formatted_time = dt.strftime("%H:%M:%S") 
                        except Exception:
                            # Fallbacks if timestamp is not valid
                            parts = raw_date.split(' ') if raw_date else []
                            formatted_date = parts[0] if len(parts) > 0 else ""
                            formatted_time = parts[1] if len(parts) > 1 else ""

                        new_rows_data.append({
                            'id': r[0],
                            'date': formatted_date,
                            'time': formatted_time, 
                            'cycle': r[2] if r[2] is not None else "-",  
                            'bag_number': r[3] if r[3] is not None else "-",
                            'result': r[4],
                            'selected': r[0] in selected_ids 
                        })
                    conn.close()
                    
                    table.rows = new_rows_data
                    table.update()
                except Exception as e:
                    print(f"Database error during refresh: {e}")

            ui.timer(5, refresh_table_data)

            def on_row_click(e):
                """Open the detailed report dialog for the clicked row."""
                record_id = e.args[1]['id']
                show_report(record_id)

            table.on('row-click', on_row_click)

            def generate_pdf(record_data):
                """Generate a single PDF report containing both images."""
                reports_dir = os.path.join(os.getcwd(), 'logs', 'reports')
                os.makedirs(reports_dir, exist_ok=True)
                
                pdf_path = os.path.join(reports_dir, f"Report_Cycle_{record_data['cycle']}_Bag_{record_data['bag_number']}.pdf")
                
                doc = SimpleDocTemplate(
                    pdf_path, 
                    pagesize=A4, 
                    title=f"Detection Report - Cycle {record_data['cycle']} Bag {record_data['bag_number']}",
                    author="ROBOPharma GUI"
                )
                styles = getSampleStyleSheet()
                elements = []

                # Title
                elements.append(Paragraph(f"Detection Report: Cycle {record_data['cycle']} - Bag {record_data['bag_number']}", styles['Title']))
                elements.append(Spacer(1, 12))

                # Metrics table 
                table_data = [
                    ['Metric', 'Value'],
                    ['Record ID', str(record_data['id'])],
                    ['Timestamp', record_data['timestamp']],
                    ['Status', record_data['status']],
                    ['Contamination Level', record_data['contamination_level']],
                    ['Recommendation', record_data['recommendation']],
                    ['Total Particles', str(record_data['particles'])],
                    ['Total Bubbles', str(record_data['bubbles'])],
                    ['Confidence Score', f"{record_data['confidence']:.2f}%"],
                    ['Detection Model', record_data['detection_model']]
                ]
                
                t = Table(table_data, colWidths=[200, 250])
                
                t.setStyle(TableStyle([
                    ('BACKGROUND', (0, 0), (-1, 0), colors.darkblue),
                    ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
                    ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                    ('GRID', (0, 0), (-1, -1), 1, colors.black),
                    ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold')
                ]))
                
                elements.append(t)
                elements.append(Spacer(1, 24))

                # Images
                img1_path = record_data.get('image_path_pos1')
                img2_path = record_data.get('image_path_pos2')
                
                # Column 1 Elements
                col1_elements = [Paragraph("<para align='center'><b>Position 1</b></para>", styles['Normal']), Spacer(1, 8)]
                if img1_path and os.path.exists(img1_path):
                    col1_elements.append(RLImage(img1_path, width=225, height=400, kind='proportional'))
                else:
                    col1_elements.append(Paragraph("<para align='center'><i>No Pos 1 image.</i></para>", styles['Normal']))

                # Column 2 Elements
                col2_elements = [Paragraph("<para align='center'><b>Position 2</b></para>", styles['Normal']), Spacer(1, 8)]
                if img2_path and os.path.exists(img2_path):
                    col2_elements.append(RLImage(img2_path, width=225, height=400, kind='proportional'))
                else:
                    col2_elements.append(Paragraph("<para align='center'><i>No Pos 2 image.</i></para>", styles['Normal']))

                # Place image columns side-by-side using a layout Table
                img_table = Table([[col1_elements, col2_elements]], colWidths=[230, 230])
                img_table.setStyle(TableStyle([
                    ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                    ('VALIGN', (0,0), (-1,-1), 'TOP'),
                ]))
                
                elements.append(img_table)
                doc.build(elements)
                return pdf_path

            def download_selected():
                """Export PDFs for every selected report row."""
                if not selected_ids:
                    ui.notify("No records selected!", type='warning')
                    return
                
                try:
                    conn = sqlite3.connect(db_path)
                    conn.row_factory = sqlite3.Row
                    cursor = conn.cursor()
                    
                    placeholders = ','.join('?' for _ in selected_ids)
                    cursor.execute(f"SELECT * FROM detections WHERE id IN ({placeholders})", tuple(selected_ids))
                    rows = cursor.fetchall()
                    conn.close()

                    for row in rows:
                        record_data = dict(row)
                        pdf_path = generate_pdf(record_data)
                        ui.download(pdf_path)
                    
                    ui.notify(f'Started download of {len(rows)} reports!', type='positive')
                except Exception as e:
                    ui.notify(f"Database error during batch download: {e}", type='negative')

            download_btn.on_click(download_selected)

            def show_report(record_id):
                """Open a pop up with 3 columns: Image 1, Image 2, and Metrics."""
                try:
                    conn = sqlite3.connect(db_path)
                    conn.row_factory = sqlite3.Row 
                    cursor = conn.cursor()
                    cursor.execute("SELECT * FROM detections WHERE id = ?", (record_id,))
                    row = cursor.fetchone()
                    conn.close()
                except Exception as e:
                    ui.notify(f"Database error: {e}", type='negative')
                    return

                if not row:
                    ui.notify("Record not found.", type='warning')
                    return
                
                record_data = dict(row)

                # Popup display
                with ui.dialog() as dialog, ui.card().classes('w-[1600px] max-w-[95vw] p-8'):
                    # Title Row
                    with ui.row().classes('w-full justify-between items-center mb-6'):
                        ui.label(f"Detailed Report - Cycle {record_data['cycle']} Bag {record_data['bag_number']}").classes('text-5xl font-bold')
                        ui.button(icon='close', on_click=dialog.close).props('flat round dense size=xl')
                    
                    # Three-Column Content Row
                    with ui.row().classes('w-full gap-6 no-wrap items-start'):
                        
                        # COLUMN 1: Position 1 Image
                        with ui.column().classes('w-1/3 gap-4 p-2 border-r-2'):
                            ui.label('Position 1').classes('text-3xl font-bold text-gray-700')
                            img1_path = record_data.get('image_path_pos1')
                            if img1_path and os.path.exists(img1_path):
                                ui.image(img1_path).classes('w-full rounded-lg shadow-md border-2 border-gray-200 object-contain')
                            else:
                                ui.label('No Image Found').classes('text-gray-400 italic p-12 text-center bg-gray-100 rounded-lg w-full text-2xl')

                        # COLUMN 2: Position 2 Image
                        with ui.column().classes('w-1/3 gap-4 p-2 border-r-2'):
                            ui.label('Position 2').classes('text-3xl font-bold text-gray-700')
                            img2_path = record_data.get('image_path_pos2')
                            if img2_path and os.path.exists(img2_path):
                                ui.image(img2_path).classes('w-full rounded-lg shadow-md border-2 border-gray-200 object-contain')
                            else:
                                ui.label('No Image Found').classes('text-gray-400 italic p-12 text-center bg-gray-100 rounded-lg w-full text-2xl')

                        # COLUMN 3: Metrics & Actions
                        with ui.column().classes('w-1/3 gap-4 pl-4'):
                            # Pass/Fail Badge
                            is_clean = record_data['status'].lower() == 'clean'
                            color = 'bg-green-500' if is_clean else 'bg-red-500'
                            text = 'PASS' if is_clean else 'FAIL'
                            ui.label(text).classes(f'{color} text-white text-4xl font-bold px-10 py-3 rounded-full self-end mb-4')

                            # Metrics List
                            def detail_row(label, value):
                                with ui.row().classes('w-full justify-between border-b-2 pb-2 mb-2 flex-nowrap'):
                                    ui.label(label).classes('font-bold text-gray-600 text-2xl shrink-0')
                                    ui.label(str(value)).classes('text-black text-2xl text-right overflow-hidden')
                            
                            detail_row('Timestamp', record_data['timestamp'])
                            detail_row('Particles', record_data['particles'])
                            detail_row('Bubbles', record_data['bubbles'])
                            detail_row('Confidence', f"{record_data['confidence']:.2f}%")
                            detail_row('Contamination', record_data['contamination_level'])
                            detail_row('Model', record_data['detection_model'])
                            
                            ui.space()
                            
                            # PDF Button
                            ui.button('EXPORT TO PDF', icon='picture_as_pdf', color='primary').classes('w-full mt-8 text-3xl py-6 rounded-xl shadow-lg').on_click(
                                lambda: [ui.download(generate_pdf(record_data)), ui.notify('PDF Download Started!', type='positive')]
                            )
                
                dialog.open()
                          
@ui.page('/settings')
def settings_page():
    """Render the system settings page used to trigger model cycling."""
    ui.add_head_html('<link rel="stylesheet" href="/static/style.css">')
    build_sidebar()
    build_status_indicator()
    
    with ui.row().classes('w-full min-h-screen flex-nowrap p-8 pb-20 bg-white gap-8'):
        with ui.column().classes('w-full'):
            with ui.row().classes('w-full justify-between items-center mb-6 pr-18'):
                ui.label('System Settings').classes('text-6xl font-bold text-black')
                
            def on_cycle_click():
                """Trigger the parameter cycle action and show the result to the user."""
                if ros_node:
                    success = ros_node.trigger_parameter_cycle()
                    if not success:
                        ui.notify('<span style="font-size: 2.5rem; font-weight: bold;">Action failed: Change model is currently not working.</span>', type='negative', position='top')

            with ui.row().classes('w-full items-start no-wrap'):
                with ui.column().classes('w-1/2 pr-4'):
                    
                    with ui.column().classes('w-full bg-[#CCEEFF] p-8 rounded-3xl shadow-sm gap-6'):
                        
                        ui.label('Detection Model').classes('text-5xl font-bold text-black mb-2')
                        
                        # Display the current model
                        with ui.row().classes('items-center gap-4'):
                            ui.label('Current Model:').classes('text-3xl font-bold')
                            
                            live_label = ui.label().classes('text-3xl font-bold')
                            
                            if ros_node:
                                # Automatically updates whenever self.current_model changes
                                live_label.bind_text_from(
                                    ros_node, 
                                    'current_model', 
                                    backward=lambda x: str(x) if x else 'Unknown'
                                )
                            else:
                                live_label.set_text("Not connected")
                        
                        # The Cycle Button
                        ui.button('Cycle Model', icon='sync', on_click=on_cycle_click) \
                            .classes('mt-4 text-3xl py-4 px-10 rounded-2xl shadow-lg bg-[#0d0d5c] text-white font-bold w-fit')

def main():
    """Start ROS, spin the dashboard node, and launch the NiceGUI app."""
    global ros_node
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = MyGuiNode()
    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()

    ui.run(
        host='0.0.0.0', 
        port=8080, 
        title="Dashboard", 
        reload=False, 
        show=False
    )

if __name__ in {"__main__", "__mp_main__"}:
    main()
