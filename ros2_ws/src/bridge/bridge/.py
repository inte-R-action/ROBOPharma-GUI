import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Change this to your actual message type
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge_node')
        
        # 1. Initialize CoppeliaSim Remote API
        # Ensure CoppeliaSim is running before starting this node
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.get_logger().info('Connected to CoppeliaSim.')

        # 2. Get Object Handles (Replace 'MyPart' with your object name in the scene)
        # self.part_handle = self.sim.getObject('/MyPart')

        # 3. ROS Subscriptions
        self.subscription = self.create_subscription(
            String, 
            'part_status_topic', 
            self.listener_callback, 
            10)

        # 4. Logic Timer (e.g., checks every 0.1s to update colors)
        self.timer = self.create_timer(0.1, self.update_sim_visuals)
        
        self.active = False

    def listener_callback(self, msg):
        # Logic to determine activity based on incoming ROS messages
        self.get_logger().info(f'Received: {msg.data}')
        self.active = True 

    def update_sim_visuals(self):
        # This is where you will call sim.setShapeColor
        if self.active:
            # Logic for "Active" color
            pass
        else:
            # Logic for "Inactive" color
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()