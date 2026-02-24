import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point

class BrainStateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_main_logic')

        self.inventory = []
        self.waypoint_map = {
            'RED': Point(x=2.0, y=0.5, z=0.0),
            'BLUE': Point(x=2.0, y=0.0, z=0.0),
            'GREEN': Point(x=2.0, y=-0.5, z=0.0)
        }

        self.current_mode = "MANUAL"
        self.latest_color = None
        self.alignment_error_x = 0.0

        self.create_subscription(String, '/app/buttons', self.gui_callback, 10)
        self.create_subscription(String, '/perception/color', self.color_callback, 10)
        self.create_subscription(Point, '/perception/error', self.error_callback, 10)

        self.nav_pub = self.create_publisher(Point, '/nav/goal', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm/command', 10)
        self.mode_pub = self.create_publisher(String, '/system/mode', 10)

        self.timer = self.create_timer(0.1, self.state_machine_loop)
        
        self.get_logger().info("Power On: ROS 2 Nodes Initialized. Mode = MANUAL")

    def gui_callback(self, msg):
        command = msg.data.upper()
        if command in ["MANUAL", "AUTO_PICK", "AUTO_PLACE"]:
            self.current_mode = command
            self.mode_pub.publish(String(data=self.current_mode))
            self.get_logger().info(f"Mode Switched to: {self.current_mode}")

    def color_callback(self, msg):
        self.latest_color = msg.data.upper()

    def error_callback(self, msg):
        self.alignment_error_x = msg.x

    def state_machine_loop(self):
        if self.current_mode == "MANUAL":
            self.execute_manual_loop()
        elif self.current_mode == "AUTO_PICK":
            self.execute_auto_pick()
        elif self.current_mode == "AUTO_PLACE":
            self.execute_auto_place()

    def execute_manual_loop(self):
        pass

    def execute_auto_pick(self):
        self.get_logger().info("Starting Pick Sequence...", once=True)
        
        if abs(self.alignment_error_x) > 5.0:
            align_msg = Twist()
            align_msg.linear.y = self.alignment_error_x * 0.01
            self.cmd_vel_pub.publish(align_msg)
            return
        
        self.cmd_vel_pub.publish(Twist()) 

        self.get_logger().info("Visual Servo Aligned. Grabbing Box...")
        self.arm_pub.publish(String(data="GRAB"))
        
        if self.latest_color:
            if self.latest_color in self.inventory:
                self.get_logger().warn(f"Duplicate {self.latest_color}! Dropping Box & Alerting User.")
                self.arm_pub.publish(String(data="DROP"))
            else:
                self.get_logger().info(f"New Color: {self.latest_color}. Saving to Inventory.")
                self.inventory.append(self.latest_color)
            
            self.latest_color = None
            self.current_mode = "MANUAL"
            self.mode_pub.publish(String(data="MANUAL"))
            self.get_logger().info("Set Mode = MANUAL")

    def execute_auto_place(self):
        if not self.inventory:
            self.get_logger().info("Inventory Empty! Task Complete.")
            self.current_mode = "MANUAL"
            self.mode_pub.publish(String(data="MANUAL"))
            return

        self.get_logger().info("Starting Delivery Sequence...", once=True)

        target_color = self.inventory[0]
        target_coords = self.waypoint_map.get(target_color)

        self.get_logger().info(f"Driving to {target_color} Zone at {target_coords}")
        self.nav_pub.publish(target_coords)

        self.arm_pub.publish(String(data="DROP"))
        self.get_logger().info(f"Box {target_color} placed successfully.")

        self.inventory.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = BrainStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()