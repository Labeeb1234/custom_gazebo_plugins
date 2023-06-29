import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile
import rclpy.qos as qos

class SimulationTime(Node):

    def __init__(self):
        super().__init__("simulation_time")
        self.sub = self.create_subscription(Clock, '/clock', self.clock_callback ,qos.qos_profile_system_default)

    def clock_callback(self, msg):
        current_sim_time = msg.clock.sec
        self.get_logger().info("Current Simulation Time is: %f" %current_sim_time)

def main(args=None):
    rclpy.init(args=args)

    simulation_time = SimulationTime()
    rclpy.spin(simulation_time)
    simulation_time.destroy_node()

    rclpy.shutdown()

if __name__ == 'main':
    main()
