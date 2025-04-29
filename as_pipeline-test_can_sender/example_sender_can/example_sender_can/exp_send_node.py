import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame as CanFrame
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
import struct

class ExpSendNode(Node):
    def __init__(self):
        super().__init__('exp_send_node')

        self.counter = 0
        self.publisher_ = self.create_publisher(CanFrame, '/send_to_sensor', 10)
        self.subscriber_ = self.create_subscription(Joy, "/joy", self.send_can, 10)
        self.asEngaged = False
        self.client = self.create_client(SetBool, 'set_asEngaged')
        self.request = SetBool.Request()
        #self.timer = self.create_timer(0.5, self.send_can)

        self.low_trq = True

    def send_can(self, msg: Joy):
        if msg.buttons[6] == 1:
            self.asEngaged = False
            self.request.data = False
            self.client.call_async(self.request)
        elif msg.buttons[7] == 1:
            self.asEngaged = True
            self.request.data = True
            self.client.call_async(self.request)

        if not self.asEngaged:
            frame = CanFrame()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.header.frame_id = 'can_frame'
            frame.id = 0x202
            frame.dlc = 3
            #print(f"Trq: {round(msg.axes[1] * 99)}")
            frame.data[0] = round(msg.axes[1] * 99)
            #frame.data[0] = 1
            #print(f"Steering: {round(-msg.axes[2] * 95)}")
            bytes_ = bytearray(struct.pack("f", -msg.axes[2]))
            frame.data[4] = bytes_[0]
            frame.data[5] = bytes_[1]
            frame.data[6] = bytes_[2]
            frame.data[7] = bytes_[3]
            #print(f"A: {msg.buttons[0]}")
            #print(f"B: {msg.buttons[1]}")
            #print(f"X: {msg.buttons[3]}")
            frame.data[2] = msg.buttons[3] << 2 | msg.buttons[1] << 1 | msg.buttons[0]
            if frame.data[0] > 25:
                #if self.low_trq:
                frame.data[2] |= 1 << 2 | 1
                #    self.low_trq = False
            #else:
            #    self.low_trq = True
            #self.get_logger().info(f"{frame.data[0]}, {frame.data[1]}, {frame.data[2]}")
            #frame.data[1] = 64
            self.publisher_.publish(frame)

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    node = ExpSendNode()  # Instantiate the node class
    rclpy.spin(node)  # Keep the node alive and process callbacks

    # Shutdown after exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
