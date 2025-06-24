
### Reciver
import json, zmq, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyNode(Node):
    def __init__(self):
        super().__init__("joy_bridge")
        ctx = zmq.Context()
        self.sub= ctx.socket(zmq.SUB)
        self.sub.connect("tcp://0.0.0.0:5555")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.pub= self.create_publisher(Joy, "joy", 10)
        self.create_timer(0.001, self.tick)

        self.seq = 0

    def tick(self):
        try:
            msg = self.sub.recv_json(flags=zmq.NOBLOCK)
            joy = Joy()
            joy.header.stamp =  self.get_clock().now().to_msg()
            joy.header.frame_id = "joy"
            joy.axes = msg["axes"]
            joy.buttons = msg["buttons"]
            self.pub.publish(joy)
            self.seq += 1

        except zmq.Again:
            pass

def main():
    rclpy.init()
    rclpy.spin(JoyNode())

if __name__ == "__main__":
    main()
