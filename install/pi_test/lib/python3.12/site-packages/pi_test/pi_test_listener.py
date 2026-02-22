import rclpy

from geometry_msgs.msg import Vector3



def drive_callback(msg):
    # loop contents
    listener.get_logger().info(f"X velocity: {msg.x}   |   Y velocity {msg.y}")

def main(args=None):
    rclpy.init(args=args)
    global listener

    listener = rclpy.create_node('controls_listener')
    subscription = listener.create_subscription(Vector3, 'drive_commands', drive_callback, 10)

    #loop
    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
