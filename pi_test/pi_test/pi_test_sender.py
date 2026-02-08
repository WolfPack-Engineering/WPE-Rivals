import socket
import json
import rclpy

from geometry_msgs.msg import Vector3

UDP_PORT = 5555
latest_data = {"x": 0.0, "y": 0.0}


def timer_callback():
    global sock, publisher, latest_data
    try:
        while True:
            data, _ = sock.recvfrom(1024)
            latest_data = json.loads(data.decode())
    except BlockingIOError:
        pass

    msg = Vector3()
    msg.x = float(latest_data["x"])
    msg.y = float(latest_data["y"])
    publisher.publish(msg)


def main(args=None):
    global sock, publisher
    rclpy.init(args=args)

    reader = rclpy.create_node('gamepad_reader')
    publisher = reader.create_publisher(Vector3, 'drive_commands', 10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)

    reader.get_logger().info(f"Listening for gamepad data on UDP port {UDP_PORT}")

    frequency = 0.05
    timer = reader.create_timer(frequency, timer_callback)

    rclpy.spin(reader)

    sock.close()
    reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
