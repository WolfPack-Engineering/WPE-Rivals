import pygame
import rclpy

# in this example I will only worry about linear motion, not rotation so I will use a 3 dimensional vector instead of a twist
from geometry_msgs.msg import Vector3

# initialize gamepad using pygame library, I'm not sure if this will successfuly detect USB devices so it will need to be tested
pygame.init()
pygame.joystick.init()


def timer_callback():
    global controller, publisher
    pygame.event.pump()
    a = controller.get_button(0)
    b = controller.get_button(1)
    x = controller.get_button(2)
    y = controller.get_button(3)

    # fairly standard but I thought I'd state it anyways: forward and right are positive x and y (cartesian)
    xVel = float(b - x)
    yVel = float(y - a)

    msg = Vector3()
    msg.x = xVel
    msg.y = yVel
    # z defaults to 0 so technically this is more like a 2 dimensional vector
    publisher.publish(msg)


def main(args=None):
    global controller, publisher
    rclpy.init(args=args)

    reader = rclpy.create_node('gamepad_reader')

    publisher = reader.create_publisher(Vector3, 'drive_commands', 10)

    # will have to play around with frequency to get the right gamepad responsiveness
    frequency = 0.05
    timer = reader.create_timer(frequency, timer_callback)

    # hopefully successfully reads from the first connected controller
    controller = pygame.joystick.Joystick(0)
    controller.init()

    rclpy.spin(reader)

    reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
