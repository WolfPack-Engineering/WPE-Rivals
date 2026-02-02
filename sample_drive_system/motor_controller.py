import rclpy
import RPi.GPIO as GPIO

from geometry_msgs.msg import Vector3

# pin assignments for motors, will update once wiring in known
FRONT_LEFT_EN  = 1
FRONT_LEFT_IN1 = 2
FRONT_LEFT_IN2 = 3

FRONT_RIGHT_EN  = 4
FRONT_RIGHT_IN1 = 5
FRONT_RIGHT_IN2 = 6

BACK_LEFT_EN  = 7
BACK_LEFT_IN1 = 8
BACK_LEFT_IN2 = 9

BACK_RIGHT_EN  = 10
BACK_RIGHT_IN1 = 11
BACK_RIGHT_IN2 = 12


def drive_callback(msg):
    # basic omni power distribution, may have to change once wheel orientation is known 
    front_left_motor  = msg.y + msg.x
    front_right_motor = msg.y - msg.x
    back_left_motor   = msg.y - msg.x
    back_right_motor  = msg.y + msg.x

    # cap power values in the range (-1, 1)
    front_left_motor  = max(-1.0, min(1.0, front_left_motor))
    front_right_motor = max(-1.0, min(1.0, front_right_motor))
    back_left_motor   = max(-1.0, min(1.0, back_left_motor))
    back_right_motor  = max(-1.0, min(1.0, back_right_motor))

    # set each motor to new capped power level
    set_motor(front_left_pwm, FRONT_LEFT_IN1, FRONT_LEFT_IN2, front_left_motor)
    set_motor(front_right_pwm, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, front_right_motor)
    set_motor(back_left_pwm, BACK_LEFT_IN1, BACK_LEFT_IN2, back_left_motor)
    set_motor(back_right_pwm, BACK_RIGHT_IN1, BACK_RIGHT_IN2, back_right_motor)

# function to set motor powers
def set_motor(pwm, in1, in2, power):
    if power > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif power < 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

    # (abs because in1 and in2 already handle motor direction)
    pwm.ChangeDutyCycle(abs(power) * 100)


def main(args=None):
    # this is just example code so I have no config
    rclpy.init(args=args)

    node = rclpy.create_node('motor_controller')

    subscription = node.create_subscription(Vector3, 'drive_commands', drive_callback, 10)

    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([FRONT_LEFT_IN1, FRONT_LEFT_IN2, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,
                BACK_LEFT_IN1, BACK_LEFT_IN2, BACK_RIGHT_IN1, BACK_RIGHT_IN2], GPIO.OUT)
    GPIO.setup([FRONT_LEFT_EN, FRONT_RIGHT_EN, BACK_LEFT_EN, BACK_RIGHT_EN], GPIO.OUT)

    global front_left_pwm, front_right_pwm, back_left_pwm, back_right_pwm
    front_left_pwm  = GPIO.PWM(FRONT_LEFT_EN, 1000)
    front_right_pwm = GPIO.PWM(FRONT_RIGHT_EN, 1000)
    back_left_pwm   = GPIO.PWM(BACK_LEFT_EN, 1000)
    back_right_pwm  = GPIO.PWM(BACK_RIGHT_EN, 1000)
    front_left_pwm.start(0)
    front_right_pwm.start(0)
    back_left_pwm.start(0)
    back_right_pwm.start(0)

    #loop
    rclpy.spin(node)

    #cleanup
    front_left_pwm.stop()
    front_right_pwm.stop()
    back_left_pwm.stop()
    back_right_pwm.stop()
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
