#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# GPIO pins for motor control
ENA = 13  # Enable motor A
IN1 = 5  # Motor A input 1
IN2 = 6  # Motor A input 2
ENB = 12  # Enable motor B
IN3 = 16  # Motor B input 1
IN4 = 20  # Motor B input 2

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Initialize PWM for motor speed control
pwm_motor_a = GPIO.PWM(ENA, 100)  # 100 Hz frequency
pwm_motor_b = GPIO.PWM(ENB, 100)
pwm_motor_a.start(0)  # Start with 0% duty cycle
pwm_motor_b.start(0)

def cmd_vel_callback(msg):
    # Calculate motor speeds based on velocities
    left_speed = abs(msg.linear.x)**4 * 100
    right_speed = abs(msg.linear.y)**4 * 100
    left_dir = msg.linear.x > 0
    right_dir = msg.linear.y > 0

    # Convert speeds to PWM duty cycles (0 to 100)
    pwm_motor_a.ChangeDutyCycle(right_speed)
    pwm_motor_b.ChangeDutyCycle(left_speed)

    # Set motor directions based on speeds
    GPIO.output(IN1, not right_dir)
    GPIO.output(IN2, right_dir)
    GPIO.output(IN3, left_dir)
    GPIO.output(IN4, not left_dir)


def motor_control_node():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Stop PWM and cleanup GPIO on node shutdown
        pwm_motor_a.stop()
        pwm_motor_b.stop()
        GPIO.cleanup()
