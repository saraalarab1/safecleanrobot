import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

# Set the pins for the motor driver inputs
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_IN1 = 22
LEFT_MOTOR_IN2 = 23

# Set the pins for the motor driver outputs
RIGHT_MOTOR_EN = 18
LEFT_MOTOR_EN = 13

# Set the GPIO pins that the encoder is connected to
ENCODER_PIN = 3
ENCODER_PIN2 = 4

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_to_pwm_callback,10)
        
        GPIO.setmode(GPIO.BCM)
        # # Set the encoder pins as inputs
        # GPIO.setup(ENCODER_PIN, GPIO.IN)
        # GPIO.setup(ENCODER_PIN2, GPIO.IN)

        # Set the GPIO pins for the motor driver inputs as outputs
        GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
        GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
        GPIO.setup(LEFT_MOTOR_IN1, GPIO.OUT)
        GPIO.setup(LEFT_MOTOR_IN2, GPIO.OUT)

        # Set the GPIO pins for the motor driver outputs as outputs
        GPIO.setup(RIGHT_MOTOR_EN, GPIO.OUT)
        GPIO.setup(LEFT_MOTOR_EN, GPIO.OUT)

         # Encoder parameters
        self.encoder_val = 0
        self.encoder_val2 = 0
        self.old_time = 0
        self.pulses_per_rev = 20
        self.wheel_circumference = math.pi * 6.5  # in cm
        self.pwm_speed = 10
        self.pwm_speed2 = 10
        self.linear_vel = 0
        self.linear_vel2 = 0
        self.update_speed = False

        # Create PWM channels for the motor driver outputs with a frequency of 100 Hz
        self.pwm_r = GPIO.PWM(RIGHT_MOTOR_EN, 100)
        self.pwm_l = GPIO.PWM(LEFT_MOTOR_EN, 100)

        # Start the PWM channels with a duty cycle of 0
        self.pwm_r.start(self.pwm_speed)
        self.pwm_l.start(self.pwm_speed2)

        self.pid = PID(1, 0, 0, setpoint=25)  # Target cm/s
        self.pid.sample_time = 0.1

        # # Attach interrupts to count encoder pulses
        # GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING,
        #                       callback=self.update_encoder_value)
        # GPIO.add_event_detect(ENCODER_PIN2, GPIO.RISING,
        #                       callback=self.update_encoder_value)


    def cmd_to_pwm_callback(self, msg):

        right_wheel_vel = ( msg.linear.x  + msg.angular.z ) /2
        left_wheel_vel = (  msg.linear.x  - msg.angular.z ) /2

        GPIO.output(RIGHT_MOTOR_IN1, right_wheel_vel > 0)
        GPIO.output(RIGHT_MOTOR_IN2, right_wheel_vel < 0 )
        GPIO.output(LEFT_MOTOR_IN1, left_wheel_vel > 0)
        GPIO.output(LEFT_MOTOR_IN2, left_wheel_vel < 0)




def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)
    VelocitySubscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()