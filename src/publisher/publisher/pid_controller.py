#!/usr/bin/env python3
import rclpy
from rclpy.node import Node, SetParametersResult
from std_msgs.msg import Float32
import time

class StepperPID(Node):

    def __init__(self):
        super().__init__("pid_stepper_controller")

        # -------------------------
        # Declare parameters
        # -------------------------
        self.declare_parameter("kp", 3.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("target_angle", 90.0)
        self.declare_parameter("deadband", 1.5)      # <--- NEW

        # Load parameters
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.Kd = self.get_parameter("kd").value
        self.target_angle = self.get_parameter("target_angle").value
        self.deadband = self.get_parameter("deadband").value

        # Dynamic updates
        self.add_on_set_parameters_callback(self.param_callback)

        # Subscriber
        self.create_subscription(Float32, "current_angle",
                                 self.encoder_callback, 10)

        # Publisher (signed speed)
        self.pub_speed = self.create_publisher(Float32, "cmd_speed", 50)

        # PID state
        self.prev_time = time.time()
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_angle = 0.0

        self.get_logger().info("PID Stepper Controller Ready (signed-speed mode + deadband).")

        # Timer → 200 Hz
        self.create_timer(0.005, self.pid_loop)

    # ----------------------------------------------------
    # Dynamic parameter update
    # ----------------------------------------------------
    def param_callback(self, params):
        for p in params:
            if p.name == "kp":
                self.Kp = p.value
            elif p.name == "ki":
                self.Ki = p.value
            elif p.name == "kd":
                self.Kd = p.value
            elif p.name == "target_angle":
                self.target_angle = p.value
            elif p.name == "deadband":
                self.deadband = p.value

        self.get_logger().info(
            f"Updated → Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, "
            f"Target={self.target_angle}, Deadband={self.deadband}"
        )
        return SetParametersResult(successful=True)

    # ----------------------------------------------------
    # Encoder callback
    # ----------------------------------------------------
    def encoder_callback(self, msg):
        self.current_angle = msg.data

    # ----------------------------------------------------
    # PID Loop
    # ----------------------------------------------------
    def pid_loop(self):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return
        self.prev_time = now

        # Compute error
        error = self.target_angle - self.current_angle

        # ----------------------------------
        # ERROR THRESHOLD (DEADBAND)
        # ----------------------------------
        if abs(error) < self.deadband:
            msg = Float32()
            msg.data = 0.0
            self.pub_speed.publish(msg)

            # Reset PID terms
            self.integral = 0.0
            self.prev_error = error

            return

        # ----------------------------------
        # PID calculation
        # ----------------------------------
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = error

        output = P + I + D

        # Clamp speed
        cmd_speed = max(-2000.0, min(2000.0, output))
        cmd_speed = -cmd_speed
        cmd_speed = round(cmd_speed, 2)

        # Publish speed
        msg = Float32()
        msg.data = cmd_speed
        self.pub_speed.publish(msg)

        # Optional debug
        if int(now * 2) % 2 == 0:
            self.get_logger().info(
                f"Angle={self.current_angle:.1f} | Err={error:.1f} | DB={self.deadband} | Speed={cmd_speed:.1f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = StepperPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
