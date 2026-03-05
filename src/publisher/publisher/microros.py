import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math


class JointStateToMicroROS(Node):

    def __init__(self):
        super().__init__('joint_state_to_microros')

        self.joint_names = ['waist_joint', 'upperarm_joint', 'lowerarm_joint','wrist_yaw_joint', 'wrist_pitch_joint']

        # -------- Command from MoveIt / ROS2 --------
        self.sub_cmd = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            100
        )

        # -------- Send to ESP32 --------
        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            '/joint_angles',
            100
        )

        # -------- Feedback from ESP32 encoders --------
        self.sub_enc = self.create_subscription(
            Float32MultiArray,
            '/encoders_angles',
            self.encoder_cb,
            100
        )

        # -------- Republish as JointState (recommended) --------
        self.pub_feedback = self.create_publisher(
            JointState,
            '/joint_states_feedback',
            100
        )

        self.latest_angles = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.ratio = 39.0 / 29.0

    # =============================
    # Command callback (ROS → ESP32)
    # =============================
    def joint_state_cb(self, msg):
        angles = {}

        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                angles[name] = pos

        if len(angles) != len(self.joint_names):
            return

        angle1 = math.degrees(angles['waist_joint'])
        angle2 = math.degrees(angles['upperarm_joint'])
        angle3 = math.degrees(angles['lowerarm_joint'])
        theta1 = math.degrees(angles['wrist_yaw_joint'])
        theta2 = math.degrees(angles['wrist_pitch_joint'])

        # =============================
        # Differential Motor Equations
        # =============================
        theta_m1 = 4.0 * (theta1 + self.ratio * theta2)
        theta_m2 = 4.0 * (self.ratio * theta2 - theta1)

        out = Float32MultiArray()
        out.data = [angle1, angle2, angle3, theta_m1, theta_m2]

        self.pub_cmd.publish(out)

    # =============================
    # Encoder callback (ESP32 → ROS)
    # =============================
    def encoder_cb(self, msg):
        if len(msg.data) < 2:
            return

        self.latest_angles[0] = math.radians(msg.data[0])
        self.latest_angles[1] = math.radians(msg.data[1])

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.latest_angles

        self.pub_feedback.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToMicroROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
