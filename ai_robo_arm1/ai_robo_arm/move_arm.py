#=================================================================
#Author : Eshanth Eshwar M
#email : eshwareshanth@gmail.com
#=================================================================


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class WordListSubscriber(Node):
    def __init__(self):
        super().__init__('word_list_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/gesture',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/iiwa_arm_controller/joint_trajectory',
            10)

    def listener_callback(self, gesture_msg):
        target_rad = list(gesture_msg.data)  # already a list of floats, no parsing needed
        self.get_logger().info(f'Received list: {target_rad}')
        a = target_rad[0]
        b = target_rad[1]
        c = target_rad[2]
        d = target_rad[3]
        e = target_rad[4]
        f = target_rad[5]
        g = target_rad[6]
        





        msg = JointTrajectory()

        msg.joint_names = [
            'joint_a1',
            'joint_a2',
            'joint_a3',
            'joint_a4',
            'joint_a5',
            'joint_a6',
            'joint_a7'
        ]

        positions = [a, b, c, d, e, f, g]

          
        
        print(positions)

        

        point = JointTrajectoryPoint()

        point.positions = positions

        point.velocities = [0.0] * 7

        point.time_from_start.sec = 5

        msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info("Trajectory sent.")


def main(args=None):
    rclpy.init(args=args)
    node = WordListSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
