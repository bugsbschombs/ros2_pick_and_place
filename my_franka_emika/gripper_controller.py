#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from franka_msgs.action import Grasp


class GripperActionClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')
        self._action_client = ActionClient(self, Grasp, 'panda_gripper/grasp')

    def send_goal(self):

        goal_msg = Grasp.Goal()
        goal_msg.width = 0.045    # in meters
        goal_msg.speed = 0.03     # in m/s
        #goal_msg.force = 0.0       # in Newtons I think ...

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):

    rclpy.init(args=args)

    action_client = GripperActionClient()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)



if __name__ == '__main__':

    main()






# class GripperJointStatePublisher(Node):
#     def __init__(self):
#         super().__init__('gripper_joint_state_publisher')
#         self.publisher_ = self.create_publisher(JointState, '/panda_gripper/joint_states', 10)
#         self.timer_ = self.create_timer(1, self.publish_joint_states)
#         self.joint_names_ = ['panda_finger_joint1', 'panda_finger_joint2']    
#     def publish_joint_states(self):
#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#         joint_state_msg.name = self.joint_names_
#         joint_state_msg.position = [0.0, 0.0]  # Set the joint positions here
#         joint_state_msg.velocity = []
#         joint_state_msg.effort = []
#         self.publisher_.publish(joint_state_msg)
        
# def main(args=None):
#     rclpy.init(args=args)
#     gripper_joint_state_publisher = GripperJointStatePublisher()
#     rclpy.spin(gripper_joint_state_publisher)
#     gripper_joint_state_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
