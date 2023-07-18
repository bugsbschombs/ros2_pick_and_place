#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(String,'go',self.callback,10)

    def callback(self,msg):
        self.publish_trajectory()
        

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        # ******* Start; 1. round

        # Create home position as first trajectory point
        # position0 = home position
        point0 = JointTrajectoryPoint()
        point0.positions = [0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096]
        point0.time_from_start.sec = 0
        point0.time_from_start.nanosec = 0

        # Create the first trajectory point to pick up bottle
        # Position1_pick
        point1 = JointTrajectoryPoint()
        point1.positions = [1.1561303263233145, 0.16385847869441925, -0.18246910177895545, -2.7882678972071178, -0.037512795173492236, 2.981830870198813, 1.7442255872634649]
        point1.time_from_start.sec = 8
        point1.time_from_start.nanosec = 0

        # Create trajectory point to place
        # Position_place
        point2 = JointTrajectoryPoint()
        point2.positions = [0.42115053240390316, 0.9816735313330011, -0.7854030794412952, -1.5668563538584264, 0.8094064742532658, 2.256929333867267, 0.08624142156073834]
        point2.time_from_start.sec = 16
        point2.time_from_start.nanosec = 0

        # Back to home position
        point3 = JointTrajectoryPoint()
        point3.positions = point0.positions
        point3.time_from_start.sec = 24
        point3.time_from_start.nanosec = 0


        # ***** End of trajectory points
        # Add the trajectory points to the trajectory message
        trajectory_msg.points.append(point0)
        trajectory_msg.points.append(point1)
        trajectory_msg.points.append(point2)
        trajectory_msg.points.append(point3)
        

        # Publish the trajectory
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info('Published trajectory.----------------------------------------')


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    
    # Publish the trajectory
    trajectory_publisher.publish_trajectory()
    
    #rclpy.spin_once(trajectory_publisher)
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

#if you are executing the script directly through terminal: (otherwise comment out)
if __name__ == '__main__':
    main()