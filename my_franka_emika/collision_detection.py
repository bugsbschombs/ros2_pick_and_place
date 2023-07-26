#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetForceTorqueCollisionBehavior

#This node does not work for now! 
class CollisionDetection(Node):
    def __init__(self):
        super().__init__('collision_detection')

        self.srv = self.create_service(SetForceTorqueCollisionBehavior, 'collision_detection', self.collision_callback)
        #Do we need a publsiher??
        #self.publisher_ = self.create_publisher()



    def collision_callback(self, request, response):
        collision_msg = SetForceTorqueCollisionBehavior()
        collision_msg.lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        collision_msg.upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        collision_msg.lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]  
        collision_msg.upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        #res = self.call(collision_msg).success
        #if not res:
            #rospy.logerr('Failed to set Force/Torque Collision Behaviour Thresholds')
        #else:
            #rospy.loginfo('Successfully set Force/Torque Collision Behaviour Thresholds')

def main(args=None):
    rclpy.init(args=args)
    collision_detection = CollisionDetection()
    rclpy.spin(collision_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()