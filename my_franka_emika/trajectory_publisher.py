#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from rclpy.action import ActionClient
from franka_msgs.action import Grasp
from time import sleep

from tutorial_interfaces.msg import Flag 

TJ1 = {"home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], "pick_hover":[0.6191161206069351, 0.11535040303879421, 0.29176843486431625, -2.760951557291725, -0.06332150335983494, 2.876210502854921, 1.7005373581461278], "pick_down": [0.6186313850198274, 0.1966324138157788, 0.28716365400953653, -2.7323819016434348, -0.06331137736057235, 2.878978080529004, 1.7271250506319373], "place_hover": [0.22822797860616936, 0.7062045551923628, -0.5768607700732141, -1.5355015665890466, 0.4112396324859977, 2.1141806991294523, 0.3097420068531845], "place_down": [0.1991540444083553, 0.9035960775478417, -0.5461253753575235, -1.4792018117650125, 0.4717196703697778, 2.194933304757648, 0.26891479822195447]}

TJ2 = {"home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], "pick_hover":[1.143925462212845, 0.25456577871684344, -0.08455514472767665, -2.498816061365435, 0.1240341158289152, 2.711423718852644, 1.6688574490720223], "pick_down": [1.1429359630140592, 0.31533354342667114, -0.07711123790758476, -2.4957345909899638, 0.12511514631932996, 2.772761021464163, 1.669696282985132], "place_hover": [0.22822797860616936, 0.7062045551923628, -0.5768607700732141, -1.5355015665890466, 0.4112396324859977, 2.1141806991294523, 0.3097420068531845], "place_down": [0.1991540444083553, 0.9035960775478417, -0.5461253753575235, -1.4792018117650125, 0.4717196703697778, 2.194933304757648, 0.26891479822195447]}



tjs_list =  [TJ1,TJ2]
pill_id = 0


#image dictionnary GUI
img_dict = {"u10":"image2.jpg"}

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_arm = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(String,'go',self.callback,10)
        self.action_client = ActionClient(self, Grasp, 'panda_gripper/grasp')

        # self.publisher_app = self.create_publisher(String, '/image_name', 4)

        self.publisher_app = self.create_publisher(Flag, 'image_flag', 4)



    def callback(self,msg):
        self.publish_trajectory(pill_id)
        

    #Gripper grasp
    def close_gripper(self):

        goal_msg = Grasp.Goal()
        goal_msg.width = 0.045    # in meters
        goal_msg.speed = 0.03     # in m/s
        #goal_msg.force = 0.0       # in Newtons I think ...

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    
    def open_gripper(self):

        goal_msg = Grasp.Goal()
        goal_msg.width = 0.07    # in meters
        goal_msg.speed = 0.0     # in m/s
        #goal_msg.force = 0.0       # in Newtons I think ...

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

#    def publish_trajectory(self, pill_id):
        #trj = tjs_list[pill_id]


    def publish_trajectory(self):

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
     

        # ******* 1. round ******

        # Create home position as first trajectory point
        # position0 = home position + Gripper OPEN
        point_home = JointTrajectoryPoint()
        point_home.positions = TJ1["home"]
        point_home.time_from_start.sec = 2
        point_home.time_from_start.nanosec = 0
        
        # pick-up movement: Hover above bottle (gripper open) - go down pick up (gripper closed) - go back to hover (gripper closed)
        # position1_pick_hover + Gripper OPEN
        point_pick_hover = JointTrajectoryPoint()
        point_pick_hover.positions = TJ1["pick_hover"]
        point_pick_hover.time_from_start.sec = 6
        point_pick_hover.time_from_start.nanosec = 0

        # 1_pick_down_close  + Gripper CLOSED
        point_pick = JointTrajectoryPoint()
        point_pick.positions = TJ1["pick_down"]
        point_pick.time_from_start.sec = 2
        point_pick.time_from_start.nanosec = 0

        # position1_pick_hover + Gripper CLOSED
        point_pick_hover2 = JointTrajectoryPoint()
        point_pick_hover2.positions = point_pick_hover.positions
        point_pick_hover2.time_from_start.sec = 2
        point_pick_hover2.time_from_start.nanosec = 0

        # Place-Movement

        # Position_place_hover + Gripper OPEN
        point_place_hover = JointTrajectoryPoint()
        point_place_hover.positions = TJ1["place_hover"]
        point_place_hover.time_from_start.sec = 8
        point_place_hover.time_from_start.nanosec = 0

        # Position_place + Gripper OPEN
        point_place = JointTrajectoryPoint()
        point_place.positions = TJ1["place_down"]
        point_place.time_from_start.sec = 4
        point_place.time_from_start.nanosec = 0

        # Back to home position + Gripper OPEN
        point_home2 = JointTrajectoryPoint()
        point_home2.positions = point_home.positions
        point_home2.time_from_start.sec = 8
        point_home2.time_from_start.nanosec = 0


        # ***** End of trajectory points

        # Publish the trajectory
        self.get_logger().info('Published trajectory.----------------------------------------')

        # Add the trajectory points to the trajectory message
        trajectory_msg.points.append(point_home)
        # print(" -----------> Want to go to home\n")
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to home & sleeping for 8 seconds\n")
        sleep(8)

        # print(" -----------> Want to go to pick hover\n")
        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick_hover)
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to pick hover & sleeping for 8 seconds\n")
        sleep(8)


        # #publish image via app
        # img_msg = String()
        # img_msg.data  = img_dict["u10"]
        # # print(" -----------> Sending image \n")
        # self.publisher_app.publish(img_msg)
        # self.get_logger().info(" -----------> Image published & sleeping for 8 seconds\n")
        # sleep(8)

        #publish image via app
        flag_msg = Flag()
        flag_msg.flag = 1
        # print(" -----------> Sending image \n")
        self.publisher_app.publish(flag_msg)
        self.get_logger().info(" -----------> Image requested & sleeping for 8 seconds\n")
        sleep(8)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick)
        # print(" -----------> Going to point pick\n")
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to point pick & sleeping for 8 seconds\n")
        sleep(8)

        #self.close_gripper()


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick_hover2)
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to pick hover 2 & sleeping for 8 seconds\n")
        sleep(8)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_place_hover)
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to place hover 2 & sleeping for 8 seconds\n")
        sleep(8)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_place)
        self.publisher_arm.publish(trajectory_msg)
        self.get_logger().info(" -----------> Going to point place & sleeping for 8 seconds\n")
        sleep(8)


        # change the image flag
        flag_msg = Flag()
        flag_msg.flag = 0       #### 0 = sends stop signal to image publisher
        # print(" -----------> Sending image \n")
        self.publisher_app.publish(flag_msg)
        self.get_logger().info(" -----------> Image stopped & sleeping for 8 seconds\n")
        sleep(8)

        #self.open_gripper()


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_home2)
        self.publisher_arm.publish(trajectory_msg)
        sleep(8)



def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    
    # Publish the trajectory
    trajectory_publisher.publish_trajectory()
    
    # rclpy.spin_once(trajectory_publisher)
    # rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

#if you are executing the script directly through terminal: (otherwise comment out)
if __name__ == '__main__':
    main()