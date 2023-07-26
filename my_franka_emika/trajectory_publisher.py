#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Homing, Move
from time import sleep

from math import floor

from tutorial_interfaces.msg import Flag 


GIGA = 1e9

#Joint limits for Safety 
lower_joint_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
upper_joint_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


#Trajectories for each pick_and_place round 

TJ1 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.6191161206069351, 0.11535040303879421, 0.29176843486431625, -2.760951557291725, -0.06332150335983494, 2.876210502854921, 1.7005373581461278], 
    "pick_down": [0.6186313850198274, 0.1966324138157788, 0.28716365400953653, -2.7323819016434348, -0.06331137736057235, 2.878978080529004, 1.7271250506319373], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ2 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[1.143925462212845, 0.25456577871684344, -0.08455514472767665, -2.498816061365435, 0.1240341158289152, 2.711423718852644, 1.6688574490720223], 
    "pick_down": [1.1429359630140592, 0.31533354342667114, -0.07711123790758476, -2.4957345909899638, 0.12511514631932996, 2.772761021464163, 1.669696282985132], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }


# tjs_list =  [TJ1, TJ2, TJ3, TJ4, TJ5, TJ6, TJ7, TJ8, TJ9, TJ10]
tjs_list =  [TJ1, TJ2]

#uncertainty = 

box_ids = [1, 2]   # this is size 2
# box_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]   # this is size 10

# three uncertainty conditions
uncertainties_low = [8,12,14,7,7,5,9,46,45]  # this is size 10
uncertainties_high = [71,98,75,62,96,83,22,52,46,45]
uncertainties_no = [0,0,0,0,0,0,0,0,0,0]


curr_box_id = 1



#image dictionnary GUI
img_dict = {"u10":"image2.jpg"}

class TrajectoryPublisher(Node):

    def __init__(self):

        super().__init__('trajectory_publisher')

        self.publisher_arm = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(String,'go',self.callback,10)
        self.action_client_grasp = ActionClient(self, Grasp, 'panda_gripper/grasp')
        self.action_client_homing = ActionClient(self, Homing, 'panda_gripper/homing')
        self.action_client_move = ActionClient(self, Move, 'panda_gripper/move')

        # self.publisher_app = self.create_publisher(String, '/image_name', 4)

        self.publisher_app = self.create_publisher(Flag, 'image_flag', 4)


    def get_wait_time(self, uncertainty):
        a = 0.08
        b = 2
        wait_time = a * uncertainty + b
        return wait_time
    

    def get_travel_time(self, uncertainty):
        a = 0.06
        b = 8
        wait_time = a * uncertainty + b
        return wait_time
        

    def callback(self,msg):
        self.go = True
        # self.publish_trajectory(box_id)

        

    # homing of the gripper
    def open_gripper(self):

        # goal_msg = Homing.Goal()
        goal_msg = Move.Goal()
        goal_msg.width = 0.07
        goal_msg.speed = 0.03

        self.action_client_move.wait_for_server()
        #call service with send_goal_asynch
        return self.action_client_move.send_goal_async(goal_msg)
    

    # closing the gripper
    def close_gripper(self):

        goal_msg = Grasp.Goal()
        goal_msg.width = float(0.048)    # in meters
        goal_msg.speed = 0.03     # in m/s
        goal_msg.force = float(30.0)     # in Newtons I think ...

        self.action_client_grasp.wait_for_server()
        return self.action_client_grasp.send_goal_async(goal_msg)
    


    def publish_with_checking(self, msg):
        if not self.within_limits(msg.points[0].positions):
            # violating limits, shutting down now
            print("violating limits, shutting down now !!!")
            rclpy.shutdown()
        else:
            self.publisher_arm.publish(msg)

    
    def within_limits(self, vals):
        for i in range(len(vals)):
            if vals[i] < lower_joint_limits[i] or vals[i] > upper_joint_limits[i]:
                return False
        return True



    ######################################## THE IMPORTANT FUNCTION RIGHT HERE ########################################
    def publish_trajectory(self, box_id):
        
        # get the current trajectory and uncertainty using the input box_id
        current_trj = tjs_list[box_id - 1]
        un = uncertainties_low[box_id - 1]
        image_flag = box_id

        # get the wait times based on the uncertainty
        pick_hover_wait_time = self.get_wait_time(un)
        travel_time = self.get_travel_time(un)

        travel_time_s = floor(travel_time)
        travel_time_ns = int((travel_time - travel_time_s) * GIGA)
        
        print(travel_time_s)
        print(travel_time_ns)


        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        # ******* 1. round ******

        # Create home position as first trajectory point
        # position0 = home position + Gripper OPEN
        point_home = JointTrajectoryPoint()
        point_home.positions = current_trj["home"]
        point_home.time_from_start.sec = 6          # this was 6
        point_home.time_from_start.nanosec = 0
        
        # pick-up movement: Hover above bottle (gripper open) - go down pick up (gripper closed) - go back to hover (gripper closed)
        # position1_pick_hover + Gripper OPEN
        point_pick_hover = JointTrajectoryPoint()
        point_pick_hover.positions = current_trj["pick_hover"]
        point_pick_hover.time_from_start.sec = 8        # this was 8
        point_pick_hover.time_from_start.nanosec = 0

        # 1_pick_down_close  + Gripper CLOSED
        point_pick = JointTrajectoryPoint()
        point_pick.positions = current_trj["pick_down"]
        point_pick.time_from_start.sec = 2        # this was 2
        point_pick.time_from_start.nanosec = 0

        # position1_pick_hover + Gripper CLOSED
        point_pick_hover2 = JointTrajectoryPoint()
        point_pick_hover2.positions = point_pick_hover.positions
        point_pick_hover2.time_from_start.sec = 2        # this was 2
        point_pick_hover2.time_from_start.nanosec = 0

        # Place-Movement

        # Position_place_hover + Gripper OPEN
        point_place_hover = JointTrajectoryPoint()
        point_place_hover.positions = current_trj["place_hover"]
        point_place_hover.time_from_start.sec = travel_time_s     # this was without -2 seconds
        point_place_hover.time_from_start.nanosec = travel_time_ns

        # Position_place + Gripper OPEN
        point_place = JointTrajectoryPoint()
        point_place.positions = current_trj["place_down"]
        point_place.time_from_start.sec = 3        # this was 4
        point_place.time_from_start.nanosec = 0

        # Back to home position + Gripper OPEN
        point_home2 = JointTrajectoryPoint()
        point_home2.positions = point_home.positions
        point_home2.time_from_start.sec = 8        # this was 8
        point_home2.time_from_start.nanosec = 0


        # ***** End of trajectory points
        #self.get_logger().info('Waiting for the /go signal !!!!!!----------------------------------------')

        # the "go" class variable is set to True here, go time!
        #if self.go:

        # Publish the trajectory
        self.get_logger().info('Published trajectory.----------------------------------------')

            # Add the trajectory points to the trajectory message
        trajectory_msg.points.append(point_home)
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to home & sleeping for 6 seconds\n")
        for i in range(6):
            print("Sleeping for %d out of 6 seconds" % int(i+1))
            sleep(1)

        # print(" -----------> Want to go to pick hover\n")
        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick_hover)
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to pick hover & sleeping for 8 seconds\n")
        for i in range(8):
            print("Sleeping for %d out of 8 seconds" % int(i+1))
            sleep(1)


        # #publish image via app
        # img_msg = String()
        # img_msg.data  = img_dict["u10"]
        # # print(" -----------> Sending image \n")
        # self.publisher_app.publish(img_msg)
        # self.get_logger().info(" -----------> Image published & sleeping for 8 seconds\n")
        # sleep(8)

            #publish image via app
        flag_msg = Flag()
        flag_msg.flag = image_flag
        # print(" -----------> Sending image \n")
        self.publisher_app.publish(flag_msg)
        self.get_logger().info(" -----------> Image requested & sleeping for %.3f seconds\n" % pick_hover_wait_time)
        sleep(pick_hover_wait_time)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick)
        # print(" -----------> Going to point pick\n")
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to point pick & sleeping for 3 seconds\n")
        for i in range(3):
            print("Sleeping for %d out of 3 seconds" % int(i+1))
            sleep(1)


        # self.open_gripper()
        # sleep(8)

        # Gripper closed
        self.close_gripper()
        sleep(8)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_pick_hover2)
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to pick hover 2 & sleeping for 3 seconds\n")
        for i in range(3):
            print("Sleeping for %d out of 3 seconds" % int(i+1))
            sleep(1)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_place_hover)
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to place hover 2 & sleeping for %.3f seconds\n" % travel_time)
        sleep(travel_time)


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_place)
        self.publish_with_checking(trajectory_msg)
        self.get_logger().info(" -----------> Going to point place & sleeping for 4 seconds\n")
        for i in range(4):
            print("Sleeping for %d out of 4 seconds" % int(i+1))
            sleep(1)


        #Gripper OPEN
        self.open_gripper()
        sleep(8)


        # change the image flag
        flag_msg = Flag()
        flag_msg.flag = 0       #### 0 = change image back to default
        # print(" -----------> Sending image \n")
        self.publisher_app.publish(flag_msg)
        self.get_logger().info(" -----------> Image stopped & sleeping for 8 seconds\n")
        # sleep(8)

        #self.open_gripper()


        trajectory_msg.points.clear()
        trajectory_msg.points.append(point_home2)
        self.publish_with_checking(trajectory_msg)
        # sleep(8)



def main(args=None):

    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    

    # Iterative interface
    for box_id in box_ids:

        signal = str(input("Current target is box #%d. Enter 'g' to start: " % box_id))
        if signal == 'g':
            print("running the publish_trajectory function now!")
            trajectory_publisher.publish_trajectory(box_id)
            print("Finished running for box_id #%d" % box_id)


    # # While it is running
    # while rclpy.ok():

    #     curr_box_id = int(input("Please enter the box_id to grasp: "))

    #     print("running the publish_trajectory function towards box_id #%d!" % curr_box_id)
    #     trajectory_publisher.publish_trajectory(curr_box_id)
    #     print("Finished running for box_id #%d" % curr_box_id)

    
    #rclpy.spin_until_future_complete(action_client, future)
    # rclpy.spin_once(trajectory_publisher)
    # rclpy.spin(trajectory_publisher)


    trajectory_publisher.destroy_node()
    rclpy.shutdown()

#if you are executing the script directly through terminal: (otherwise comment out)
if __name__ == '__main__':
    main()