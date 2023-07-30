#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Homing, Move
from time import sleep
from math import floor

import csv




from tutorial_interfaces.msg import Flag, MyConfig


GIGA = 1e9

#Joint limits for Safety 
lower_joint_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
upper_joint_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


#Trajectories for each pick_and_place round 

TJ1 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.692855558167406, -0.031825485408802444, 0.2645964320684744, -2.791755121867545, 0.03278677458583874, 2.759731178275691, 1.6749953547651055], 
    "pick_down": [0.6955110798740082, 0.17227346213413142, 0.2357867527342614, -2.7575197029709217, 0.03327169493965579, 2.9047469232848497, 1.6688220709581738], 
    "pick_hover":[0.692855558167406, -0.031825485408802444, 0.2645964320684744, -2.791755121867545, 0.03278677458583874, 2.759731178275691, 1.6749953547651055],
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ2 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.786713804839311, 0.19385285552931764, 0.27632609641172406, -2.4826266474702576, -0.036695691829446056, 2.643415397996074, 1.8473523196316204], 
    "pick_down": [0.7913833179469447, 0.34291284116867377, 0.250890697335531, -2.440987002315242, -0.03670663859436242, 2.720697628938791, 1.8184151395863586], 
    "pick_hover":[0.786713804839311, 0.19385285552931764, 0.27632609641172406, -2.4826266474702576, -0.036695691829446056, 2.643415397996074, 1.8473523196316204], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ3 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.8008699109039782, 0.07301470155106025, 0.026954735334214483, -2.655276026699324, -0.041939082666088486, 2.726736576555586, 1.6379421249619202], 
    "pick_down": [0.8008173027038632, 0.24097551183062382, 0.025758841439122927, -2.6174289258110504, -0.041942447244985526, 2.8318295350053897, 1.626246827201089], 
    "pick_hover":[0.8008699109039782, 0.07301470155106025, 0.026954735334214483, -2.655276026699324, -0.041939082666088486, 2.726736576555586, 1.6379421249619202],
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ4 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.829002895295599, 0.2518092767357725, 0.13408721889798145, -2.3776301009244336, -0.039981570836241764, 2.6113081684118105, 1.7782435037163862], 
    "pick_down": [0.8363393160753297, 0.3886204716534108, 0.12024573014869352, -2.3559177551018515, -0.040619071033295914, 2.7195453900993947, 1.7545289841578797], 
    "pick_hover":[0.829002895295599, 0.2518092767357725, 0.13408721889798145, -2.3776301009244336, -0.039981570836241764, 2.6113081684118105, 1.7782435037163862], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ5 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.8664263468424315, 0.1324476047950334, -0.13192654373862173, -2.536809741150129, -0.03483336699011202, 2.7039187629360866, 1.5466355307527875], 
    "pick_down": [0.870959021772894, 0.30553792865355234, -0.12554749682332803, -2.523319232865486, -0.034826015564199585, 2.8565273193317613, 1.5425674678162118], 
    "pick_hover":[0.8664263468424315, 0.1324476047950334, -0.13192654373862173, -2.536809741150129, -0.03483336699011202, 2.7039187629360866, 1.5466355307527875], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ6 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.8284068694484286, 0.31563376307109886, 0.05054645114218371, -2.2613523910264783, -0.027917037379968607, 2.5642111977616087, 1.6739232945967575], 
    "pick_down": [0.8346349560062076, 0.44392968907984465, 0.037647485805243824, -2.2216185951215706, -0.033306573059047814, 2.6380558731080876, 1.6670963135253924], 
    "pick_hover":[0.8284068694484286, 0.31563376307109886, 0.05054645114218371, -2.2613523910264783, -0.027917037379968607, 2.5642111977616087, 1.6739232945967575], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }


TJ7 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.8295166602353828, 0.2502591169728821, -0.17074352631019443, -2.381098331011856, -0.012211921982277807, 2.6721870553378073, 1.486887120995305], 
    "pick_down": [0.8295193076605679, 0.39651625436988863, -0.16341982653004256, -2.3781028330446343, -0.012190060595786562, 2.84470178893632, 1.458873316848614], 
    "pick_hover":[0.8295166602353828, 0.2502591169728821, -0.17074352631019443, -2.381098331011856, -0.012211921982277807, 2.6721870553378073, 1.486887120995305], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ8 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.6316659289942639, 0.42822201133685955, 0.16342496168045237, -2.060195135090972, -0.1030481092367054, 2.439635359735207, 1.6101745882778866], 
    "pick_down": [0.6319427701761905, 0.539014475524197, 0.1560667329671316, -2.057520155767496, -0.10301112312350642, 2.5420407516930483, 1.5880923778194715], 
    "pick_hover":[0.6316659289942639, 0.42822201133685955, 0.16342496168045237, -2.060195135090972, -0.1030481092367054, 2.439635359735207, 1.6101745882778866], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ9 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.6293224911107395, 0.36018139518444764, -0.044389919138178247, -2.1856469310395608, 0.018919492226506227, 2.5715949417950505, 1.3934300874766954], 
    "pick_down": [0.6320421473786435, 0.48872565800790274, -0.04491327985640215, -2.16968699281308, 0.018907493837867415, 2.6858445008803455, 1.3449440696506774], 
    "pick_hover":[0.6293224911107395, 0.36018139518444764, -0.044389919138178247, -2.1856469310395608, 0.018919492226506227, 2.5715949417950505, 1.3934300874766954],
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }

TJ10 = {
    "home":[0.48367694844501913, -0.3030616318398186, -0.5229186330040432, -2.3766162168751475, -0.18668388709686684, 2.0748756006017803, 0.8758746603421096], 
    "pick_hover":[0.8007202372847209, 0.5095106127165671, -0.08217522296621782, -1.9338736469883688, 0.016744317705654415, 2.4531447425674693, 1.5324661656092409], 
    "pick_down": [0.8006017067810978, 0.6177665980460825, -0.07752797953347676, -1.9276800056137855, 0.016766618850804493, 2.571633566895307, 1.4836206438234918], 
    "pick_hover":[0.8007202372847209, 0.5095106127165671, -0.08217522296621782, -1.9338736469883688, 0.016744317705654415, 2.4531447425674693, 1.5324661656092409], 
    "place_hover": [-0.0737010309183906, 0.5348086683724749, -0.23783670820799785, -1.5686379855403472, 0.12078098163196738, 2.0961449675840016, 0.41571346794998365],
    "place_down": [-0.0711010494361804, 0.7973276222806459, -0.20880710476954248, -1.5694039211525184, 0.14360818026794253, 2.3493687043093217, 0.43407348498047793]
    }


tjs_list =  [TJ1, TJ2, TJ3, TJ4, TJ5, TJ6, TJ7, TJ8, TJ9, TJ10]



###########################   Config 

max_number_of_boxes = len(tjs_list)


# three uncertainty conditions
uncertainties_low = [8,12,14,7,7,5,9,46,45]  # this is size 10
uncertainties_high = [71,98,75,62,96,83,22,52,46,45]
uncertainties_no = [0,0,0,0,0,0,0,0,0,0]



class TrajectoryPublisher(Node):

    def __init__(self):

        super().__init__('trajectory_publisher')
        self.subscriber_config = self.create_subscription(MyConfig,'myconfig',self.setup_expe_config ,10)

        self.publisher_arm = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(String,'go',self.callback,10)
        self.action_client_grasp = ActionClient(self, Grasp, 'panda_gripper/grasp')
        self.action_client_homing = ActionClient(self, Homing, 'panda_gripper/homing')
        self.action_client_move = ActionClient(self, Move, 'panda_gripper/move')

        # self.publisher_app = self.create_publisher(String, '/image_name', 4)

        self.publisher_app = self.create_publisher(Flag, 'image_flag', 4) 

        self.times = []
        
        self.participant = 0


        # modality
        self.modality = 'holder'

        # session
        self.session = 0

        # box ids
        self.starting_box = 0
        
        # uncertainty level
        self.uncertaintylevel = 'no'
        self.current_uncertainty_level = uncertainties_low



    def setup_expe_config(self,msg):

        self.get_logger().info('******************* GOT CONFIG')
        current_config = msg
        
        #config current_box
        self.starting_box = current_config.box
        print(current_config.box)

        #config uncertaintylevel
        self.uncertaintylevel = current_config.uncertaintylevel
        if(self.uncertaintylevel == 'no'):
            self.current_uncertainty_level = uncertainties_no
        elif (self.uncertaintylevel == 'low'):
            self.current_uncertainty_level = uncertainties_low
        elif (self.uncertaintylevel == 'high'):
            self.current_uncertainty_level = uncertainties_high


        ### particpant
        self.participant = current_config.participant


        ### session
        self.participant = current_config.participant


        ### modality
        print(type(current_config.modality))
        print(current_config.modality)

        self.modality = current_config.modality




        # with open('logging.csv', 'w+', newline='') as file:
        #     fieldnames = ['participant', 'modality','session','box','uncertaintylevel']
        #     writer = csv.DictWriter(file, fieldnames=fieldnames)
        #     self.get_logger().info('******************* CSV WRITER')

        #     writer.writeheader()
        #     writer.writerow({'participant': participant, 'modality': self.modality, 'session': session, 'box': starting_box, "uncertaintylevel":uncertaintylevel})
        #     file.close()

        # return



    def get_wait_time(self, uncertainty):
        a = 0.05
        b = 1
        wait_time = a * uncertainty + b
        return wait_time
    

    def get_travel_time(self, uncertainty):
        a = 0.06
        b = 6
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
        
        # get the current config 
        current_trj = tjs_list[box_id-1]
        un = self.current_uncertainty_level[box_id-1]
        image_flag = un

        # get the wait times based on the uncertainty
        pick_hover_wait_time = self.get_wait_time(un)
        travel_time = self.get_travel_time(un)

        travel_time_s = floor(travel_time)
        travel_time_ns = int((travel_time - travel_time_s) * GIGA)
        
        print(travel_time_s)
        print(travel_time_ns)


        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']


        print("=" * 100)
        print("\n")
        print("The current modality is %s" % self.modality)
        print("\n")
        print("=" * 100)

        print("=" * 100)
        print("\n")
        print("The current box is %s" % box_id)
        print("\n")
        print("=" * 100)


        ####################################################################################################
        if self.modality == 'GUI':

            # ******* Start ******
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
            point_pick_hover.time_from_start.sec = 6        # this was 8
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
            point_place_hover.time_from_start.sec = 8    # NOT travel_time_s
            point_place_hover.time_from_start.nanosec = 0   # NOT travel_time_s

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



            #****************** Trajectory START ***********************************************
            

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
            self.get_logger().info(" -----------> Going to pick hover & sleeping for 6 seconds\n")
            for i in range(6):
                print("Sleeping for %d out of 6 seconds" % int(i+1))
                sleep(1)


            #publish image via app
            flag_msg = Flag()
            flag_msg.flag = image_flag
            # print(" -----------> Sending image \n")
            self.publisher_app.publish(flag_msg)
            self.get_logger().info(" -----------> Image requested & sleeping for 2 seconds\n") # NOT% pick_hover_wait_time
            sleep(2) # NOT sleep(pick_hover_wait_time)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_pick)
            # print(" -----------> Going to point pick\n")
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to point pick & sleeping for 2 seconds\n")
            for i in range(2):
                print("Sleeping for %d out of 2 seconds" % int(i+1))
                sleep(1)


            # Gripper closed
            self.close_gripper()
            sleep(3)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_pick_hover2)
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to pick hover 2 & sleeping for 2 seconds\n")
            for i in range(2):
                print("Sleeping for %d out of 2 seconds" % int(i+1))
                sleep(1)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_place_hover)
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to place hover & sleeping for 8 seconds\n")
            sleep(8)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_place)
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to point place & sleeping for 4 seconds\n")
            for i in range(4):
                print("Sleeping for %d out of 4 seconds" % int(i+1))
                sleep(1)


            #Gripper OPEN
            self.open_gripper()
            sleep(3)


            # change the image flag
            flag_msg = Flag()
            flag_msg.flag = -1       #### 0 = change image back to default
            # print(" -----------> Sending image \n")
            self.publisher_app.publish(flag_msg)
            self.get_logger().info(" -----------> Image stopped & sleeping for 8 seconds\n")
            # sleep(8)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_home2)
            self.publish_with_checking(trajectory_msg)
            # sleep(1)


            with open('logging_'+self.participant+'.csv', 'w+', newline='') as file:
                fieldnames = ['participant', 'modality','session','box','uncertaintylevel']
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                self.get_logger().info('******************* CSV WRITER')

                writer.writeheader()
                writer.writerow({'participant': self.participant, 'modality': self.modality, 'session': self.session, 'box': self.box_id, "uncertaintylevel":self.uncertaintylevel})
                file.close()



        ####################################################################################################
        else:

            # ******* Start ******
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



            #****************** Trajectory START ***********************************************
            
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


            self.get_logger().info(" -----------> No image and sleeping for %.3f seconds\n" % pick_hover_wait_time)
            sleep(pick_hover_wait_time)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_pick)
            # print(" -----------> Going to point pick\n")
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to point pick & sleeping for 2 seconds\n")
            for i in range(2):
                print("Sleeping for %d out of 2 seconds" % int(i+1))
                sleep(1)


            # Gripper closed
            self.close_gripper()
            sleep(3)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_pick_hover2)
            self.publish_with_checking(trajectory_msg)
            self.get_logger().info(" -----------> Going to pick hover 2 & sleeping for 2 seconds\n")
            for i in range(2):
                print("Sleeping for %d out of 2 seconds" % int(i+1))
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
            sleep(3)


            trajectory_msg.points.clear()
            trajectory_msg.points.append(point_home2)
            self.publish_with_checking(trajectory_msg)
            # sleep(8)


            with open('logging_'+self.participant+'.csv', 'w+', newline='') as file:
                fieldnames = ['participant', 'modality','session','box','uncertaintylevel']
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                self.get_logger().info('******************* CSV WRITER')

                writer.writeheader()
                writer.writerow({'participant': self.participant, 'modality': self.modality, 'session': self.session, 'box': self.box_id, "uncertaintylevel":self.uncertaintylevel})
                file.close()





def main(args=None):

    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin_once(trajectory_publisher)
    
    sleep(3)
    # Iterative interface
    for box_id in range(trajectory_publisher.starting_box, max_number_of_boxes):

        trajectory_publisher.get_logger().info("*****CURRENT BOX: #%d "% box_id)
        signal = str(input("Current target is box #%d. Enter 'g' to start: " % box_id))

        if signal == 'g':
            print("running the publish_trajectory function now!")
            trajectory_publisher.publish_trajectory(box_id)
            print("Finished running for box_id #%d" % box_id)




    trajectory_publisher.destroy_node()
    
    rclpy.shutdown()

#if you are executing the script directly through terminal: (otherwise comment out)
if __name__ == '__main__':
    main()