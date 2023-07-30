import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Flag 
from std_msgs.msg import String




class ImagePublisher(Node):

    def __init__(self):

        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(String, 'image_name', 4)
        timer_period = 0.1        # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self.subscription = self.create_subscription(Flag,'image_flag', self.flag_callback, 10)
        self.subscription

        self.image_flag = 0


    def publish_callback(self):
        
        if self.image_flag == -1:
            # publish default image via app
            img_msg = String()
            img_msg.data = "image.png"
            self.publisher_.publish(img_msg)
            self.get_logger().info(" -----------> Publishing (default) image.png \n")

        else : 
            # publish image via app
            img_msg = String()
            img_msg.data = "image_"+str(self.image_flag)+".jpg"
            self.publisher_.publish(img_msg)
            self.get_logger().info(" -----------> Publishing image2.jpg \n")
        

    def flag_callback(self, msg):
        
        flag = msg.flag
        self.get_logger().info('I heard: "%d"' % flag)
        self.image_flag = flag



def main(args=None):

    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()