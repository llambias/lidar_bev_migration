import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from cv_bridge import CvBridge, toCvCopy
import cv2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan



class BirdsEye(Node):

    def __init__(self):
        super().__init__('birds eye')
        self.create_comunications()
        self.scan_ = LaserScan()

    def create_comunications(self):
        # Create publishers
        self.birds_eye_pub = self.create_publisher(Image, 'birds_eye', 10)
        self.color_mask_pub_ = self.create_publisher(Image, 'color_mask', 10)
        self.grid_pub_ = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

        # Create subscribers
        self.image_sub_ = self.create_subscription(Image, 'camera/image_raw', self.imageCb, 10)
        self.scan_sub_ = self.create_subscription(LaserScan, 'scan', self.scanCb, 10)

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    def imageCb(self,msg):
    
        #Convert to cv image

        try:
            cv_ptr = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
        
            self.get_logger().error(f"cv_bridge exception: {e.wath}")
            return
        
        frame = cv_ptr

        ## Compute and publish birds eye
        self.birdsEye(frame)
        msg_out = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')
        
        self.birds_eye_pub.publish(msg_out)

        ## Compute and publish color mask
        self.colorMask(frame)
        msg_out = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')
        self.color_mask_pub_.publish(msg_out)

        ## Create occupancy grid
        grid = OccupancyGrid()
        self.makeOccupancyGrid(frame, self.scan_, grid)
        self.grid_pub_.publish(grid)
    
    def birdsEye(self, image):
        # ...resto del código a migrar... ##TODO
        pass

    def colorMask(self, image):
        # ...resto del código a migrar... ##TODO
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = BirdsEye()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
