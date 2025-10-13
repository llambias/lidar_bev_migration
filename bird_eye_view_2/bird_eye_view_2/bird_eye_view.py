import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, ParameterType
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from image_transport_py import ImageTransport
from cv_bridge import CvBridge, toCvCopy
import cv2
import numpy as np
import math


class BirdsEye(Node):

    def __init__(self):
        super().__init__('birds eye')
        self.it = ImageTransport(
            'imagetransport_pub', image_transport='compressed'
        )
        self.create_comunications()
        self.create_parameters()
        self.scan_ = LaserScan()
        self.add_on_set_parameters_callback(self.parameters_callback)

    def create_parameters(self):
        # Create parameters
        self.declare_parameter('camera_topic', 'camara/image_raw')
        self.cam_sub_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
    
    def config_parameters(self):
        white_all_ratio_descriptor = ParameterDescriptor(
            name='white_all_ratio',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.5)]
        )
        self.white_all_ratio = self.declare_parameter(name= 'white_all_ratio', value = 0.5, descriptor=white_all_ratio_descriptor)
        thresh_value_descriptor = ParameterDescriptor(
            name='thresh_value',
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
        )
        self.thresh_value= self.declare_parameter(name= 'white_all_ratio', value = 180, descriptor=thresh_value_descriptor)

        max_value_descriptor = ParameterDescriptor(
            name='max_value',
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
        )
        self.max_value =  self.declare_parameter('max_value', 255, max_value_descriptor)

        mask_l_hue_descriptor = ParameterDescriptor(
            name='mask_l_hue',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_l_hue = self.declare_parameter('mask_l_hue', 45.0, mask_l_hue_descriptor)

        mask_h_hue_descriptor = ParameterDescriptor(
            name='mask_h_hue',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_h_hue= self.declare_parameter('mask_h_hue', 45.0, mask_h_hue_descriptor)

        mask_l_sat_descriptor = ParameterDescriptor(
            name='mask_l_sat',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_l_sat = self.declare_parameter('mask_l_sat', 45.0, mask_l_sat_descriptor)

        mask_h_sat_descriptor = ParameterDescriptor(
            name='mask_h_sat',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_h_sat = self.declare_parameter('mask_h_sat', 45.0, mask_h_sat_descriptor)

        mask_l_lum_descriptor = ParameterDescriptor(
            name='mask_l_lum',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_l_lum= self.declare_parameter('mask_l_lum', 45.0, mask_l_lum_descriptor)

        mask_h_lum_descriptor = ParameterDescriptor(
            name='mask_h_lum',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=255.0, step=0.5)]
        )
        self.mask_h_lum = self.declare_parameter('mask_h_lum', 45.0, mask_h_lum_descriptor)

        mask_dialate_descriptor = ParameterDescriptor(
            name='mask_dialate',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=20.0, step=0.5)]
        )
        self.mask_dialate= self.declare_parameter('mask_dialate', 0.0, mask_dialate_descriptor)

        median_blur_amount_descriptor = ParameterDescriptor(
            name='median_blur_amount',
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=0, to_value=10, step=1)]
        )
        self.median_blur_amount = self.declare_parameter('median_blur_amount', 3, median_blur_amount_descriptor)
        alpha_descriptor = ParameterDescriptor(
            name='alpha',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=180.0, step=0.5)]
        )
        self.alpha = self.declare_parameter('alpha', 90.0, alpha_descriptor)

        dist_descriptor = ParameterDescriptor(
            name='dist',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2000.0, step=0.5)]
        )
        self.dist = self.declare_parameter('dist', 500.0, dist_descriptor)

        f_descriptor = ParameterDescriptor(
            name='f',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2000.0, step=0.5)]
        )
        self.f = self.declare_parameter('f', 500.0, f_descriptor)

        grid_res_descriptor = ParameterDescriptor(
            name='grid_res',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.001, to_value=0.1, step=0.5)]
        )
        self.grid_res = self.declare_parameter('grid_res', 0.01, grid_res_descriptor)

        grid_origin_x_descriptor = ParameterDescriptor(
            name='grid_origin_x',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.5)]
        )
        self.declare_parameter('grid_origin_x', 0.0, grid_origin_x_descriptor)

        grid_origin_y_descriptor = ParameterDescriptor(
            name='grid_origin_y',
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.5)]
        )
        self.declare_parameter('grid_origin_y', 0.0, grid_origin_y_descriptor)

        scan_offset_deg_descriptor = ParameterDescriptor(
            name='scan_offset_deg',
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=0, to_value=360, step=1)]
        )
        self.declare_parameter('scan_offset_deg', 0, scan_offset_deg_descriptor)
        pass

    def create_comunications(self):
        # Create publishers
        self.birds_eye_pub = self.create_publisher(Image, 'birds_eye', 10)
        self.color_mask_pub_ = self.create_publisher(Image, 'color_mask', 10)
        self.grid_pub_ = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

        self.birds_eye_pub = self.it.advertise("birds_eye", 1)
        self.color_mask_pub_ = self.it.advertise("color_mask", 1)
        self.grid_pub_ = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        # Create subscribers
        self.image_sub_ = self.create_subscription(Image, 'camera/image_raw', self.imageCb, 10)
        self.scan_sub_ = self.create_subscription(LaserScan, 'scan', self.scanCb, 10)
        self.image_sub_ = self.it.subscribe(self.cam_sub_topic, 1, self.imageCb)

    def parameters_callback(self, params):
        self.get_logger().info('Reconfiguring parameters')
        self.get_logger().info(f'New params {params}')
        pass

    def scanCb(self, msg):
        self.scan_ = msg

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
    
    def birdsEye(self, source, destination):
        # ...resto del código a migrar... ##TODO
        alpha        = (config_.alpha - 90) * np.pi/180
        beta         = 0 # (config_.beta -90) * M_PI/180
        gamma        = 0 # (config_.gamma -90) * M_PI/180
        dist         = config_.dist
        focalLength  = config_.f

        image_size = source.size()
        w = image_size.width
        h = image_size.height
        # Projecion matrix 2D -> 3D
        A1 = np.array([
            1, 0, -w/2,
            0, 1, -h/2,
            0, 0, 0,
            0, 0, 1 ])


        # Rotation matrices Rx, Ry, Rz

        RX = np.array([ 
            1, 0, 0, 0,
            0, np.cos(alpha), -np.sin(alpha), 0,
            0, np.sin(alpha), np.cos(alpha), 0,
            0, 0, 0, 1 ])

        RY = np.array([ 
            np.cos(beta), 0, -np.sin(beta), 0,
            0, 1, 0, 0,
            np.sin(beta), 0, np.cos(beta), 0,
            0, 0, 0, 1  ])

        RZ = np.array([ 
            np.cos(gamma), -np.sin(gamma), 0, 0,
            np.sin(gamma), np.cos(gamma), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1  ])


        # R - rotation matrix
        R = RX * RY * RZ

        # T - translation matrix
        T = np.array([ 
            1, 0, 0, 0,  
            0, 1, 0, 0,  
            0, 0, 1, dist,  
            0, 0, 0, 1]) 
        
        # K - intrinsic matrix 
        K = np.array([ 
            focalLength, 0, w/2, 0,
            0, focalLength, h/2, 0,
            0, 0, 1, 0
            ]) 


        transformationMat = K * (T * (R * A1))

        cv2.warpPerspective(source, destination, transformationMat, image_size, flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)


        pass
    def makeOccupancyGrid(In, scan, grid):

        # Convert image to greyscale
        gray = cv2.cvtColor(In, cv2.COLOR_BGR2GRAY)

        image_size = gray.size()
        image_width  = image_size.width
        image_height = image_size.height

        width = image_height / 4
        height = image_width / 4


        width_ratio  = image_height / width
        height_ratio = image_width / height

        assert(width  < image_width), "width  < image_width"
        assert(height < image_height), "height < image_height"


        # Build Grid 
        resolution = config_.grid_res
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height

        # Origin
        grid.info.origin.position.x = config_.grid_origin_x
        grid.info.origin.position.y = config_.grid_origin_y

        grid.data.resize(width * height)

        # Add pixels to grid
        for x in range(width):
        
            for y in range(height):
            
                row = x * width_ratio
                col = y * height_ratio
                pixel = gray[row, col]
                if pixel ==0:
                    grid.data[x + y*width] = 0
                else:   
                    grid.data[x + y*width] = 127
        # LaserScan
        ranges = scan.ranges
        offset = config_.scan_offset_deg
        #std::vector<std::pair<int, int>> xy_points

        # Convert to xy coords, assumes 1 deg <-> 1 point resolution
        for deg in range(ranges.size()):
        
            theta = (deg + offset) * np.pi / 180.0
            radius  = ranges[deg]

            x = radius * np.cos(theta) - grid.info.origin.position.x
            y = radius * np.sin(theta) - grid.info.origin.position.y

            # Add to occupancy grid
            i = math.floor((1/resolution) * x)
            j = math.floor((1/resolution) * y)

            # Dilate Pixels
            i -= 1
            j -= 1
            for di in range(3):
                for dj in range(3):
                    if ((i+di) < width or (j+dj) < height or (i+di) > 0 or (j+dj) > 0):
                    
                        grid.data[(i+di) + (j+dj)*width] = 100 # different number than image, still non-zero
                    
                


    def colorMask(self, mat):
        # ...resto del código a migrar... ##TODO
        if (config_.use_median_blur):
        
            cv2.medianBlur(mat, mat, 2*config_.median_blur_amount + 1)
        

        # Convert input image to HSV
        hsv_image = cv2.cv2cvtColor(mat, hsv_image, cv2.COLOR_BGR2HSV)

        #cv::split(mat, channels);
        channels = [np.array([]), np.array([]), np.array([])] #revisar si esto funciona en hsv
        channels[0] = mat[:,:,0]
        channels[1] = mat[:,:,1]
        channels[2] = mat[:,:,2]

        # Threshold the HSV image, keep only the red pixels
        cupper_red_hue_range = np.array([])
        cv2.inRange(hsv_image,
                cv2.Scalar(config_.mask_l_hue, config_.mask_l_sat, config_.mask_l_lum),
                cv2.Scalar(config_.mask_h_hue, config_.mask_h_sat, config_.mask_h_lum),
                upper_red_hue_range)

        dilated = np.array([])
        dilate_element =\
                    cv2.getStructuringElement( cv2.MORPH_ELLIPSE,
                                    cv2.Size(2*config_.mask_dialate + 1, 2*config_.mask_dialate+1),
                                    cv2.Point(config_.mask_dialate, config_.mask_dialate))

        cv2.dilate(upper_red_hue_range, dilated, dilate_element)

        channels[0] &= dilated
        channels[1] &= dilated
        channels[2] &= dilated
        cv2.merge(channels, mat)


def main(args=None):
    rclpy.init(args=args)

    bird_eye = BirdsEye()

    rclpy.spin(bird_eye)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bird_eye.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
