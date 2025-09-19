#!/usr/bin/env python

# detections_refinement.py: Node for online refinement of detections (BirdNet 1 only)

import sys
import rclpy
from rclpy.node import Node
import datetime
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from perception_msgs.msg import ObstacleList
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from birdview_detection_refiner import BirdviewDetectionRefiner


class detections_refinement(Node):
    def __init__(self):
        super().__init__('detections_refinement')

        self.refined_pub = self.create_publisher('/refined_obstacle_list', ObstacleList, queue_size=1)

        bvres = 0.05
        self.lidar_h = 1.73 # TODO Change to use TF
        only_front = False
        self.count = 0
        self.ms = 0
        self.velo_pub = self.create_publisher('/velo_sync', PointCloud2, queue_size=1)


    def callback(self, obstaclelist, bird_view, bird_ground, velo_cloud):
        a = datetime.datetime.now()
        bridge = CvBridge()
        bv_image = []
        bv_ground = []
        try:
            bv_image = bridge.imgmsg_to_cv2(bird_view, "bgr8")
            bv_ground = bridge.imgmsg_to_cv2(bird_ground, "32FC1")
        except CvBridgeError as e:
            print(e)
            sys.exit(-1)

        # Init BV Detection Refiner
        bv_refiner = BirdviewDetectionRefiner(bv_image, bv_ground, bvres, self.lidar_h, only_front)

        refined_list = ObstacleList()
        refined_list.header = obstaclelist.header
        refined_list.header.frame_id = 'velodyne'

        for obj in obstaclelist.obstacles:
            # Filter detections with a very low score
            if obj.score < 0.1: # Remove highly unlikely detections
                continue

            try:
                # Refine detection and append to list
                bv_refiner.refine_detection(obj)
                refined_list.obstacles.append(obj)
            except Exception as e:
                self.get_logger().error(e)

        self.refined_pub.publish(refined_list)
        self.velo_pub.publish(velo_cloud)

        b = datetime.datetime.now()
        delta = b - a

        self.ms += delta.microseconds
        self.count += 1
        self.get_logger().info('average ms: {}'.format(self.ms / self.count / 1000.0))


def main(args):
    global bvres, lidar_tf_frame, camera_tf_frame, max_height, only_front
    # Initializes and cleanup ros node
    rclpy.init_node('detections_refinement', anonymous=True)
    rclpy.get_logger().info("[detections_refinement] Ready")
    bvres = rclpy.get_param("~cell_size", 0.05)
    max_height = rclpy.get_param("~max_height", 3.0)
    obstacles_topic = rclpy.get_param("~obstacles_list", '/obstacle_list')
    birdview_topic = rclpy.get_param("~bird_view", '/bird_view')
    birdground_topic = rclpy.get_param("~bird_ground", '/bird_ground')
    lidar_tf_frame = rclpy.get_param("~lidar_tf_frame", 'velodyne')
    camera_tf_frame = rclpy.get_param("~camera_tf_frame", 'stereo_camera')
    only_front = rclpy.get_param("~only_front", False)
    print('Using only front part of BEV: {}'.format(only_front))

    velo_topic = '/velodyne_points'

    obstacle_sub = message_filters.Subscriber(obstacles_topic, ObstacleList)
    birdview_sub = message_filters.Subscriber(birdview_topic, Image)
    birdground_sub = message_filters.Subscriber(birdground_topic, Image)
    velo_sub = message_filters.Subscriber(velo_topic, PointCloud2)

    ts = message_filters.TimeSynchronizer([obstacle_sub, birdview_sub, birdground_sub, velo_sub], 10)
    ts.registerCallback(callback)
    rclpy.spin()

if __name__ == '__main__':
    main(sys.argv)