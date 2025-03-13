import numpy as np
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import os 
import time
import shutil
import yaml
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
import struct


# Node for saving images from a camera to a specified folder
class SynchronizedDataNode(Node):
    def __init__(self):
        super().__init__('synchronized_data_node')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)
        
        # Publisher to re-publish lidar data with comparable timestamp
        self.lidar_pub = self.create_publisher(PointCloud2, '/ouster/points/timed', qos)
        
        # Camera directory counter for directory name
        self.nextDir = 0

        # create directory structure for data to be saved in
        self.createDirStructure()

        # listen to input from /image_raw and process it in callback function listener_callback
        self.image_sub = Subscriber(self, Image, '/image_raw', qos_profile=qos)
        self.lidar_sub = Subscriber(self, PointCloud2, '/ouster/points/timed', qos_profile=qos)
        
        # Synchronize input of images and lidar data
        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        
        self.create_subscription(PointCloud2, '/ouster/points', self.lidar_callback, qos)
        
        self.bridge = CvBridge()
        

    # Create new directory for data to be saved in
    def createDirStructure(self):
        baseTargetDir = os.path.dirname(os.path.realpath(__file__ + "/../../../")) + '/sampledata/raw/'
        if not os.path.exists(baseTargetDir):
            os.makedirs(baseTargetDir)

        # create meta.yaml in raw directory
        metaYamlRaw = {
            'entity': 'scan_project',
            'type': 'scan_project',
            'crs': '',
            'coordinate_system': 'right-handed',
            'unit': 'meter',
            'transformation': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            'name': ''
        }
        with open(baseTargetDir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlRaw, yaml_file, default_flow_style=None)

        # new directory name for files (= last directory name + 1)
        self.newDir = 0
        for dir in os.listdir(baseTargetDir):
            if dir != 'meta.yaml':
                if int(dir) > self.newDir:
                    self.newDir = int(dir)
        self.newDir += 1
        self.newDir = baseTargetDir + str(self.newDir).zfill(8) + '/'

        if not os.path.exists(self.newDir):
            os.makedirs(self.newDir)

        # Create meta.yaml in new directory
        metaYamlNewDir = {
            'barometric_amsl': 243.01679418504236,
            'dm': [-9.0561037058738127e-11, -1.2138630270337523e-09, -6980.3679730779722],
            'dm_conf': [0.00085025547556449529, 0.00085025547556441181, 249.94351132271046],
            'gnss': {
                'ageOfCorrections': 0,
                'altitude': 351.43301391601562,
                'antName': 'Int. GNSS with Cam.',
                'antPosSOCS': [0.020899999999999998, 0, 0.28470000000000001],
                'coordinateSystem': 'EPSG::4979',
                'fix': 1,
                'fixInfo': 'Single',
                'frameAngle': 35.85182291666667,
                'hmsl': 306.76101684570312,
                'horizontalAccuracy': 19.549001693725586,
                'horizontalDop': 1.4900000095367432,
                'latitude': 50.838180899999998,
                'longitude': 12.9240739,
                'numSatellites': 6,
                'positionAccuracy': 30.139999389648438,
                'positionDop': 2.6499998569488525,
                'rotating': True,
                'utcDateTime': '2022-07-19T14:27:07Z',
                'verticalAccuracy': 22.934001922607422,
                'verticalDop': 2.190000057220459
            },
            'gnssSOCS': {
                'altitude': 351.14887811336666,
                'latitude': 50.838181093974086,
                'longitude': 12.924074142593263
            },
            'gyro_offset': [16.4951171875, -28.09765625, 1.8994140625],
            'ku': [0, 0, 0],
            'ku_conf': [0, 0, 0],
            'magnReferencePosition': {
                'altitude': 351.43301391601562,
                'coordSystem': 'EPSG::4979',
                'latitude': 50.838180899999998,
                'longitude': 12.9240739
            },
            'mode': 'static',
            'mrot': [9935.2003752840574, 3431.9508997419503, -2.0824528612827431e-11],
            'mrot_conf': [130.78711754548857, 131.54123572648982, 0.00085025547556449204],
            'navigation': {
                'pitch': -0.5422376488972489,
                'pitch_conf': 0.01266945702056381,
                'roll': -2.2786103562505913,
                'roll_conf': 0.01266945702056381,
                'uoffset': [107.69557725414916, -95.049089990847307, 529.39369485408383],
                'uoffset_conf': [1.8095240415497507, 1.812234876371934, 5.4260638196052726],
                'uscale': 1,
                'uscale_conf': 0
            },
            'pitch': -0.5422376488972489,
            'pitch_conf': 0.01266945702056381,
            'roll': -2.2786103562505913,
            'roll_conf': 0.01266945702056381,
            'sscale': 0.74378143198983449,
            'sscale_conf': 0.0039952960016883974,
            'static_positions': 8,
            'uoffset': [107.69557725414916, -95.049089990847307, 529.39369485408383],
            'uoffset_conf': [1.8095240415497507, 1.812234876371934, 5.4260638196052726],
            'uscale': 1,
            'uscale_conf': 0,
            'yaw': 172.80407695034901,
            'yaw_conf': 1,
            'original_name': 21,
            'entity': 'scan_position',
            'type': 'scan_position',
            'pose_estimation': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            'transformation': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            'timestamp': -1
        }
        with open(self.newDir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlNewDir, yaml_file, default_flow_style=True)

        # create image target directory
        self.cam0Dir = self.newDir + 'cam_00000000/'

        if not os.path.exists(self.cam0Dir):
            os.makedirs(self.cam0Dir)

        # Create meta.yaml in camera directory
        # TODO: An Kamera anpassen
        metaYamlCam = {
            "entity": "sensor",
            "type": "camera",
            "name": "",
            "transformation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "model": {
                "entity": "model",
                "type": "pinhole",
                "intrinsic": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
                "resolution": [0, 0],
                "distortion_model": "opencv",
                "distortion_coefficients": []
            }
        }
        with open(self.cam0Dir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlCam, yaml_file, default_flow_style=None)

        # Create <img>.yaml in camera directory
        metaYamlImgDir = {
            "entity": "sensor_data_group",
            "type": "camera_images",
            "transformation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        }
        with open(self.cam0Dir + '00000000.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlImgDir, yaml_file, default_flow_style=None)

        # create scan target directory
        self.lidar0Dir = self.newDir + 'lidar_00000000/'

        if not os.path.exists(self.lidar0Dir):
            os.makedirs(self.lidar0Dir)

    
    # Add comparable timestamp to lidar data
    def lidar_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_pub.publish(msg)
        
    # Process synchronized data
    def sync_callback(self, cam, lidar):
        self.save_image(cam)
        self.save_lidardata(lidar)
        self.nextDir += 1


    # Save camera image
    def save_image(self, msg):
        # Create new directory for image to be saved in
        self.imgTargetDir = self.cam0Dir + str(self.nextDir).zfill(8) + '/'
        # self.imgTargetDir = self.cam0Dir + str(int(self.nextTimeStamp * 10)) + '/'
        os.makedirs(self.imgTargetDir)

        # write current camera image to specified folder
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        imageFile = self.imgTargetDir + 'image_00000000.png'
        imageMeta = self.imgTargetDir + 'meta_00000000.yaml'

        cv2.imwrite(imageFile, cv_image) 

        # Timestamp in nanoseconds
        timestamp = msg.header.stamp.nanosec

        # Create meta png yaml
        # TODO: An Kamera anpassen
        metaYamlPng = {
            "frameAngle": 323.99996948242188, 
            "lineAngle": 90, 
            "transformation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], 
            "pose_estimation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], 
            "timestamp": timestamp, 
            "entity": "sensor_data", 
            "type": "camera_image", 
            "resolution": [0, 0, 3]
        }
        with open(self.imgTargetDir + 'meta_png.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlPng, yaml_file, default_flow_style=True)

        # Create yaml for image
        # TODO: An Kamera anpassen
        metaYamlImg = {
            "frameAngle": 35.999996185302734, 
            "lineAngle": 90, 
            "transformation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], 
            "pose_estimation": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], 
            "timestamp": timestamp, 
            "entity": "sensor_data", 
            "type": "camera_image", 
            "resolution": [0, 0, 3]
        }
        with open(imageMeta, 'w') as yaml_file:
            yaml.dump(metaYamlImg, yaml_file, default_flow_style=True)


    # Save lidar data
    def save_lidardata(self, msg):
        print("--- Getting Lidar data ---")
        # Create new directory for lidar data to be saved in
        self.lidarTargetDir = self.lidar0Dir + str(self.nextDir).zfill(8) + '/'
        os.makedirs(self.lidarTargetDir)
        
        # Timestamp in nanoseconds
        timestamp = msg.header.stamp.nanosec
        
        # TODO: Add yaml files
        
        # Create pointcloud from incoming data
        points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True))

        amount_points = len(points)
        
        header_intensities = f'{{"TYPE": "FLOAT", "SHAPE": [{amount_points},1]}};'
        header_points = f'{{"TYPE": "FLOAT", "SHAPE": [{amount_points},3]}};'

        intensities_path = os.path.join(self.lidarTargetDir, "intensities.data")
        points_path = os.path.join(self.lidarTargetDir, "points.data")

        # Write data into .data files
        with open(intensities_path, "wb") as data_file_intensities, open(points_path, "wb") as data_file_points:
            # Add header
            data_file_intensities.write(header_intensities.encode('utf-8'))
            data_file_points.write(header_points.encode('utf-8'))
            
            for point in points:
                x, y, z, intensity = point
                data_file_intensities.write(struct.pack("f", intensity))
                data_file_points.write(struct.pack("fff", x, y, z))


def main(args=None):
    # start node
    rclpy.init(args=args)
    synchronized_data = SynchronizedDataNode()
    rclpy.spin(synchronized_data)

    # stop node
    synchronized_data.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
