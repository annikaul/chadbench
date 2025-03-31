import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import os 
import yaml
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from message_filters import Subscriber, ApproximateTimeSynchronizer
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
        self.lidar_path_pub = self.create_publisher(TransformStamped, 'tf/timed', qos)
        
        # Camera directory counter for directory name
        self.nextDir = 0

        # create directory structure for data to be saved in
        self.createDirStructure()

        # listen to input from /image_raw and process it in callback function listener_callback
        self.image_sub = Subscriber(self, Image, '/image_raw', qos_profile=qos)
        self.lidar_sub = Subscriber(self, PointCloud2, '/ouster/points/timed', qos_profile=qos)
        self.lidar_path_sub = Subscriber(self, TransformStamped, '/tf/timed', qos_profile=qos)
        
        # Synchronize input of images and lidar data
        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub, self.lidar_path_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        
        self.create_subscription(PointCloud2, '/ouster/points', self.lidar_callback, qos)
        self.create_subscription(TFMessage, '/tf', self.lidar_transformation_callback, qos)
        
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
        } # Add information here if needed
        
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
            'entity': 'scan_position',
            'type': 'scan_position',
        } # Add information here if needed
        
        with open(self.newDir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlNewDir, yaml_file, default_flow_style=True)

        # create image target directory
        self.cam0Dir = self.newDir + 'cam_00000000/'

        if not os.path.exists(self.cam0Dir):
            os.makedirs(self.cam0Dir)

        # Create meta.yaml in camera directory
        metaYamlCam = {
            "entity": "sensor",
            "type": "camera",
        } # Add information here if needed
        
        with open(self.cam0Dir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlCam, yaml_file, default_flow_style=None)

        # Create yaml for camera directory
        metaYamlImgDir = {
            "entity": "sensor_data_group",
            "type": "camera_images",
        } # Add information here if needed
        
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
        
    # Add comparable timestamp to lidar transformation
    def lidar_transformation_callback(self, msg):
        for transform in msg.transforms:
            # only use transform for odom-base-link
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                transform.header.stamp = self.get_clock().now().to_msg()
                self.lidar_path_pub.publish(transform)
            
        
        
    # Process synchronized data
    def sync_callback(self, cam, lidar, lidar_transformation):
        self.save_image(cam)
        self.save_lidardata(lidar, lidar_transformation)
        
        self.nextDir += 1

    # Save camera image
    def save_image(self, msg):
        # Create new directory for image to be saved in
        self.imgTargetDir = self.cam0Dir + str(self.nextDir).zfill(8) + '/'
        os.makedirs(self.imgTargetDir)

        # write current camera image to specified folder
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        imageFile = self.imgTargetDir + 'image_00000000.png'
        imageMeta = self.imgTargetDir + 'meta_00000000.yaml'

        cv2.imwrite(imageFile, cv_image)

        # Timestamp in nanoseconds
        timestamp = msg.header.stamp.nanosec

        # Create meta png yaml
        metaYamlPng = {
            "timestamp": timestamp, 
            "entity": "sensor_data", 
            "type": "camera_image",
        } # Add information here if needed
        
        with open(self.imgTargetDir + 'meta_png.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlPng, yaml_file, default_flow_style=True)

        # Create yaml for image
        metaYamlImg = {
            "timestamp": timestamp, 
            "entity": "sensor_data", 
            "type": "camera_image", 
        } # Add information here if needed
        
        with open(imageMeta, 'w') as yaml_file:
            yaml.dump(metaYamlImg, yaml_file, default_flow_style=True)

    # Save lidar data
    def save_lidardata(self, msg, transformation):
        # Create new directory for lidar data to be saved in
        self.lidarTargetDir = self.lidar0Dir + str(self.nextDir).zfill(8) + '/'
        if not os.path.exists(self.lidarTargetDir):
            os.makedirs(self.lidarTargetDir)
        
        # Timestamp in nanoseconds
        timestamp = msg.header.stamp.nanosec
        
        metaYamlRaw = {
            'transformation': [transformation.transform.translation.x, transformation.transform.translation.y, transformation.transform.translation.z],
            'rotation': [transformation.transform.rotation.x, transformation.transform.rotation.y, transformation.transform.rotation.z, transformation.transform.rotation.w],
            'start_time': timestamp
        } # Add information here if needed
        
        with open(self.lidarTargetDir + 'meta.yaml', 'w') as yaml_file:
            yaml.dump(metaYamlRaw, yaml_file, default_flow_style=None)
        
        
        # Create pointcloud from incoming data
        points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "reflectivity"], skip_nans=True))
        amount_points = len(points)
        
        header_intensities = f'{{"TYPE": "FLOAT", "SHAPE": [{amount_points},1]}};'
        header_points = f'{{"TYPE": "FLOAT", "SHAPE": [{amount_points},3]}};'
        header_reflectivity = f'{{"TYPE": "INT", "SHAPE": [{amount_points},1]}};'

        intensities_path = os.path.join(self.lidarTargetDir, "intensities.data")
        points_path = os.path.join(self.lidarTargetDir, "points.data")
        reflectivity_path = os.path.join(self.lidarTargetDir, "reflectivity.data")

        # Write data into .data files
        with open(intensities_path, "wb") as data_file_intensities, open(points_path, "wb") as data_file_points, open(reflectivity_path, "wb") as data_file_reflectivity:
            # Add header
            data_file_intensities.write(header_intensities.encode('utf-8'))
            data_file_points.write(header_points.encode('utf-8'))
            data_file_reflectivity.write(header_reflectivity.encode('utf-8'))
            
            for point in points:
                x, y, z, intensity, reflectivity = point
                data_file_intensities.write(struct.pack("f", intensity))
                data_file_points.write(struct.pack("fff", x, y, z))
                data_file_reflectivity.write(struct.pack("i", reflectivity))
            
            
        # Write .yaml files for .data files
        yaml_data =  {
            'entity': 'channel',
            'data_type': 'float',
            'type': 'array',
            'shape': [amount_points, 1],
            'name': 'intensities'
        }
        
        with open(self.lidarTargetDir + 'intensities.yaml', 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)


        yaml_data =  {
            'entity': 'channel',
            'data_type': 'float',
            'type': 'array',
            'shape': [amount_points, 3],
            'name': 'points'
        }
        
        with open(self.lidarTargetDir + 'points.yaml', 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
            
            
        yaml_data =  {
            'entity': 'channel',
            'data_type': 'int',
            'type': 'array',
            'shape': [amount_points, 1],
            'name': 'reflectivity'
        }
        
        with open(self.lidarTargetDir + 'reflectivity.yaml', 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
            
        # Add more .data + their yaml files here if needed
            
            
# Set yaml flow of arrays (for dumping multidimensional arrays)
def represent_list_as_flow(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True) 

def main(args=None):
    # Set yaml flow
    yaml.add_representer(list, represent_list_as_flow)
    
    # start node
    rclpy.init(args=args)
    synchronized_data = SynchronizedDataNode()
    rclpy.spin(synchronized_data)

    # stop node
    synchronized_data.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
