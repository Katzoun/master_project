from interface_pkg.srv import CaptureImage
import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import struct
import numpy as np

# print("Current Working Directory:", os.getcwd())
# print(os.path.dirname(__file__))
sys.path.append(os.path.dirname(__file__))
from configclass import PhotoneoConfig
from harvesterclass import get_harvester, get_image_acquisition

class PhotoneoNode(Node):

    def __init__(self):
        super().__init__('photoneo_node')

        # Load configuration from YAML file
        self.photoneo_config = PhotoneoConfig(self.get_logger())
        # Log loaded configuration
        self.photoneo_config.log_config()
        self.harvester = get_harvester()
        self.ia = get_image_acquisition()

            
        #pridat clean up jako je v RWS provideru, udelat nastaveni v yaml
        # realne podle toho co potrebuju a pridat callback na topic pro update configu

        self.srv = self.create_service(CaptureImage, 'capture_image', self.capture_image_callback)
        self.get_logger().info('Photoneo capture service ready')

    

    def capture_image_callback(self, request, response):
        self.get_logger().info('Incoming capture request:')
        self.get_logger().info(f'Current robot pose: {request.robot_pose}')
        self.get_logger().info(f'Camera settings: {request.camera_settings}')
        
        # Access configuration values
        camera_config = self.photoneo_config.config['photoneo_camera']

        try:
            # Use configuration for camera setup
            ip = camera_config['connection']['ip']
            port = camera_config['connection']['port']
            exposure = camera_config['capture']['exposure_time']
            gain = camera_config['capture']['gain']
            output_dir = camera_config['output']['directory']
            image_format = camera_config['output']['image_format']
            
            self.get_logger().info(f'Connecting to camera at {ip}:{port}')
            self.get_logger().info(f'Using exposure: {exposure}, gain: {gain}')
            
            # Create output directory if it doesn't exist
            os.makedirs(output_dir, exist_ok=True)
            
            # Generate filename
            filename = f"image_{request.robot_pose.position.x}_{request.robot_pose.position.y}.{image_format}"
            file_path = os.path.join(output_dir, filename)
            
            self.get_logger().info(f'Capturing image to: {file_path}')
            
            # TODO: Add actual Photoneo camera capture logic here
            # camera = PhotoneoCamera(ip, port)
            # camera.set_exposure(exposure)
            # camera.set_gain(gain)
            # image = camera.capture()
            # image.save(file_path)
            
            response.success = True
            response.file_path = file_path
            
        except Exception as e:
            response.success = False
            response.file_path = f"Failed to capture image: {str(e)}"
            
        return response

def main(args=None):
    rclpy.init(args=args)
    photoneo_node = PhotoneoNode()
    rclpy.spin(photoneo_node)
    photoneo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()