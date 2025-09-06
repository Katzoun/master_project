from interface_pkg.srv import CaptureImage, ReloadConfig
import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import numpy as np

# print("Current Working Directory:", os.getcwd())
# print(os.path.dirname(__file__))
sys.path.append(os.path.dirname(__file__))
from configclass import PhotoneoConfig

class PhotoneoNode(Node):

    def __init__(self):
        super().__init__('photoneo_node')


        # Load configuration from YAML file
        self.photoneo = PhotoneoConfig(self.get_logger())

        self.photoneo.get_camera_param("PhotoneoDeviceID")

        self.capture_service = self.create_service(CaptureImage, 'capture_image', self.capture_image_callback)
        self.reload_service = self.create_service(ReloadConfig, 'reload_config', self.reload_config_callback)
        self.get_logger().info('Photoneo node ready')
    

    def capture_image_callback(self, request, response):
        self.get_logger().info('Incoming capture request')
        transform = np.array(request.transform).reshape((4, 4))

        
        scan_dir = self.photoneo.config['photoneo_camera']['output']['directory']
        scan_dir = os.path.join(os.getcwd(), scan_dir)  # Make path absolute

        point_cloud_format = self.photoneo.config['photoneo_camera']['output']['point_cloud_format']
        
        self.photoneo.set_robot_transform(transform)

        self.photoneo.ia.start()
        # Trigger frame by calling property's setter.
        # Must call TriggerFrame before every fetch.
        self.photoneo.features.TriggerFrame.execute()
        
        try:
            buffer = self.photoneo.ia.fetch(timeout=10.0)
            self.get_logger().info(str(buffer))

            os.makedirs(scan_dir, exist_ok=True)
            scan_path = os.path.join(scan_dir, f"scan_{self.photoneo.next_scan_num}.{point_cloud_format}")

            self.photoneo.features.SaveLastScanFilePath.value = scan_path
            self.photoneo.features.SaveLastScanFrameId.value= -1

            json_options = '{"Mesh": true, "DepthMap": true, "PointCloud": true, "NormalMap": true}'
            self.photoneo.features.SaveLastScanJsonOptions.value = json_options
            self.photoneo.features.SaveLastScan.execute()

            self.get_logger().info(f"NEW SCAN SAVED TO --> scan_path: {scan_path}")

            self.photoneo.next_scan_num += 1

            # self.get_logger().info(f"Current Working Directory: {os.getcwd()}")
            # self.get_logger().info(os.path.dirname(__file__))
       
            response.success = True
            response.file_path = scan_path
            
        except Exception as e:
            response.success = False
            response.file_path = "ERROR"
            self.get_logger().error(f'Error during image capture: {e}')

        return response

    def reload_config_callback(self, request, response):
        try:
            self.get_logger().info('Incoming reload config request')
            self.photoneo.config = self.photoneo.load_config()
            self.photoneo.log_config()
            self.photoneo.apply_config_to_camera()
        except Exception as e:
            self.get_logger().error(f'Error reloading config: {e}')
            response.success = False
            return response

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    try:
        photoneo_node = PhotoneoNode()
        rclpy.spin(photoneo_node)
    except Exception as e:
        photoneo_node.get_logger().error(f'Exception occurred: {e}')
    except KeyboardInterrupt:
        photoneo_node.get_logger().error(f'KeyboardInterrupt occurred')
    finally:
        photoneo_node.get_logger().info('Cleaning up resources...')
        photoneo_node.harvester.reset()
        photoneo_node.ia.destroy()

        photoneo_node.get_logger().info('Shutting down Photoneo Node...')
        photoneo_node.destroy_node()
        rclpy.shutdown()





if __name__ == '__main__':
    main()