import yaml
import os
from harvesterclass import get_harvester, get_image_acquisition
import numpy as np
import struct

class PhotoneoConfig:
    def __init__(self, logger):

        self.logger = logger
        self.config = self.load_config()
        self.log_config()

        device = self.config['photoneo_camera']['connection']['device_id']
        
        self.harvester = get_harvester(device)

        self.logger.info('Creating Image Acquirer')
        self.logger.info(f'Using device: {device}')


        self.ia = get_image_acquisition(device)
        self.features = self.ia.remote_device.node_map

        self.logger.info("Applying configuration to Photoneo camera...")
        self.apply_config_to_camera()

        self.next_scan_num = self.get_next_scan_number()

    def load_config(self) -> dict:
        """
        Load configuration from YAML file
        Returns:
            dict: Configuration dictionary
        """


        try:
            # Get package path and config file path
            self.logger.info(f'Current working directory: {os.getcwd()}')


            config_path = os.path.join('conf', 'photoneo_config.yaml')

            # If not found in install space, try source space
            if not os.path.exists(config_path):
                raise FileNotFoundError
            self.logger.info(f'Loading config from: {config_path}')
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            return config
            
        except FileNotFoundError:
            self.logger.info(f'Config file not found: {config_path}')
            return self.get_default_config()
        except yaml.YAMLError as e:
            self.logger.info(f'Error parsing YAML: {e}')
            return self.get_default_config()

    def get_default_config(self):

        self.logger.info(f'Fallback - could not load config - using default configuration')

        """Return default configuration if YAML file is not found"""
        return {
            'photoneo_camera': {
                'connection': {
                    'device_id': 'PhotoneoTL_DEV_2019-06-011-LC3',
                },
                'output': {
                    'directory': 'tmp/captured_images',
                    'image_format': 'png',
                    'point_cloud_format': 'ply'
                },
                'capture': {
                    'PhotoneoTriggerMode' : 'Software',
                    'AmbientLightSuppression': True,
                    'CodingStrategy': 'Interreflections',
                },
                'point_cloud_settings': {
                    'SendTexture': True,
                    'SendPointCloud': True,
                    'SendNormalMap': True,
                    'SendDepthMap': True,
                    'SendConfidenceMap': True,
                },
                'processing': {
                    'InterreflectionsFiltering': True,
                    'InterreflectionFilterStrength': 0.5,
                    'MaxInaccuracy': 2.0
                },
                'coordinate_space':{
                    'CoordinateSpace': 'RobotSpace',
                    'TransformationSpaceSelector': 'Robot'
                }
            }
        }


    def log_config(self):
        """Log loaded configuration"""
        camera_config = self.config['photoneo_camera']
        for section, params in camera_config.items():
            if isinstance(params, dict):
                self.logger.info(f"{section}:")
                for key, value in params.items():
                    self.logger.info(f"  {key}: {value}")
            else:
                self.logger.info(f"{section}: {params}")



    def reload_config(self):
        """Reload configuration from YAML file"""
        self.config = self.load_config()
        self.logger.info('Configuration reloaded')
        self.log_config()
        self.logger.info("Applying configuration to Photoneo camera...")
        self.apply_config_to_camera()


    def apply_config_to_camera(self):
        
        """Apply configuration settings to the Photoneo camera"""
        camera_config = self.config['photoneo_camera']
        try:

            if hasattr(self.features, 'PhotoneoTriggerMode'):
                self.features.PhotoneoTriggerMode.value = camera_config['capture']['PhotoneoTriggerMode']
            else:
                self.logger.info('Feature PhotoneoTriggerMode not available on this device.')

            if hasattr(self.features, 'AmbientLightSuppression'):
                self.features.AmbientLightSuppression.value = camera_config['capture']['AmbientLightSuppression']
            else:
                self.logger.info('Feature AmbientLightSuppression not available on this device.')

            if hasattr(self.features, 'CodingStrategy'):
                self.features.CodingStrategy.value = camera_config['capture']['CodingStrategy']
            else:
                self.logger.info('Feature CodingStrategy not available on this device.')

            if hasattr(self.features, 'SendTexture'):
                self.features.SendTexture.value = camera_config['point_cloud_settings']['SendTexture']
            else:
                self.logger.info('Feature SendTexture not available on this device.')

            if hasattr(self.features, 'SendPointCloud'):
                self.features.SendPointCloud.value = camera_config['point_cloud_settings']['SendPointCloud']
            else:
                self.logger.info('Feature SendPointCloud not available on this device.')

            if hasattr(self.features, 'SendNormalMap'):
                self.features.SendNormalMap.value = camera_config['point_cloud_settings']['SendNormalMap']
            else:
                self.logger.info('Feature SendNormalMap not available on this device.')

            if hasattr(self.features, 'SendDepthMap'):
                self.features.SendDepthMap.value = camera_config['point_cloud_settings']['SendDepthMap']
            else:
                self.logger.info('Feature SendDepthMap not available on this device.')

            if hasattr(self.features, 'SendConfidenceMap'):
                self.features.SendConfidenceMap.value = camera_config['point_cloud_settings']['SendConfidenceMap']
            else:
                self.logger.info('Feature SendConfidenceMap not available on this device.')

            if hasattr(self.features, 'InterreflectionsFiltering'):
                self.features.InterreflectionsFiltering.value = camera_config['processing']['InterreflectionsFiltering']
            else:
                self.logger.info('Feature InterreflectionsFiltering not available on this device.')

            if hasattr(self.features, 'InterreflectionFilterStrength'):
                self.features.InterreflectionFilterStrength.value = camera_config['processing']['InterreflectionFilterStrength']
            else:
                self.logger.info('Feature InterreflectionFilterStrength not available on this device.')

            if hasattr(self.features, 'MaxInaccuracy'):
                self.features.MaxInaccuracy.value = camera_config['processing']['MaxInaccuracy']
            else:
                self.logger.info('Feature MaxInaccuracy not available on this device.')

            if hasattr(self.features, 'CoordinateSpace'):
                self.features.CoordinateSpace.value = camera_config['coordinate_space']['CoordinateSpace']
            else:
                self.logger.info('Feature CoordinateSpace not available on this device.')

            if hasattr(self.features, 'TransformationSpaceSelector'):
                self.features.TransformationSpaceSelector.value = camera_config['coordinate_space']['TransformationSpaceSelector']
            else:
                self.logger.info('Feature TransformationSpaceSelector not available on this device.')

            self.logger.info('Configuration applied to Photoneo camera')
        except Exception as e:
            self.logger.error(f'Failed to apply configuration to camera: {e}')

    def get_camera_param(self, param: str):

        if hasattr(self.features, param):
            attr_name = param
            return getattr(self.features, attr_name).value
        else:
            self.logger.info(f'Camera parameter {param} not available')
            return None

    def get_next_scan_number(self):
        """
        Returns last scan number + 1 based on existing scan files.
        Can be used to determine the next available scan number.
        """

        # Find the last scan file based on the scan number
        scan_dir = self.config['photoneo_camera']['output']['directory']
        point_cloud_format = "ply"
        scan_num = 0
        while os.path.exists(os.path.join(scan_dir, f"scan_{scan_num}.{point_cloud_format}")):
            scan_num += 1

        return scan_num

    def set_robot_transform(self, transform: np.ndarray):
        """
        Sets the robot transformation matrix (4x4) ndarray to photoneo

        """

        if not isinstance(transform, np.ndarray):
            self.logger.error('Invalid transformation matrix')
            return

        if transform.shape != (4, 4):
            self.logger.error('Transformation matrix must be 4x4')
            return

        self.robot_transformation = transform
        self.logger.info(f'Robot transformation matrix set:\n{transform}')

        rot_mat = transform[:3, :3]
        trans_vect = transform[:3, 3].tolist()
        rot_vect = rot_mat.reshape(1, 9).tolist()

        try:
            robot_transformation_rotation_matrix_new_values = rot_vect[0]
            robot_transformation_rotation_matrix_length = self.features.RobotTransformationRotationMatrix.length
            robot_transformation_rotation_matrix_bytes = self.features.RobotTransformationRotationMatrix.get(robot_transformation_rotation_matrix_length)
            robot_transformation_rotation_matrix = struct.unpack('9d', robot_transformation_rotation_matrix_bytes)
            robot_transformation_rotation_matrix_new_bytes = struct.pack('9d', *robot_transformation_rotation_matrix_new_values)

            robot_transformation_translation_vector_new_values = trans_vect
            robot_transformation_translation_vector_length = self.features.RobotTransformationTranslationVector.length
            robot_transformation_translation_vector_bytes = self.features.RobotTransformationTranslationVector.get(robot_transformation_translation_vector_length)
            robot_transformation_translation_vector = struct.unpack('3d', robot_transformation_translation_vector_bytes)
            robot_transformation_translation_vector_new_bytes = struct.pack('3d', *robot_transformation_translation_vector_new_values)
            
            self.features.RobotTransformationTranslationVector.set(robot_transformation_translation_vector_new_bytes)
            self.features.RobotTransformationRotationMatrix.set(robot_transformation_rotation_matrix_new_bytes)
            self.logger.info('Robot transformation matrices updated successfully')
            return True

        except Exception as e:
            self.logger.error(f'Failed to update robot transformation matrices: {e}')
