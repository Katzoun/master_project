import yaml
import os

class PhotoneoConfig:
    def __init__(self, logger):

        self.logger = logger
        self.config = self.load_config()




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
        """Return default configuration if YAML file is not found"""
        return {
            'photoneo_camera': {
                'connection': {
                    'ip': '192.168.1.100',
                    'port': 11003,
                    'timeout': 5.0
                },
                'capture': {
                    'exposure_time': 100,
                    'gain': 50,
                    'brightness': 0,
                    'contrast': 0
                },
                'point_cloud': {
                    'enable': True,
                    'quality': 'high'
                },
                'output': {
                    'directory': '/tmp/captured_images',
                    'image_format': 'png',
                    'point_cloud_format': 'ply'
                },
                'processing': {
                    'apply_filters': True,
                    'min_distance': 0.1,
                    'max_distance': 5.0
                }
            }
        }

    def log_config(self):
        """Log loaded configuration"""
        camera_config = self.config['photoneo_camera']
        self.logger.info('=== Photoneo camera configuration ===')
        self.logger.info(f"IP: {camera_config['connection']['ip']}")
        self.logger.info(f"Port: {camera_config['connection']['port']}")
        self.logger.info(f"Exposure: {camera_config['capture']['exposure_time']}")
        self.logger.info(f"Gain: {camera_config['capture']['gain']}")
        self.logger.info(f"Output Dir: {camera_config['output']['directory']}")
        self.logger.info(f"Point Cloud: {camera_config['point_cloud']['enable']}")

    def reload_config(self):
        """Reload configuration from YAML file"""
        self.config = self.load_config()
        self.logger.info('Configuration reloaded')
        self.log_config()

