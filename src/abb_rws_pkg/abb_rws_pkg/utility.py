
import os
import yaml

class Utilities:

    def __init__(self, logger):

        if logger is None:
            class DefaultLogger:
                @staticmethod
                def info(msg):
                    print(msg)
            self.logger = DefaultLogger()
        else:
            self.logger = logger

    def load_config(self, config_path: str) -> dict:
        """
        Load configuration from YAML file
        Returns:
            dict: Configuration dictionary
        """


        try:
            # Get package path and config file path
            self.logger.info(f'Current working directory: {os.getcwd()}')

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
            'robot_config': {
                'connection': {
                    'ip_address': '192.168.0.37',
                    'port': 443,
                    'username': 'Admin',
                    'password': 'robotics',
                    'keepalive': True,
                    'keepalive_interval': 60  # seconds
                },
                'calibration': {
                    'calibration_matrix_path': 'conf/calib_mat.txt'
                }
            }
        }

    def log_config(self, config: dict):
        """Log loaded configuration"""
        
        for section, params in config.items():
            if isinstance(params, dict):
                self.logger.info(f"{section}:")
                for key, value in params.items():
                    self.logger.info(f"  {key}: {value}")
            else:
                self.logger.info(f"{section}: {params}")
