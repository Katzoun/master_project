import os
import sys
from sys import platform
from harvesters.core import Harvester

class HarvesterSingleton:
    _instance = None
    _harvester = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(HarvesterSingleton, cls).__new__(cls)
        return cls._instance
    
    def get_harvester(self, device_id=None):
        if self._harvester is None:
            self._initialize_harvester(device_id)
        return self._harvester
    
    def _initialize_harvester(self, device_id=None):
        # Set default device_id if not provided
        if device_id is None:
            device_id = "PhotoneoTL_DEV_2019-06-011-LC3"
            if len(sys.argv) == 2:
                device_id = "PhotoneoTL_DEV_" + sys.argv[1]
        
        print("--> device_id: ", device_id)
        
        # Setup CTI file path
        if platform == "win32":
            cti_file_path_suffix = "/API/bin/photoneo.cti"
        else:
            cti_file_path_suffix = "/API/lib/photoneo.cti"
        cti_file_path = os.getenv('PHOXI_CONTROL_PATH') + cti_file_path_suffix
        print("--> cti_file_path: ", cti_file_path)
        
        # Initialize harvester
        self._harvester = Harvester()
        self._harvester.add_file(cti_file_path, True, True)
        self._harvester.update()
        
        # Print available devices
        print()
        print("Name : ID")
        print("---------")
        for item in self._harvester.device_info_list:
            print(item.property_dict['serial_number'], ' : ', item.property_dict['id_'])
        print()
    
    def create_image_acquisition(self, device_id=None):
        """Create image acquisition object"""
        harvester = self.get_harvester(device_id)
        if device_id is None:
            device_id = "PhotoneoTL_DEV_2019-06-011-LC3"
        return harvester.create({'id_': device_id})
    
    def cleanup(self):
        """Cleanup resources"""
        if self._harvester is not None:
            self._harvester.reset()
            self._harvester = None

# Create singleton instance
harvester_singleton = HarvesterSingleton()

# Convenience function for easy access
def get_harvester(device_id=None):
    return harvester_singleton.get_harvester(device_id)

def get_image_acquisition(device_id=None):
    return harvester_singleton.create_image_acquisition(device_id)