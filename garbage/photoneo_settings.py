import os
import sys
from sys import platform
import struct
from utils.harvester_singleton import get_harvester, get_image_acquisition

harvester = get_harvester()
    
with get_image_acquisition() as ia:
    features = ia.remote_device.node_map
    print(id(ia))
    attr_name = "PhotoneoDeviceID"
    print(getattr(features, attr_name).value)
    print(features.IsPhoXiControlRunning.value)
    while True:
        pass
