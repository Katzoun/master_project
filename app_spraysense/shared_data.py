from __future__ import annotations
import threading
import os
import sys
from datetime import datetime
import logging
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))



class SharedData:
    """Thread-safe singleton for sharing data between threads"""
    _instance = None
    _lock = threading.Lock()  # Class-level lock for singleton

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:  # Double-checked locking pattern
                if cls._instance is None:
                    cls._instance = super(SharedData, cls).__new__(cls)
                    cls._instance._init()
        return cls._instance

    def _init(self):

        # robot configuration
        self.robot_ip = "192.168.0.37"
        self.robot_username = "Admin"
        self.robot_password = "robotics"
        self.robot_port = 443  # real robot 443
        self.robot_comm_keepalive = True
        self.robot_comm_keepalive_interval = 180  # seconds

        self.calibration_matrix_file = "conf/calib_mat.txt"

        try:
            self.robot_hand_eye_matrix = np.loadtxt(self.calibration_matrix_file)
        except Exception as e:
            self.robot_hand_eye_matrix = None
            print(f"Warning: Failed to load '{self.calibration_matrix_file}': {e}")
        

        # create directories
        self.log_directory = "logs"
        self.scan_directory = "scans"
        self.path_directory = "paths"

        self.create_dirs()

        #create logger
        self.logger = self.create_logger()

    def create_dirs(self) -> None:
        """Creates directories for logs, scans, paths ... if they don't exist."""
        for dir_path in [self.log_directory, self.scan_directory, self.path_directory]:
            os.makedirs(dir_path, exist_ok=True)


    def create_logger(self, name: str = "SharedLogger") -> logging.Logger:
        """
        Create and configure a shared logger.
        Args:
            name (str): The name of the logger.
        Returns Configured logger instance.
        """

        current_date = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        log_filename = os.path.join(self.log_directory, f"log_{current_date}.log")

        logger = logging.getLogger(name)

        if logger.hasHandlers():
            return logger

        logger.setLevel(logging.DEBUG)

        file_handler = logging.FileHandler(log_filename, encoding="utf-8")
        file_handler.setLevel(logging.INFO)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)

        # Log message format
        formatter = logging.Formatter(
            "[%(levelname)s] %(asctime)s | %(name)s | %(message)s",
            datefmt="%d-%m-%Y %H:%M:%S"
        )

        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Attach handlers to the logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger

    def get_logger(self) -> logging.Logger:
        """
        Returns the shared logger instance.
        Returns:
            logging.Logger: The shared logger instance.
        """
        return self.logger

    @classmethod
    def get_instance(cls) -> SharedData:
        """
        Returns:
            SharedData: The singleton instance of SharedData    
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
