import requests
from requests.auth import HTTPBasicAuth
import sys
import os
import threading
import time
from datetime import datetime, timedelta
import atexit
sys.path.append(os.path.dirname(__file__))
from exceptions import RWSException
requests.packages.urllib3.disable_warnings(
    requests.packages.urllib3.exceptions.InsecureRequestWarning
)

class RWSClient:

    def __init__(self, host: str, username: str, password: str, port=80, logger=None):
        proto = "https"
        self.base_url = f"{proto}://{host}:{port}"
        self.session = requests.Session()
        self._logged_in = False

        if logger is None:
            class DefaultLogger:
                @staticmethod
                def info(msg):
                    print(msg)

                @staticmethod
                def error(msg):
                    print(f"ERROR: {msg}")

            self.logger = DefaultLogger()
        else:
            self.logger = logger

        self.session.verify = False
        self.auth_method = HTTPBasicAuth(username, password)
        self.header_typ = {'Accept': 'application/hal+json;v=2.0', 
                       'Content-Type': 'application/x-www-form-urlencoded;v=2.0'}
        self.header_opt = {'Accept': 'application/xhtml+xml;v=2.0'}


    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.logger.info(f"RWSClient.__exit__ called - Exception: {exc_type is not None}")
        
        # Cleanup resources
        self.logger.info("RWS Auto-logout triggered...")
        self.logout()

        # Log exception if any
        if exc_type is not None:
            self.logger.error(f"Exception in RWSClient: {exc_type.__name__}: {exc_val}")
            
        return False  # Propagate exception


    def login(self):
        """
        Attempts to log in to the ABB RWS server using the provided session and authentication method.
        This function performs the following steps:
        1. Sends a GET request to the ABB RWS server using the configured session, headers, and authentication method.
        2. Checks for the presence of the 'ABBCX' cookie in the session to verify successful login.
        3. Updates the internal login state based on the response and logs the result.
        Returns:
            bool: True if login is successful (status code 200 and 'ABBCX' cookie present), False otherwise.
        Notes:
            - Uses the requests.Session object for HTTP communication.
            - Logs detailed information about the login attempt and errors.
            - The login state is stored internally in the _logged_in attribute.
            - If the connection fails or the required cookie is missing, login is considered unsuccessful.
        """

        self._logged_in = False
        url = f"{self.base_url}"
        try:
            resp = self.session.get(url, headers=self.header_typ, auth=self.auth_method)

            if 'ABBCX' not in self.session.cookies.get_dict():
                self.logger.error("Login failed: missing ABBCX cookie")
                return False

            if resp.status_code == 200:
                self._logged_in = True
                self.logger.info(f"Login successful, status code: {resp.status_code}")
                return True
            else:
                self.logger.info(f"Login failed, status code: {resp.status_code}")
                return False
            
        except Exception as e:
            self.logger.error(f"Login request failed, check connection: {e}")
            return False

    
    def logout(self):
        """
        Logs out the current session from the RWS (Robot Web Services) server.
        Sends a GET request to the logout endpoint. If the logout is successful (HTTP 204),
        logs a success message. If the logout fails (e.g., already logged out), logs a failure message.
        Raises an RWSException if the logout was not successful.
        Closes the session after attempting logout.
        Returns:
            int: The HTTP status code returned by the logout request.
        Raises:
            RWSException: If the logout request does not return a 204 status code.
        """
        url = f"{self.base_url}/logout"
        self._logged_in = False
        try:
            # Check if session is already closed
            if not self.session.adapters:
                self.logger.info("Session already closed.")
                return False

            resp = self.session.get(url, headers=self.header_typ)
            self.session.close()

            if resp.status_code == 204:
                self.logger.info(f"Logout successful, status code: {resp.status_code}")
                return True
            else:
                self.logger.info(f"Logout failed (probably already logged out), status code: {resp.status_code}")
                return False
            
        except Exception as e:
            self.logger.error(f"Logout request failed, message: {e}")
            return False
        
    def get_login_state(self):
        """
        Checks the login state by sending a GET request to the base URL.
        Returns:
            bool: True if the login request is successful (HTTP status code 200), False otherwise.
        Logs:
            Error message if the request fails.
        """
        

        url = f"{self.base_url}"
        try:
            resp = self.session.get(url, headers=self.header_typ)
            if resp.status_code == 200:
                return True
        except Exception as e:
            self.logger.error(f"Login state request failed, message: {e}")
        return False
        


    def send_keepalive(self):
        """
        Sends a lightweight GET request to the controller to keep the connection alive.
        This method checks the controller state by sending a GET request to the `/rw/system` endpoint.
        If the request is successful (HTTP status code 200), it logs a success message, sets the
        `_logged_in` flag to True, and returns True. If the request fails or an exception occurs,
        it logs an error message, sets the `_logged_in` flag to False, and returns False.
        Returns:
            bool: True if the keepalive request was successful, False otherwise.
        Logs:
            Success or error messages based on the outcome of the request.
        """

        # Lightweight GET request - just check controller state
        url = f"{self.base_url}/rw/system"
        try:
            resp = self.session.get(url, headers=self.header_typ, timeout=10)
            if resp.status_code == 200:
                self.logger.info("Keepalive successful")
                self._logged_in = True
                return True
            else:
                self.logger.error(f"Keepalive failed, status code: {resp.status_code}")
                self._logged_in = False
                return False
            
        except Exception as e:
            self.logger.error(f"Keepalive request failed, message: {e}")
            self._logged_in = False
            return False

    
    def get_request(self, path):
        """
        Sends a GET request to the specified path using the configured session and headers.
        Args:
            path (str): The endpoint path to append to the base URL for the GET request.
        Returns:
            tuple: A tuple containing:
                - dict or None: The JSON response from the server if available, otherwise None.
                - int: The HTTP status code of the response, or -1 if an exception occurred.
        Logs:
            Errors are logged if the request fails or if the response status code is not 200.
        """
        
        url = f"{self.base_url}{path}"
        try:
            resp = self.session.get(url, headers=self.header_typ)
            if resp.status_code != 200:
                self.logger.error(f"GET {path} failed: {resp.status_code}")
            
            return (resp.json() if resp.content else None, resp.status_code)
        except Exception as e:
            self.logger.error(f"GET request {path} failed: {e}")
            return (None, int(-1))

    def post_request(self, path, dataIn=None):
        """
        Sends a POST request to the specified path using the configured session and headers.
        Args:
            path (str): The endpoint path to append to the base URL for the POST request.
            dataIn (dict, optional): The JSON payload to include in the POST request.
        Returns:
            int: The HTTP status code of the response, or -1 if an exception occurred.
        Logs:
            Errors are logged if the request fails or if the response status code is not in (200, 201, 204, 500).
        """

        url = f"{self.base_url}{path}"
        try:
            resp = self.session.post(url, headers=self.header_typ, data=dataIn)
            if resp.status_code not in (200, 201, 204, 500): # 500 is returned by some DIPC calls
                self.logger.error(f"POST {path} failed: {resp.status_code}")
            return resp.status_code
        
        except Exception as e:
            self.logger.error(f"POST request {path} failed: {e}")
            return int(-1)
    
    # def dipc_post_request(self, path, dataIn=None):
    #     url = f"{self.base_url}{path}"
    #     try:
    #         resp = self.session.post(url, headers=self.header_typ, data=dataIn)
    #         if resp.status_code not in (204, 500):
    #             self.logger.error(f"DIPC POST {path} failed: {resp.status_code}")
    #         return resp.status_code

    #     except Exception as e:
    #         self.logger.error(f"DIPC POST request {path} failed: {e}")
    #         return int(-1)


    def options_request(self, path):
        """
        Sends an OPTIONS request to the specified path using the configured session and headers.
        Args:
            path (str): The endpoint path to append to the base URL for the OPTIONS request.
        Returns:
            tuple: A tuple containing:
                - dict or None: The JSON response from the server if available, otherwise None.
                - int: The HTTP status code of the response, or -1 if an exception occurred.
        Logs:
            Errors are logged if the request fails or if the response status code is not in (200, 201, 204).
        """

        url = f"{self.base_url}{path}"

        try:
            resp = self.session.options(url, headers=self.header_opt)
            if resp.status_code not in (200, 201, 204):
                self.logger.error(f"OPTIONS {path} failed: {resp.status_code}")
            return (resp.json() if resp.content else None, resp.status_code)
    

        except Exception as e:
            self.logger.error(f"OPTIONS request {path} failed: {e}")
            return (None, int(-1))

    
    