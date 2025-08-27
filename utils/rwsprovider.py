import requests
from requests.auth import HTTPBasicAuth
from .exceptions import RWSException
import threading
import time
from datetime import datetime, timedelta


requests.packages.urllib3.disable_warnings(
    requests.packages.urllib3.exceptions.InsecureRequestWarning
)

class RWSClient:

    def __init__(self, host, username, password, port=80, keepalive=True, keepalive_interval=180):
        proto = "https"
        self.base_url = f"{proto}://{host}:{port}"
        self.session = requests.Session()

        self.session.verify = False
        self.auth_method = HTTPBasicAuth(username, password)
        self.header_typ = {'Accept': 'application/hal+json;v=2.0', 
                       'Content-Type': 'application/x-www-form-urlencoded;v=2.0'}
        self.header_opt = {'Accept': 'application/xhtml+xml;v=2.0'}
        
        # Keepalive setup
        if keepalive:
            self.keepalive_interval = keepalive_interval  # seconds (3 min default)
            self.keepalive_thread: threading.Thread = None
            self.keepalive_running = False
            self.last_activity = datetime.now()
            self._lock = threading.Lock()



    def login(self):
        url = f"{self.base_url}"
        resp = self.session.get(url, headers=self.header_typ, auth=self.auth_method)
        print(f"Login status code: {resp.status_code}")
        if 'ABBCX' not in self.session.cookies.get_dict():
            raise RWSException("Login failed: missing ABBCX cookie")
        
        self.start_keepalive()
        return resp.status_code
    
    def logout(self):

        self.stop_keepalive()

        url = f"{self.base_url}/logout"
        resp = self.session.get(url, headers=self.header_typ)
        print(f"Logout status code: {resp.status_code}")
        
        if resp.status_code != 204:
            raise RWSException(f"Logout failed: {resp.status_code}")
        
        self.session.close()
        return resp.status_code
    
    def start_keepalive(self):
        """Starts a background thread to keep the connection alive"""
        if self.keepalive_thread is None or not self.keepalive_thread.is_alive():
            self.keepalive_running = True
            self.keepalive_thread = threading.Thread(target=self._keepalive_worker, daemon=True)
            self.keepalive_thread.start()
            print(f"Keepalive thread started (interval: {self.keepalive_interval}s)")
    
    def stop_keepalive(self):
        """Stops the keepalive thread"""
        self.keepalive_running = False
        if self.keepalive_thread and self.keepalive_thread.is_alive():
            self.keepalive_thread.join(timeout=2)
            print("Keepalive thread stopped")

    def _keepalive_worker(self):
        """Background worker for keepalive"""
        while self.keepalive_running:
            try:
                time.sleep(10)  # Check every 10 seconds
                with self._lock:
                    time_since_activity = datetime.now() - self.last_activity
                    
                if time_since_activity.total_seconds() >= self.keepalive_interval:
                    try:
                        self._send_heartbeat()
                        print(f"Heartbeat sent at {datetime.now().strftime('%H:%M:%S')}")
                    except Exception as e:
                        print(f"Heartbeat failed: {e}")
                        # Try to re-login if heartbeat fails
                        try:
                            self._reconnect()
                        except Exception as reconnect_error:
                            print(f"Reconnection failed: {reconnect_error}")
                            
            except Exception as e:
                print(f"Keepalive worker error: {e}")
                time.sleep(10)  # Wait before retry

    def _send_heartbeat(self):
        """Sends a lightweight request to keep the connection alive"""
        # Lightweight GET request - just check controller state
        url = f"{self.base_url}/rw/system"
        resp = self.session.get(url, headers=self.header_typ, timeout=10)
        
        if resp.status_code == 200:
            with self._lock:
                self.last_activity = datetime.now()
        else:
            raise RWSException(f"Heartbeat failed: {resp.status_code}")
    
    def _reconnect(self):
        """Attempts to reconnect"""
        print("Attempting reconnection...")
        self.session.close()
        self.session = requests.Session()
        self.session.verify = False
        
        # Re-login
        url = f"{self.base_url}"
        resp = self.session.get(url, headers=self.header_typ, auth=self.auth_method)
        
        if resp.status_code == 200 and 'ABBCX' in self.session.cookies.get_dict():
            with self._lock:
                self.last_activity = datetime.now()
            print("Reconnection successful")
        else:
            raise RWSException("Reconnection failed")
    
    def _update_activity(self):
        """Updates the last activity time"""
        with self._lock:
            self.last_activity = datetime.now()
    

    def is_logged_in(self):
        url = f"{self.base_url}"
        resp = self.session.get(url, headers=self.header_typ)
        print(f"Login status code: {resp.status_code}")
        if resp.status_code == 200:
            return True
        return False

    def get_request(self, path):
        url = f"{self.base_url}{path}"
        resp = self.session.get(url, headers=self.header_typ)
        #print(f"GET status code: {resp.status_code}")
        #print(f"Cookies: {self.session.cookies.get_dict()}")
        self._update_activity()
        
        if resp.status_code != 200:
            raise RWSException(f"GET {path} failed: {resp.status_code}")
        return (resp.json() if resp.content else None, resp.status_code)

    def post_request(self, path, dataIn=None):
        url = f"{self.base_url}{path}"
        resp = self.session.post(url, headers=self.header_typ, data=dataIn)

        self._update_activity()
        #print(resp.text)
        #print(f"POST status code: {resp.status_code}")
        if resp.status_code not in (200, 201, 204):
            raise RWSException(f"POST {path} failed: {resp.status_code}")
        return resp.status_code
    
    def dipc_post_request(self, path, dataIn=None):
        url = f"{self.base_url}{path}"
        resp = self.session.post(url, headers=self.header_typ, data=dataIn)
        #print(resp.text)
        self._update_activity()
        #print(f"POST status code: {resp.status_code}")
        if resp.status_code not in (204,500):
            
            raise RWSException(f"POST {path} failed: {resp.status_code}")
        return resp.status_code

    def options_request(self, path):
        url = f"{self.base_url}{path}"

        self._update_activity()
        
        resp = self.session.options(url, headers=self.header_opt)
        #print(f"OPTIONS status code: {resp.status_code}")
        if resp.status_code not in (200, 201, 204):
            raise RWSException(f"OPTIONS {path} failed: {resp.status_code}")
        
        return (resp.json() if resp.content else None, resp.status_code)
    
    