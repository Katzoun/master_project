# rws_wrappers.py
from exceptions import RWSException
import numpy as np
import json
from rwsproviderV2 import RWSClient
import rwsutilities

class RWSInterface(RWSClient):
    """
    RWSInterface provides a high-level Python interface for interacting with ABB Robot Web Services (RWS).
    
    It extends RWSClient and wraps HTTP requests to the robot controller, exposing convenient methods for
    getting and setting robot state, RAPID variables, IO signals, and DIPC queues.
    
    Main Features:
        - Helper methods for generic GET/POST requests and JSON formatting.
        - GET methods for retrieving robot controller state, RAPID execution state, IO signals, mechanical unit positions,
        RAPID symbols, DIPC queue information, and more. Returns either string, JSON, or Python objects.
        - POST methods for controlling robot state (motors on/off, restart controller, reset program pointer, start/stop RAPID),
        requesting/releasing mastership, setting IO signals, updating RAPID symbols, and sending DIPC messages.
        - Utility methods for converting between RAPID data types and Python types (int, float, bool, list, string).
    Args:
        host (str): Robot controller hostname or IP address.
        username (str): Username for authentication.
        password (str): Password for authentication.
        port (int, optional): HTTP port (default: 80).
        logger (optional): Logger instance for debug output.
    
    
    Usage Example:
        ```python
        rws = RWSInterface(host="192.168.125.1", username="Admin", password="robotics")
        rws.login()
        clock, status = rws.get_clock()
        value, status = rws.get_rapid_symbol_int("counter", "MainModule")
    ```
    """


    def __init__(self, host: str, username: str, password: str, port: int = 80, logger=None):

        super().__init__(host, username, password, port=port, logger=logger)

    ###################################
    ## HELPER METHODS
    ###################################
    
    def get_generic(self, endpoint: str, feature: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
        data = data['state'][0]
        return (data.get(feature, 'unknown').lower(), status)
    
    def get_embed_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
        data = data.get('_embedded', {}).get('resources', [])
        data_json = json.dumps(data, indent=2)
        return (data_json, status)
    
    def get_generic_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
        data = data['state']
        data_json = json.dumps(data, indent=2)
        return (data_json, status)
    

    ###################################
    ## GET METHODS - string or JSON return types
    ###################################


    def get_clock(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: A string representing the current clock value from the robot controller.
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/ctrl/clock", "datetime")

    def get_controller_state(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The state of the controller (motors on/off, guardstop, emergencystop, init).
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/rw/panel/ctrl-state", "ctrlstate")
        
    def get_opmode_state(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The current operational mode of the robot (auto, man, manf).
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/rw/panel/opmode", "opmode")

    def get_safety_mode(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The current safety mode of the robot.
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/ctrl/safety/mode", "safetymode")

    def get_speedratio(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The current speed ratio of the robot.
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/rw/panel/speedratio", "speedratio")

    def get_robot_type(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The type of the robot (e.g., CRB 15000-10/1.52).
            - int: The HTTP status code of the response.
        """
        return self.get_generic("/rw/system/robottype", "robot-type")
    
    def get_network_info(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The network information of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/ctrl/network")

    def get_system_options(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The system options of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/system/options")
    
    def get_system_products(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The system products of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/system/products")
    
    def get_energy_info(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The energy information of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/system/energy")
    
    def get_mechunits(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The mechanical unit information of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/motionsystem/mechunits")
        
        #does not work

    def get_rapid_modules(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The list of RAPID modules as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/rapid/modules")

    def get_leadthrough_state(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The leadthrough state of the robot.
            - int: The HTTP status code of the response.
        """
        return self.get_generic(f"/rw/motionsystem/mechunits/{mechunit_name}/lead-through", "status")
    
    def get_robot_baseframe(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The robot base frame information as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/baseframe")
    
    def get_robot_cartesian(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The robot cartesian position as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/cartesian")

    def get_robot_robtarget(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The robot robtarget position as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/robtarget")

    def get_robot_jointtarget(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The robot joint target position as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget")
    
    def get_rapid_execution_state(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The current state of the RAPID execution as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_generic_json("/rw/rapid/execution")

    def get_io_networks(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The IO networks of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_embed_json("/rw/iosystem/networks")
    
    def get_io_signals(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The IO signals of the robot as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_embed_json("/rw/iosystem/signals")

    def get_rapid_tasks(self) -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The list of RAPID tasks as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_embed_json("/rw/rapid/tasks")

    def get_task_robtarget(self, task_name = "T_ROB1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The robtarget information of a specific RAPID task as JSON string.
            - int: The HTTP status code of the response.
        Args:
            task_name: Name of the RAPID task (default: "T_ROB1").
        """
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/robtarget")
    
    def get_task_jointtarget(self, task_name = "T_ROB1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The jointtarget information of a specific RAPID task as JSON string.
            - int: The HTTP status code of the response.
        Args:
            task_name: Name of the RAPID task (default: "T_ROB1").
        """
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/jointtarget")
    
    def get_task_modules(self, task_name = "T_ROB1") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The list of modules in a specific RAPID task as JSON string.
            - int: The HTTP status code of the response.
        Args:
            task_name: Name of the RAPID task (default: "T_ROB1").
        """
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/modules")

    def get_io_signal(self, signal_name, network="", device="") -> tuple[str, int]:
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The IO signal state for a specific signal as JSON string.
            - int: The HTTP status code of the response.
        Args:
            signal_name: Name of the IO signal.
            network: Network name (optional, default: "").
            device: Device name (optional, default: "").
        Note:
            For some signals, network and device do not have to be specified.
        """
        if not signal_name:
            raise RWSException("Signal name cannot be empty")
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'

        return self.get_embed_json(f"/rw/iosystem/signals/{network}{device}{signal_name}")
    
    def get_rapid_symbol(self, symbol_name, module_name, task_name="T_ROB1"):
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The value of a specific RAPID symbol as string.
            - int: The HTTP status code of the response.
        Args:
            symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol.
            task_name: Name of the RAPID task (default: "T_ROB1").
        """
        if not symbol_name or not module_name:
            raise RWSException("Symbol name and module name cannot be empty")

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_generic(f"/rw/rapid/symbol/{symbol_url}/data", "value")

    def get_rapid_symbol_properties(self, symbol_name, module_name, task_name="T_ROB1"):
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The properties of a specific RAPID symbol as JSON string.
            - int: The HTTP status code of the response.
        Args:
            symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol.
            task_name: Name of the RAPID task (default: "T_ROB1").
        """
        if not symbol_name or not module_name:
            raise RWSException("Symbol name and module name cannot be empty")
        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_embed_json(f"/rw/rapid/symbol/{symbol_url}/properties")
    
    
    def get_dipc_queues(self):
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The information about the DIPC queues as JSON string.
            - int: The HTTP status code of the response.
        """
        return self.get_embed_json("/rw/dipc")
  
    
    def get_dipc_queue_info(self, queue_name="RMQ_T_ROB1"):
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - str: The information about a specific DIPC queue as JSON string.
            - int: The HTTP status code of the response.
        Args:
            queue_name: Name of the DIPC queue (default: "RMQ_T_ROB1").
        """
        return self.get_embed_json(f"/rw/dipc/{queue_name}/information")
    
    def read_dipc_message(self, queue_name="RMQ_T_ROB1", timeout=0):
        """ 
        Returns:
            tuple: 
            A tuple containing:
            - dict: The message data from the specified DIPC queue.
            - int: The HTTP status code of the response.
        Args:
            queue_name: Name of the DIPC queue (default: "RMQ_T_ROB1").
            timeout: Timeout in seconds (default: 0, which means no timeout).

        """

        if not queue_name:
            raise RWSException("Queue name cannot be empty")
        if not isinstance(timeout, int) or timeout < 0:
            raise RWSException("Timeout must be a non-negative integer")
        
        (data, status) = self.get_request(f"/rw/dipc/{queue_name}?timeout={timeout}")

        if status != 200:
            self.logger.error(f"Failed to read message from DIPC queue {queue_name}")

        return (data, status)

    def get_mastership_state(self, domain: str) -> tuple[str, int]:
        """
        Checks if the client currently holds mastership on the specified domain.
        Returns:
            tuple[str, int]: A tuple containing a string indicating the mastership state 
            ('True' if mastership is held by the client, 'False' otherwise) and an integer 
            status code from the request.
        Raises:
            RWSException: If an invalid domain is specified.
        Note:
            The domain must be either 'edit' or 'motion'. The method queries the 
            mastership state from the RWS interface and determines if the client 
            holds mastership for the given domain.
        """

        if domain not in ['edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership check")
            return ("ERR", -1)

        if domain:
            return self.get_generic_json(f"/rw/mastership/{domain}")

    ##########################################################################################################################

    ###################################
    ## POST METHODS
    ###################################

    def motors_on(self) -> tuple[str, int]:
        """ 
        Turn on the motors of the robot.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoron'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to turn on motors")
            return ("ERR - Failed to turn on motors", status)

    def motors_off(self) -> tuple[str, int]:
        """
        Turn off the motors of the robot.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoroff'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to turn off motors")
            return ("ERR - Failed to turn off motors", status)

    def restart_controller(self) -> tuple[str, int]:
        """
        Restart the robot controller.
        
        Note:
            Requires mastership on both edit and motion domains.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not (self.is_master('edit') and self.is_master('motion')):
            return ("ERR - Mastership on both domains is required", -1)

        status = self.post_request("/rw/panel/restart", dataIn={'restart-mode': 'restart'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to restart controller")
            return ("ERR - Failed to restart controller", status)

    def reset_pp(self) -> tuple[str, int]:
        """
        Reset the program pointer (PP) to the beginning of the program.
        
        Note:
            Requires mastership on the edit domain.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not self.is_master('edit'):
            self.logger.error("Mastership on edit domain is required to reset program pointer")
            return ("ERR - Mastership on edit domain is required to reset program pointer", -1)

        status = self.post_request("/rw/rapid/execution/resetpp")
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to reset program pointer")
            return ("ERR - Failed to reset program pointer", status)

    def start_rapid_script(self) -> tuple[str, int]:
        """
        Start the RAPID script execution.
        
        Note:
            Requires mastership ONLY on the edit domain. Mastership on motion domain will result in error.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        
        if not self.is_master('edit') or self.is_master('motion'):
            self.logger.error("Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error")
            return ("ERR - Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error", -1)

        status = self.post_request("/rw/rapid/execution/start", dataIn={'regain' :'clear' ,'execmode' :'continue' ,'cycle' : 'once' ,'condition' : 'callchain' ,'stopatbp' : 'disabled','alltaskbytsp' : 'false'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to start rapid script")
            return ("ERR - Failed to start rapid script", status)

    def stop_rapid_script(self) -> tuple[str, int]:
        """
        Stop the RAPID script execution.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        status = self.post_request("/rw/rapid/execution/stop", dataIn={'stopmode' : 'stop', 'usetsp' : 'normal'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to stop rapid script")
            return ("ERR - Failed to stop rapid script", status)

    def request_mastership(self, domain=None) -> tuple[str, int]:
        """
        Request explicit mastership on a specific domain or all domains.
        
        Args:
            domain (str, optional): The domain to request mastership on ("edit" or "motion"). 
                                  If None, requests mastership on all domains. Defaults to None.
        
        Note:
            Controller must be in automatic mode.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """

        if domain not in [None, 'edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership request")
            return ("ERR - Invalid domain specified", -1)
        
        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/request")
            if status != 204:
                self.logger.error(f"Failed to obtain mastership on {domain} domain")
                return (f"ERR - Failed to obtain mastership on {domain}", status)
            return ("OK", status)

        else:
            status = self.post_request("/rw/mastership/request")
            if status != 204:
                self.logger.error("Failed to obtain mastership on all domains")
                return ("ERR - Failed to obtain mastership on all domains", status)
            return ("OK", status)

        
    def release_mastership(self, domain=None)  -> tuple[str, int]:
        """
        Release mastership on a specific domain or all domains.
        
        Args:
            domain (str, optional): The domain to release mastership on ("edit" or "motion"). 
                                  If None, releases mastership on all domains. Defaults to None.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        
        if domain not in [None, 'edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership release")
            return ("ERR - Invalid domain specified", -1)

        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/release")
            return ("OK", status) if status == 204 else (f"ERR - Mastership on {domain} domain was not released", status)

        else:
            status = self.post_request("/rw/mastership/release")
            return ("OK", status) if status == 204 else ("ERR - Mastership on all domains was not released", status)

    def set_io_signal(self, signal_name: str, signal_value: str, network="", device="") -> tuple[str, int]:
        """
        Set the value of an I/O signal.
        
        Args:
            signal_name (str): The name of the I/O signal to set.
            signal_value (str): The value to set for the signal.
            network (str, optional): The network name. Defaults to "".
            device (str, optional): The device name. Defaults to "".
        
        Note:
            For some signals, network and device parameters do not have to be specified.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not signal_name or signal_value is None:
            self.logger.error("Signal name (param1) or value (param2) cannot be empty")
            return ("ERR - Signal name (param1) or value (param2) cannot be empty", -1)
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'


        path = f"/rw/iosystem/signals/{network}{device}{signal_name}/set-value"
        
        status = self.post_request(path, dataIn={'lvalue': signal_value})
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to set IO signal {signal_name}")
            return (f"ERR - Failed to set IO signal {signal_name}", status)
        

    def set_speedratio(self, speed_ratio: str)  -> tuple[str, int]:
        """
        Set the speed ratio of the robot.
        
        Args:
            speed_ratio (str): The speed ratio value as a string (must be between 0-100).
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        speed_ratio_int = int(speed_ratio)
        if not (0 <= speed_ratio_int <= 100):
            return ("ERR - Speed ratio must be between 0 and 100", -1)

        status = self.post_request("/rw/panel/speedratio", dataIn={'speed-ratio': speed_ratio})
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to set speed ratio to {speed_ratio}") 
            return (f"ERR - Failed to set speed ratio to {speed_ratio}", status)

    def reset_energy_info(self)  -> tuple[str, int]:
        """
        Reset the accumulated energy information of the robot.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        status = self.post_request("/rw/system/energy/reset")
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to reset energy information")
            return ("ERR - Failed to reset energy information", status)

    def set_rapid_symbol_raw(self, value: str, symbol_name: str, module_name: str, task_name: str = "T_ROB1")  -> tuple[str, int]:
        """
        Set the value of a specific RAPID symbol.
        
        Args:
            value (str): The value to set for the symbol (must be in string format).
            symbol_name (str): The name of the RAPID symbol.
            module_name (str): The name of the module containing the symbol.
            task_name (str, optional): The task name. Defaults to "T_ROB1".
        
        Note:
            Requires mastership on the edit domain.
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not symbol_name or not module_name:
            self.logger.error("Signal name (param1) or module name (param2) cannot be empty")
            return ("ERR - Signal name (param1) or module name (param2) cannot be empty", -1)

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"

        status = self.post_request(f"/rw/rapid/symbol/{symbol_url}/data", dataIn={'value': value})
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to set RAPID symbol {symbol_name} in module {module_name}")
            return (f"ERR - Failed to set RAPID symbol {symbol_name} in module {module_name}", status)
            


    def create_dipc_queue(self, queue_name: str, queue_size: str, message_size: str) -> tuple[str, int]:
        """
        Create a DIPC queue with the specified name and size.
        
        Args:
            queue_name (str): The name of the DIPC queue to create.
            queue_size (str): The size of the queue (must be a positive integer as string).
            message_size (str): The maximum message size (must be a positive integer as string).
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not queue_name:
            self.logger.error("Queue name cannot be empty")
            return ("ERR - Queue name cannot be empty", -1)
        
        if not queue_size.isdigit() or int(queue_size) <= 0:
            self.logger.error("Queue size must be a positive integer")
            return ("ERR - Queue size must be a positive integer", -1)

        if not message_size.isdigit() or int(message_size) <= 0:
            self.logger.error("Message size must be a positive integer")
            return ("ERR - Message size must be a positive integer", -1)

        datastr = f'dipc-queue-name={queue_name}&dipc-queue-size={queue_size}&dipc-max-msg-size={message_size}'
        # print(datastr)

        status = self.post_request("/rw/dipc", dataIn=datastr)

        if status == 201:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to create DIPC queue {queue_name}")
            return (f"ERR - Failed to create DIPC queue {queue_name}", status)

    def send_dipc_message(self, message: str, userdef: str = "1",  queue_name: str = "RMQ_T_ROB1",) -> tuple[str, int]:
        """
        Send a message to the specified DIPC queue.
        
        Args:
            message (str): The message content to send.
            userdef (str, optional): User-defined value (must be integer between 0-255 as string). Defaults to "1".
            queue_name (str, optional): The name of the DIPC queue. Defaults to "RMQ_T_ROB1".
        
        Returns:
            tuple: 
            A tuple containing:
            - str: "OK" if successful, error message if failed.
            - int: The HTTP status code of the response.
        """
        if not queue_name or not message:
            self.logger.error("Queue name and message cannot be empty")
            return ("ERR - Queue name and message cannot be empty", -1)
        
        if not userdef.isdigit() or not (0 <= int(userdef) <= 255):
            self.logger.error("Userdef must be an integer between 0 and 255")
            return ("ERR - Userdef must be an integer between 0 and 255", -1)
        
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": 0, "dipc-userdef": userdef,
                 "dipc-msgtype": 1, "dipc-data": message}
        

        # message_str = f"dipc-src-queue-name={queue_name}&dipc-cmd=111&dipc-userdef=222&dipc-msgtype=1&dipc-data={message}"
        
        status = self.post_request(f"/rw/dipc/{queue_name}/?action=dipc-send", dataIn=payload)

        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to send message to DIPC queue (QUEUE PROBABLY FULL){queue_name}")
            return (f"ERR - Failed to send message to DIPC queue (QUEUE PROBABLY FULL){queue_name}", status)

























    ########################################
    # GET METHODS - non string return types - not suitable for ROS communication due to return type,
    # but more convenient for Python applications, e.g. returning lists or booleans, 
    # TODO: Better documentation for these methods
    ########################################
    def is_running(self):
        """ Returns True if the RAPID execution is running, False otherwise """
        (data, status) = self.get_rapid_execution_state()
        if status != 200:
            self.logger.error("Failed to get RAPID execution state")
            raise RWSException("Failed to get RAPID execution state")
        #print(data)
        # Check if the execution state is 'running' or "stopped'

        if data.get('ctrlexecstate', 'unknown').lower() == 'running':
            return True
        elif data.get('ctrlexecstate', 'unknown').lower() == 'stopped':
            return False
        else:
            raise RWSException(f"Unknown RAPID execution state: {data.get('ctrlexecstate', 'unknown')}")
    
    def is_master(self, domain=None): 
        """ 
        Returns True if the client has mastership on given domain (Edit or Motion), False otherwise 
        """
        if domain not in ['edit', 'motion']:
            raise RWSException("Invalid domain specified")
        
        if domain:
            (data,status) = self.get_request(f"/rw/mastership/{domain}")
            mastership  = data.get('state', [])[0]
            held_by_me = mastership.get('mastershipheldbyme', False)
            if held_by_me:
                return True
            else:
                return False

    # def get_robot_cartesian_pose(self, mechunit_name = "ROB_1"):
    #     """ 
    #     Returns the robot cartesian position in format [x, y, z, q1, q2, q3, q4]
    #     """
    #     (data, status) = self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/cartesian")

    #     position = np.array([float(data.get('x', 0)), float(data.get('y', 0)), float(data.get('z', 0))])
    #     quaternion = np.array([float(data.get('q1', 0)), float(data.get('q2', 0)), 
    #                           float(data.get('q3', 0)), float(data.get('q4', 0))])

    #     result = np.concatenate([position, quaternion])

    #     return (result, status)
    
    # def get_robot_cartesian_pose_euler(self, mechunit_name = "ROB_1"):
    #     """ 
    #     Returns the robot cartesian position with Euler angles in format [x, y, z, rx, ry, rz]
    #     Euler angles are in degrees and follow ABB convention (xyz intrinsic rotations)
    #     """
    #     (cartesian_data, status) = self.get_robot_cartesian(mechunit_name)
        
    #     if status != 200:
    #         return (cartesian_data, status)
        
    #     # Extract position and quaternion
    #     position = cartesian_data[:3]
    #     quaternion = cartesian_data[3:]
        
    #     # Convert quaternion to Euler angles using ABB convention
    #     euler_angles = rwsutilities.abb_quaternion_to_euler_xyz(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
    #     # Combine position and Euler angles
    #     result = np.concatenate([position, euler_angles])
        
    #     return (result, status)
    
    # def get_robot_robtarget_pose(self, mechunit_name = "ROB_1"):
    #     """ 
    #     Returns the robot robtarget position in format [[x, y, z], [q1, q2, q3, q4]]
    #     """
    #     if not mechunit_name:
    #         raise RWSException("Mechanical unit name cannot be empty")

    #     (data, status) = self.get_request(f"/rw/motionsystem/mechunits/{mechunit_name}/robtarget")
    #     data = data['state'][0]
    #     #print(data)
    #     # Extract and convert to float
    #     x = float(data.get('x', 0))
    #     y = float(data.get('y', 0))
    #     z = float(data.get('z', 0))
    #     q1 = float(data.get('q1', 0))
    #     q2 = float(data.get('q2', 0))
    #     q3 = float(data.get('q3', 0))
    #     q4 = float(data.get('q4', 0))
    #     result = [[x, y, z], [q1, q2, q3, q4]]
    #     return (result, status)
    
    # def get_robot_jointtarget_list(self, mechunit_name = "ROB_1"):
    #     """ 
    #     Returns the robot joint target position in format [j1, j2, j3, j4, j5, j6]
    #     """
    #     if not mechunit_name:
    #         raise RWSException("Mechanical unit name cannot be empty")

    #     (data, status) = self.get_request(f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget")
    #     data = data['state'][0]
    #     #print(data)
    #     # Extract and convert to float
    #     joints = [float(data.get(f'rax_{i+1}', -999)) for i in range(6)]
    #     return (joints, status)
    
    # def get_io_networks_list(self): 
    #     """ Returns the IO networks of the robot as a list of strings """
    #     (data, status) = self.get_request("/rw/iosystem/networks")
    #     resources = data.get('_embedded', {}).get('resources', [])
    #     networks = [res.get('_title') for res in resources if res.get('_title')]
    #     #print(networks)
    #     return (networks, status)
    
    # def get_io_signals_list(self):
    #     """ Returns the IO signals of the robot as a list of strings """
    #     (data, status) = self.get_request("/rw/iosystem/signals")
    #     resources = data.get('_embedded', {}).get('resources', [])
    #     signals = [res.get('_title') for res in resources if res.get('_title')]
    #     #print(signals)
    #     return (signals, status)
    
    # def get_io_signals_list(self):
    #     """ Returns the IO signals of the robot """
    #     (data, status) = self.get_request("/rw/iosystem/signals")
    #     resources = data.get('_embedded', {}).get('resources', [])
    #     signals = [res.get('_title') for res in resources if res.get('_title')]
    #     #print(signals)
    #     return (signals, status)
    
    # def get_io_signal_old(self, signal_name, network="", device=""):
    #     """ 
    #     Returns the IO signal state for a specific signal. 
    #     For some signals, network and device does not have to be specified.
    #     """
    #     if not signal_name:
    #         raise RWSException("Signal name cannot be empty")
    #     if network:
    #         network = network if network.endswith('/') else network + '/'
    #     if device:
    #         device = device if device.endswith('/') else device + '/'

    #     path = f"/rw/iosystem/signals/{network}{device}{signal_name}"

    #     (data, status) = self.get_request(path)
    #     resource = data.get('_embedded', {}).get('resources', [])
    #     #print(resource)
    #     result = {'lvalue': resource[0].get('lvalue'),'type': resource[0].get('type')}

    #     return (result, status)
    
    # def get_task_robtarget_list(self, task_name = "T_ROB1"):
    #     """ 
    #     Returns the robtarget information of a specific RAPID task.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not task_name:
    #         raise RWSException("Task name cannot be empty")

    #     (data, status) = self.get_request(f"/rw/rapid/tasks/{task_name}/motion/robtarget")
    #     #print(data)
    #     data = data['state'][0]
    #     #print(data)
    #     x = float(data.get('x', 0))
    #     y = float(data.get('y', 0))
    #     z = float(data.get('z', 0))
    #     q1 = float(data.get('q1', 0))
    #     q2 = float(data.get('q2', 0))
    #     q3 = float(data.get('q3', 0))
    #     q4 = float(data.get('q4', 0))
    #     result = [[x, y, z], [q1, q2, q3, q4]]
    #     return (result, status)
    
    
    # def get_task_jointtarget_list(self, task_name = "T_ROB1"):
    #     """ 
    #     Returns the jointtarget information of a specific RAPID task.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not task_name:
    #         raise RWSException("Task name cannot be empty")

    #     (data, status) = self.get_request(f"/rw/rapid/tasks/{task_name}/motion/jointtarget")
    #     #print(data)
    #     data = data['state'][0]
    #     #print(data)
    #     joints = [float(data.get(f'rax_{i+1}', -999)) for i in range(6)]
    #     return (joints, status)
    
    # def get_rapid_symbol_int(self, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the integer value of a specific RAPID symbol.
    #     """
    #     (value, status) = self.get_rapid_symbol(symbol_name, module_name, task_name)
    #     try:
    #         value = int(value)
    #     except ValueError:
    #         raise RWSException(f"Symbol {symbol_name} in module {module_name} is not an integer")
        
    #     return (value, status)
    
    # def get_rapid_symbol_string(self, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the string value of a specific RAPID symbol.
    #     """
    #     (value, status) = self.get_rapid_symbol(symbol_name, module_name, task_name)
    #     if not isinstance(value, str):
    #         raise RWSException(f"Symbol {symbol_name} in module {module_name} is not a string")
        
    #     return (value, status)
    
    # def get_rapid_symbol_bool(self, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the boolean value of a specific RAPID symbol.
    #     """
    #     (value, status) = self.get_rapid_symbol(symbol_name, module_name, task_name)
    #     if isinstance(value, str):
    #         value = value.lower() in ['true', '1']
    #     elif isinstance(value, bool):
    #         value = bool(value)
    #     else:      
    #         raise RWSException(f"Symbol {symbol_name} in module {module_name} is not a boolean")
    #     return (value, status)
    
    # def get_rapid_symbol_float(self, symbol_name, module_name, task_name="T_ROB1"):
    #     "Returns the float value of a specific RAPID symbol."
    
    #     (value, status) = self.get_rapid_symbol(symbol_name, module_name, task_name)
    #     try:
    #         value = float(value)
    #     except ValueError:
    #         raise RWSException(f"Symbol {symbol_name} in module {module_name} is not a float")
        
    #     return (value, status)
    
    # def get_rapid_tool(self, tool_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the tooldata of specific tool.
    #     tool_name: Name of the tool.
    #     module_name: Name of the module.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not tool_name or not module_name:
    #         raise RWSException("Tool name and module name cannot be empty")
    #     (data, status) = self.get_rapid_symbol(tool_name, module_name, task_name)
    #     #print(data)
    #     tooldata = rwsutilities.string_to_list(data)

    #     return (tooldata, status)
    
    # def get_rapid_wobj(self, wobj_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the wobjdata of specific tool.
    #     wobj_name: Name of the workobject.
    #     module_name: Name of the module.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not wobj_name or not module_name:
    #         raise RWSException("Tool name and module name cannot be empty")
    #     (data, status) = self.get_rapid_symbol(wobj_name, module_name, task_name)
    #     #print(data)
    #     wobjdata = rwsutilities.string_to_list(data)

    #     return (wobjdata, status)
    
    # def get_rapid_robtarget(self, robtarget_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the robtarget data of specific robtarget.
    #     robtarget_name: Name of the robtarget.
    #     module_name: Name of the module.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not robtarget_name or not module_name:
    #         raise RWSException("Tool name and module name cannot be empty")
        
    #     (data, status) = self.get_rapid_symbol(robtarget_name, module_name, task_name)
    #     #print(data)
    #     robtarget = rwsutilities.string_to_list(data)

    #     return (robtarget, status)
    
    # def get_rapid_jointtarget(self, jointtarget_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Returns the jointtarget data of specific jointtarget.
    #     jointtarget_name: Name of the jointtarget.
    #     module_name: Name of the module.
    #     task_name: Name of the RAPID task.
    #     """
    #     if not jointtarget_name or not module_name:
    #         raise RWSException("Tool name and module name cannot be empty")
        
    #     (data, status) = self.get_rapid_symbol(jointtarget_name, module_name, task_name)
    #     #print(data)
    #     jointtarget = rwsutilities.string_to_list(data)

    #     return (jointtarget, status)
         

    # ##########################################################################################################################


    # ##################################
    # ## POST METHODS
    # ##################################


    # def motors_on(self):
    #     """ Turn on the motors """
    #     status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoron'})
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to turn on motors")

    # def motors_off(self):
    #     """ Turn off the motors """
    #     status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoroff'})
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to turn off motors")
        
    # def restart_controller(self):
    #     """ Restarts the controller, requires mastership on all domains. """
        
    #     if not (self.is_master('edit') and self.is_master('motion')):
    #         raise RWSException("Mastership on both edit and motion domains is required to restart the controller")

    #     status = self.post_request("/rw/panel/restart", dataIn={'restart-mode': 'restart'})
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to restart controller")

    # def reset_pp(self):
    #     """ Resets the program pointer (PP) (mastership on edit domain is required) """
    #     if not self.is_master('edit'):
    #         raise RWSException("Mastership on edit domain is required to reset program pointer")
        
    #     status = self.post_request("/rw/rapid/execution/resetpp")
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to reset program pointer")

    # def start_rapid_script(self):
    #     """ Starts the rapid script (mastership ONLY on edit domain is required. Mastership on motion domain will result in error) """
        
    #     if not self.is_master('edit') or self.is_master('motion'):
    #         raise RWSException("Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in error")

    #     status = self.post_request("/rw/rapid/execution/start", dataIn={'regain' :'clear' ,'execmode' :'continue' ,'cycle' : 'once' ,'condition' : 'callchain' ,'stopatbp' : 'disabled','alltaskbytsp' : 'false'})
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to start rapid script")
        
    # def stop_rapid_script(self):
    #     """ Stops the rapid script """
    #     status = self.post_request("/rw/rapid/execution/stop", dataIn={'stopmode' : 'stop', 'usetsp' : 'normal'})
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to stop rapid script")
        
    
    # def request_mastership(self, domain=None):
    #     """ 
    #     Requests explicit mastership on selected domain ("edit" or "motion"). None = all domains.
    #     """
    #     if domain not in [None, 'edit', 'motion']:
    #         raise RWSException("Invalid domain specified")
        
    #     if domain:
    #         self.post_request(f"/rw/mastership/{domain}/request")
    #     else:
    #         self.post_request("/rw/mastership/request")

    #     # Check if mastership was granted
    #     if domain is None:
    #         if self.is_master('edit') and self.is_master('motion'):
    #             return True
    #         return False
    #     else:
    #         return self.is_master(domain)
        
    # def release_mastership(self, domain=None):
    #     """ Releases mastership on given domain (Edit or Motion), 
    #     returns True if mastership was released, False otherwise. """
        
    #     if domain not in [None, 'edit', 'motion']:
    #         raise RWSException("Invalid domain specified")

    #     if domain:
    #         self.post_request(f"/rw/mastership/{domain}/release")
    #     else:
    #         self.post_request("/rw/mastership/release")

    #     if domain is None:
    #         if not (self.is_master('edit') or self.is_master('motion')):
    #             return True
    #         return False
    #     else:
    #         return not self.is_master(domain)
        
    # def set_io_signal(self,signal_name, signal_value, network="", device=""):
    #     """ 
    #     Sets the IO signal state for a specific signal.
    #     For some signals, network and device does not have to be specified.
    #     Returns True if the signal was set successfully
    #     """
    #     if not signal_name or signal_value is None:
    #         raise RWSException("Signal name or value cannot be empty")
    #     if network:
    #         network = network if network.endswith('/') else network + '/'
    #     if device:
    #         device = device if device.endswith('/') else device + '/'

    #     if isinstance(signal_value, bool):
    #         signal_value_str = '1' if signal_value else '0'
    #     elif isinstance(signal_value, int):
    #         signal_value_str = str(signal_value)

    #     path = f"/rw/iosystem/signals/{network}{device}{signal_name}/set-value"
        
    #     status = self.post_request(path, dataIn={'lvalue': signal_value_str})
        
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException(f"Failed to set IO signal {signal_name}")
        

    # def set_speedratio(self, speed_ratio):
    #     """ Sets the speed ratio of the robot (0-100) """
    #     speed_ratio = int(speed_ratio)
    #     if not (0 <= speed_ratio <= 100):
    #         raise RWSException("Speed ratio must be between 0 and 100")

    #     status = self.post_request("/rw/panel/speedratio", dataIn={'speed-ratio': str(speed_ratio)})
        
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException(f"Failed to set speed ratio to {speed_ratio}")
        
    # def reset_energy_info(self):
    #     """ Resets the accumulated energy information of the robot """
    #     status = self.post_request("/rw/system/energy/reset")
        
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException("Failed to reset energy information")
        
    # def set_rapid_symbol_raw(self, value, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Sets the value of a specific RAPID symbol
    #     Value has to be in string format.
    #     Requires mastership on edit domain.
    #     """
    #     if not symbol_name or not module_name:
    #         raise RWSException("Symbol name and module name cannot be empty")
        
    #     symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"

    #     status = self.post_request(f"/rw/rapid/symbol/{symbol_url}/data", dataIn={'value': value})
        
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException(f"Failed to set RAPID symbol {symbol_name} in module {module_name}")
    
    # def set_rapid_symbol_int(self, value, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Sets the integer value of a specific RAPID symbol.
    #     Requires mastership on edit domain.
    #     """

    #     if not isinstance(value, int):
    #         try:
    #             value = int(value)
    #         except ValueError:
    #             raise RWSException("Value cannot be converted to an integer")
        
    #     status = self.set_rapid_symbol_raw(str(value), symbol_name, module_name, task_name)
    #     return status
    
    # def set_rapid_symbol_string(self, value, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Sets the string value of a specific RAPID symbol.
    #     Requires mastership on edit domain.
    #     """
    #     if not isinstance(value, str):
    #         raise RWSException("Value must be a string")
    #     value_quoted = f'"{value}"'  # RAPID strings are quoted

    #     status = self.set_rapid_symbol_raw(value_quoted, symbol_name, module_name, task_name)
    #     return status
    
    # def set_rapid_symbol_float(self, value, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Sets the float value of a specific RAPID symbol.
    #     Requires mastership on edit domain.
    #     """
    #     if not isinstance(value, float):
    #         try:
    #             value = float(value)
    #         except ValueError:
    #             raise RWSException("Value cannot be converted to a float")
        
    #     status = self.set_rapid_symbol_raw(str(value), symbol_name, module_name, task_name)
    #     return status
    
    # def set_rapid_symbol_list(self, value, symbol_name, module_name, task_name="T_ROB1"):
    #     """ 
    #     Requires mastership on edit domain.
    #     """
    #     value_str = rwsutilities.list_to_string(value)
    #     status = self.set_rapid_symbol_raw(value_str, symbol_name, module_name, task_name)
    #     return status
    
    # def create_dipc_queue(self, queue_name, queue_size=100, message_size=100):
    #     """ 
    #     Creates an DIPC queue with the specified name and size.
    #     """
    #     if not queue_name:
    #         raise RWSException("Queue name cannot be empty")
    #     datastr = f'dipc-queue-name={queue_name}&dipc-queue-size={queue_size}&dipc-max-msg-size={message_size}'
    #     print(datastr)

    #     status = self.post_request("/rw/dipc", dataIn=datastr)

    #     if status == 201:
    #         return True
    #     else:
    #         raise RWSException(f"Failed to create IPC queue {queue_name}")
        
    # def send_dipc_message(self,message, queue_name = "RMQ_T_ROB1"):
    #     """ 
    #     Sends a message to the specified DIPC queue.
    #     """
    #     if not queue_name or not message:
    #         raise RWSException("Queue name and message cannot be empty")
            
    #     payload={"dipc-src-queue-name": queue_name, "dipc-cmd": 0, "dipc-userdef": 1,
    #              "dipc-msgtype": 1, "dipc-data": message}

    #     # message_str = f"dipc-src-queue-name={queue_name}&dipc-cmd=111&dipc-userdef=222&dipc-msgtype=1&dipc-data={message}"
        
    #     status = self.dipc_post_request(f"/rw/dipc/{queue_name}/?action=dipc-send", dataIn=payload)

        
    #     if status == 204:
    #         return True
    #     else:
    #         raise RWSException(f"Failed to send message to IPC queue {queue_name}, queue is propably full")

