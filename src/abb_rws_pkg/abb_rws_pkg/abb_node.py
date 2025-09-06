from interface_pkg.srv import RWSCommandNoParams, ResponseOnly, RWSCommand, FloatResponse, IntResponse, StringResponse, BoolResponse
from interface_pkg.msg import BoolMsg
import rclpy
from rclpy.node import Node
import os
import sys
import numpy as np
import traceback

sys.path.append(os.path.dirname(__file__))
from utility import Utilities
from rwsinterface import RWSInterface

class AbbNode(Node):

    def __init__(self):
        super().__init__('abb_node')
        self._logged_in = False

        self.logger = self.get_logger()

        self.logger.info('Starting ABB Node...')
        self.utilities = Utilities(self.logger)

        config_path = os.path.join('conf', 'robot_config.yaml')
        self.config = self.utilities.load_config(config_path)['robot_config']

        self.utilities.log_config(self.config)

        #load calibration matrix
        try:
            calib_path = self.config['calibration']['calibration_matrix_path']
            calib_mat = np.loadtxt(calib_path)
            self.logger.info(f"Camera calibration matrix loaded:\n{calib_mat}")
        except Exception as e:
            self.logger.error(f'Failed to load camera calibration matrix: {e}')

        try:
            self.RWS = RWSInterface(
                host=self.config['connection']['ip_address'],
                username=self.config['connection']['username'],
                password=self.config['connection']['password'],
                port=self.config['connection']['port'],
                logger=self.logger
            )
        except Exception as e:
            self.logger.error(f'Failed to initialize RWSInterface: {e}')
            return
        
        if self.config['connection']['keepalive']:
            period = self.config['connection'].get('keepalive_period', 60.0)
            self.keepalive_timer = self.create_timer(period, self.keepalive_callback)

            self.keepalive_publisher = self.create_publisher(BoolMsg, 'abb_node/keepalive_ok', 10)



        self.get_service = self.create_service(RWSCommandNoParams, 'abb_node/rws_get_no_params', self.rws_get_callback_no_params)
        self.login_service = self.create_service(ResponseOnly, 'abb_node/login', self.login_callback)
        self.logout_service = self.create_service(ResponseOnly, 'abb_node/logout', self.logout_callback)
        self.get_rapid_float_service = self.create_service(FloatResponse, 'abb_node/get_rapid_symbol_float', self.get_rapid_symbol_float_callback)
        self.get_rapid_int_service = self.create_service(IntResponse, 'abb_node/get_rapid_symbol_int', self.get_rapid_symbol_int_callback)
        self.get_rapid_string_service = self.create_service(StringResponse, 'abb_node/get_rapid_symbol_string', self.get_rapid_symbol_string_callback)
        self.get_rapid_bool_service = self.create_service(BoolResponse, 'abb_node/get_rapid_symbol_bool', self.get_rapid_symbol_bool_callback)

        self.logger.info('ABB Node initialized successfully.')

        # self.post_service = self.create_service(RWSPostCommand, 'rws_post', self.rws_post_callback)


    def login_callback(self, request, response):
        self.logger.info('Login service called')
        try:
            status = self.RWS.login()
            self._logged_in = True
            self.logger.info('Login successful')
            response.status_code = status  # Success

        except Exception as e:
            self.logger.error(f'Login failed: {e}')
            response.status_code = -1
        return response


    def logout_callback(self, request, response):
        self.logger.info('Logout service called')
        try:
            status = self.RWS.logout()
            response.status_code = status  # Success
            self._logged_in = False
            self.logger.info('Logout successful')

        except Exception as e:
            self.logger.error(f'Logout failed: {e}')
            response.status_code = -1
        return response
        
    def rws_get_callback_no_params(self, request, response):
        self.logger.info(f'Received RWS GET command: {request.command_type}')
        command = request.command_type
        try:
            if command == 'is_logged_in':
                method = getattr(self.RWS, command)
                result = method()
                if result == True:
                    self._logged_in = True
                else:
                    self._logged_in = False

                response.result_str = str(result) # Explicitly set unused field
                response.status_code = 200
                return response
            
            # Dynamically call method on self.RWS based on command_type string
            if self._logged_in == True:
                if hasattr(self.RWS, command):
                    method = getattr(self.RWS, command)
                    (result, status_code) = method()
                    
                else:
                    result = 'unknown_command'
                    status_code = -1
                self.logger.info(f"Type of result: {type(result)}")

                response.result_str = str(result)
                response.status_code = status_code
                return response
            else:
                response.result_str = 'not_logged_in'
                response.status_code = -2
                return response
        
        except Exception as e:
            self.logger.error(f'Error executing RWS command: {e}')
            response.result_str = str(e)
            response.status_code = -1

        return response
        
    def keepalive_callback(self):
        if self._logged_in:
            self.logger.info('Sending keepalive signal...')
            self._logged_in = False
            try:
                result = self.RWS.send_keepalive()
                msg = BoolMsg()
                if result == True:
                    msg.data = True
                    self.keepalive_publisher.publish(msg)
                    self._logged_in = True
                    self.logger.info('Keepalive successful')
                else:
                    msg.data = False
                    self.keepalive_publisher.publish(msg)

            except Exception as e:
                self.logger.error(f'Keepalive failed: {e}')
        else:
            self.logger.info('Not logged in, skipping keepalive.')

    def get_rapid_symbol_float_callback(self, request, response):
        self.logger.info(f'Received get_rapid_symbol_float command for {request.object_name} in {request.module_name}')
        if self._logged_in == True:
            try:
                (result, status_code) = self.RWS.get_rapid_symbol_float(str(request.object_name), str(request.module_name))
                response.result = result
                response.status_code = status_code
                return response
            except Exception as e:
                self.logger.error(f'Error getting RAPID float symbol: {e}')
                response.result = 0.0
                response.status_code = -1
                return response
        else:
            self.logger.error('Not logged in, cannot get RAPID float symbol.')
            response.result = 0.0
            response.status_code = -2
            return response
    
    def get_rapid_symbol_int_callback(self, request, response):
        self.logger.info(f'Received get_rapid_symbol_int command for {request.object_name} in {request.module_name}')
        if self._logged_in == True:
            try:
                (result, status_code) = self.RWS.get_rapid_symbol_int(str(request.object_name), str(request.module_name))
                response.result = result
                response.status_code = status_code
                return response
            except Exception as e:
                self.logger.error(f'Error getting RAPID int symbol: {e}')
                response.result = 0
                response.status_code = -1
                return response
        else:
            self.logger.error('Not logged in, cannot get RAPID int symbol.')
            response.result = 0
            response.status_code = -2
            return response

    def get_rapid_symbol_string_callback(self, request, response):
        self.logger.info(f'Received get_rapid_symbol_string command for {request.object_name} in {request.module_name}')
        if self._logged_in == True:
            try:
                (result, status_code) = self.RWS.get_rapid_symbol_string(request.object_name, request.module_name)
                response.result = result
                response.status_code = status_code
                return response
            except Exception as e:
                self.logger.error(f'Error getting RAPID string symbol: {e}')
                response.result = ''
                response.status_code = -1
                return response
        else:
            self.logger.error('Not logged in, cannot get RAPID string symbol.')
            response.result = ''
            response.status_code = -2
            return response

    def get_rapid_symbol_bool_callback(self, request, response):
        self.logger.info(f'Received get_rapid_symbol_bool command for {request.object_name} in {request.module_name}')
        if self._logged_in == True:
            try:
                (result, status_code) = self.RWS.get_rapid_symbol_bool(request.object_name, request.module_name)
                response.result = result
                response.status_code = status_code
                return response
            except Exception as e:
                self.logger.error(f'Error getting RAPID bool symbol: {e}')
                response.result = False
                response.status_code = -1
                return response
        else:
            self.logger.error('Not logged in, cannot get RAPID bool symbol.')
            response.result = False
            response.status_code = -2
            return response
        pass



def main(args=None):

    import debugpy
    debugpy.listen(5678)
    print("Waiting for debugger attach...")
    debugpy.wait_for_client()


    rclpy.init(args=args)

    try:
        abb_node = AbbNode()
        rclpy.spin(abb_node)
    except KeyboardInterrupt:
        if abb_node:
            abb_node.get_logger().error(f'KeyboardInterrupt occurred')

    except Exception as e:

        if abb_node:
            abb_node.get_logger().error(f'Exception occurred: {str(e)}')
            abb_node.get_logger().error(f'Traceback: {traceback.format_exc()}')
        else:
            print(f'Exception during node creation: {str(e)}')
            traceback.print_exc()

    finally:

        abb_node.get_logger().info('Shutting down ABB Node...')
        abb_node.RWS.logout()
        abb_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()