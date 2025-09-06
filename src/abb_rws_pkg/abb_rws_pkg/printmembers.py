import os
import sys
import inspect

sys.path.append(os.path.dirname(__file__))
from utility import Utilities
from rwsinterface import RWSInterface

if __name__ == "__main__":

    utilities = Utilities(None)

    logger = utilities.logger
    logger.info('Starting ABB RWS Print Members Script...')

    config_path = os.path.join('conf', 'robot_config.yaml')
    config = utilities.load_config(config_path)['robot_config']

    utilities.log_config(config)

    try:
        RWS = RWSInterface(
            host=config['connection']['ip_address'],
            username=config['connection']['username'],
            password=config['connection']['password'],
            port=config['connection']['port'],
            keepalive= config['connection'].get('keepalive', True),
            keepalive_interval=config['connection'].get('keepalive_interval', 180),
            auto_cleanup=True,
            logger=logger
        )
    except Exception as e:
        logger.error(f'Failed to initialize RWSInterface: {e}')
        exit(1)

    methods = [name for name, obj in inspect.getmembers(RWSInterface, predicate=inspect.isfunction)]

    get_methods = [m for m in methods if m.startswith('get')]

    # logger.info(get_methods)

    for method in get_methods:
        logger.info(method)  
    
    # try:
    #     RWS.login()
    #     logger.info("Logged in successfully.")

    #     members = dir(RWS)
    #     logger.info("RWS Interface Members:")
    #     for member in members:
    #         logger.info(member)

    #     methods = [m for m in members if callable(getattr(RWS, m)) and not m.startswith("_")]
    #     logger.info("RWS Interface Methods:")
    #     for method in methods:
    #         logger.info(method)

    # except Exception as e:
    #     logger.error(f'Error during RWS operations: {e}')
    # finally:
    #     RWS.logout()
    #     logger.info("Logged out.")