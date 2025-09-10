import time
import sys
import os

sys.path.append(os.path.dirname(__file__))
from rwsinterfaceV2 import RWSInterface

import numpy as np
import json

robotIP = "192.168.0.37"
username = "Admin"
password = "robotics"

if __name__ == "__main__":
    
    try:
        

        rws = RWSInterface(robotIP, username, password, port=443)

        rws.login()

        print("Logged in:", rws.get_login_state())


        # # Get robot positions using RWSInterface methods
        # cartesian = rws.get_robot_cartesian("ROB_1")[0]
        # print("Cartesian position (JSON):", cartesian)

        # robtarget = rws.get_robot_robtarget("ROB_1")[0]
        # print("Robtarget position (JSON):", robtarget)

        # jointtarget = rws.get_robot_jointtarget("ROB_1")[0] 
        # print("Jointtarget position (JSON):", jointtarget)

        # get_rapid_exec = rws.get_rapid_execution_state()[0]
        # print("RAPID execution state (JSON):", get_rapid_exec)

        # get_io_nets = rws.get_io_networks()[0]
        # print("IO Networks (JSON):", get_io_nets)

        # get_io_signals = rws.get_io_signals()[0]
        # print("IO Signals (JSON):", get_io_signals)

        # get_io_signal = rws.get_io_signal("OUTPUT_STATIONARY_ROB_1")[0]
        # print("IO Signal OUTPUT_STATIONARY_ROB_1 (JSON):", get_io_signal)
        # print(type(get_io_signal))


        # modules = rws.get_rapid_modules()[0]
        # print("RAPID Modules (JSON):", modules)

        # tasks = rws.get_rapid_tasks()[0]
        # print("RAPID Tasks (JSON):", tasks)

        # task_robt = rws.get_task_robtarget("T_ROB1")[0]
        # print("Task T_ROB1 robtargets (JSON):", task_robt)

        # task_jt = rws.get_task_jointtarget("T_ROB1")[0]
        # print("Task T_ROB1 jointtargets (JSON):", task_jt)

        # task_modules = rws.get_task_modules("T_ROB1")[0]
        # print("Task T_ROB1 modules (JSON):", task_modules)

        get_rapid_symbol = rws.get_rapid_symbol("Target_30", "TRobTargets", "T_ROB1")[0]
        print("RAPID symbol Target_30 in TRobTargets (JSON):", get_rapid_symbol)

        # get_rapid_symbol_prop = rws.get_rapid_symbol_properties("Target_30", "TRobTargets", "T_ROB1")[0]
        # print("RAPID symbol Target_30 properties in TRobTargets (JSON):", get_rapid_symbol_prop)

        # # print("Mastership edit:", rwsWrap.is_master("edit"))
        # # # #rwsWrap.stop_rapid_script()
        # print(rwsWrap.release_mastership("edit"))
        # time.sleep(1)
        # print("Requesting mastership edit:", rwsWrap.request_mastership("edit"))
        # print("Mastership edit:", rwsWrap.is_master("edit"))
        # # rwsWrap.motors_on()
        # # rwsWrap.reset_pp()
        # # # time.sleep(1)
        # # rwsWrap.start_rapid_script()
        # print(rwsWrap.set_rapid_symbol_string("tool_tuzka", "tool_name", "TRobUser"))
        # # print(rwsWrap.set_rapid_symbol_string("tool_tuzka", "tool_name", "TRobUser"))
        # # time.sleep(1)
        # print(rwsWrap.set_rapid_symbol_string("change_tool_new", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))
        # time.sleep(6)
        # print(rwsWrap.set_rapid_symbol_string("smaz_tabuli", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))

        # print(rwsWrap.get_task_modules())
        # print("Mastership mot:", rwsWrap.is_master("motion"))
        # print(rwsWrap.release_mastership("edit"))
        #print(rwsWrap.create_dipc_queue("T_ROB1"))
        # print(rwsWrap.get_dipc_queues())
        # print(rwsWrap.get_dipc_queue_info())
        # print(rwsWrap.send_dipc_message('num;22'))
        # print(rwsWrap.start_rapid_script())

        # time.sleep(6)

        # #read points from csv file line by line
        # with open('path.csv', 'r') as f:
        #     points = f.readlines()
        # points = [line.strip().split(',') for line in points]
        # points = [[float(coord) for coord in point] for point in points]
        # # print(points)
        # i =0 

        # s = np.linspace(0, 2*np.pi, 100)
        # X = 700 + 300 * np.sin(s)
        # Y = 700 +300 * np.cos(s) 
        # i=0

        # print(rwsWrap.read_dipc_message(timeout=5))
        # print("cc")

        # print(rwsWrap.set_rapid_symbol_string("draw_routine_buffer", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))


        # print(rwsWrap.set_rapid_symbol_string("runMoveJ", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_list([[500,500,0],[1,0,0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]], "move_robtarget_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))
        # # # print(rwsWrap.send_dipc_message('string;"cccc"'))
        # # # print(rwsWrap.send_dipc_message('string;"ahoj"'))
        # # # print(rwsWrap.send_dipc_message('string;"runMoveJ"'))
        # time.sleep(2)

        # # initialPos = [[1000, 0, 700], [0, 0, -1.0, 0], [0, 0, 0, 0], [9E+9, 9E+9, 9E+9, 9E+9, 9E+9, 9E+9]]

        # while True:
        #     try:
        #         if i < len(points):

        #             x,y,z = points[i]
        #             status = rwsWrap.send_dipc_message(f'[[{x},{y},{-z}],[0,1,0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]', 'robtarget')
        #             if status is True:
        #                 i += 1

        #         else:
        #             break
        #         # if initialPos[0][0] > -1200:
        #         #     initialPos[0][0] -= 100
        #         # else:
        #         #     initialPos[0][0] = 1200



        #     except Exception as e:
        #         print(f"Error sending message: {e}")
        #         time.sleep(0.4)
        #         continue
        #     time.sleep(0.1)

        # time.sleep(1)
        # status = rwsWrap.send_dipc_message(f'[[{0},{-200},{200}],[0,1,0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]', 'robtarget')
        # time.sleep(5)
        # print(rwsWrap.set_rapid_symbol_raw(False, "run_buffer", "TRobUser"))
        
        # time.sleep(15)

        # print(rwsWrap.set_rapid_symbol_string("tool_houba", "tool_name", "TRobUser"))
        # # print(rwsWrap.set_rapid_symbol_string("tool_tuzka", "tool_name", "TRobUser"))
        # # time.sleep(1)
        # print(rwsWrap.set_rapid_symbol_string("change_tool_new", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))

        # time.sleep(6)
        # print(rwsWrap.set_rapid_symbol_string("smaz_tabuli", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))

        # print(rwsWrap.send_dipc_message('robtarget;[[1200,0,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[800,0,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[1000,500,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[-600,0,800],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))

        # # print(rwsWrap.read_dipc_message(timeout=5))
        # print(rwsWrap.read_dipc_message(timeout=5))
        #(rwsWrap.send_dipc_message("Hello world!"))
        
        # print(rwsWrap.start_rapid_script())

        # robt = rwsWrap.get_rapid_robtarget("move_robtarget_input", "TRobRAPID")[0]

        # robt[0][0] = 800

        # print(rwsWrap.set_rapid_symbol_list(robt, "move_robtarget_input", "TRobRAPID"))

        time.sleep(2)


    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rws.logout()