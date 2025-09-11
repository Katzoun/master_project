# RWS Interface - GET Methods Documentation

This document provides comprehensive documentation for all GET methods in the RWSInterface class that return string outputs. These methods are suitable for ROS communication and return data as JSON strings or simple string values.

## Method Categories

- [Controller State Methods](#controller-state-methods)
- [System Information Methods](#system-information-methods)
- [Motion System Methods](#motion-system-methods)
- [RAPID Methods](#rapid-methods)
- [IO System Methods](#io-system-methods)
- [DIPC Methods](#dipc-methods)

---

## Controller State Methods

### `get_clock()`
**Returns the current clock value from the robot controller.**

```python
clock, status = rws.get_clock()
```

**Returns:**
- `str`: Current clock value from the robot controller
- `int`: HTTP status code (200 = success)

**Example Output:**
```python
("2025-09-11t14:30:25", 200)
```

---

### `get_controller_state()`
**Returns the state of the controller.**

```python
state, status = rws.get_controller_state()
```

**Returns:**
- `str`: Controller state (motors on/off, guardstop, emergencystop, init)
- `int`: HTTP status code

**Possible Values:**
- `"init"` - Controller initializing
- `"motoron"` - Motors are on
- `"motoroff"` - Motors are off
- `"guardstop"` - Emergency stop activated
- `"emergencystop"` - Emergency stop pressed

---

### `get_opmode_state()`
**Returns the current operational mode of the robot.**

```python
mode, status = rws.get_opmode_state()
```

**Returns:**
- `str`: Operational mode (auto, man, manf)
- `int`: HTTP status code

**Possible Values:**
- `"auto"` - Automatic mode
- `"man"` - Manual mode
- `"manf"` - Manual full speed mode

---

### `get_safety_mode()`
**Returns the current safety mode of the robot.**

```python
safety_mode, status = rws.get_safety_mode()
```

**Returns:**
- `str`: Current safety mode
- `int`: HTTP status code

---

### `get_speedratio()`
**Returns the current speed ratio of the robot.**

```python
speed, status = rws.get_speedratio()
```

**Returns:**
- `str`: Speed ratio (0-100)
- `int`: HTTP status code

---

## System Information Methods

### `get_robot_type()`
**Returns the type of the robot.**

```python
robot_type, status = rws.get_robot_type()
```

**Returns:**
- `str`: Robot type (e.g., "CRB 15000-10/1.52")
- `int`: HTTP status code

---

### `get_network_info()`
**Returns the network information of the robot as JSON string.**

```python
network_info, status = rws.get_network_info()
```

**Returns:**
- `str`: Network information as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "_type": "ctrl-netw",
    "_title": "X6",
    "logical-name": "LAN1",
    "network": "Private",
    "addr": "192.168.125.1",
    "mask": "255.255.255.0",
    "gateway": "0.0.0.0"
  }
]
```

---

### `get_system_options()`
**Returns the system options of the robot as JSON string.**

```python
options, status = rws.get_system_options()
```

**Returns:**
- `str`: System options as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "_type": "sys-option-li",
    "_title": "0",
    "option": "RobotControl Base"
  },
  {
    "_type": "sys-option-li",
    "_title": "1",
    "option": "English"
  }
]
```

---

### `get_system_products()`
**Returns the system products of the robot as JSON string.**

```python
products, status = rws.get_system_products()
```

**Returns:**
- `str`: System products as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "_type": "sys-product-li",
    "_title": "RobotOS",
    "version": "16.0.0+231",
    "version-name": "16.0.0"
  }
]
```

---

### `get_energy_info()`
**Returns the energy information of the robot as JSON string.**

```python
energy_info, status = rws.get_energy_info()
```

**Returns:**
- `str`: Energy information as JSON string
- `int`: HTTP status code

---

## Motion System Methods

### `get_mechunits()`
**Returns the mechanical unit information of the robot as JSON string.**

```python
mechunits, status = rws.get_mechunits()
```

**Returns:**
- `str`: Mechanical unit information as JSON string
- `int`: HTTP status code

---

### `get_leadthrough_state(mechunit_name="ROB_1")`
**Returns the leadthrough state of the robot.**

```python
leadthrough_state, status = rws.get_leadthrough_state()
leadthrough_state, status = rws.get_leadthrough_state("ROB_1")
```

**Parameters:**
- `mechunit_name` (str, optional): Mechanical unit name (default: "ROB_1")

**Returns:**
- `str`: Leadthrough state
- `int`: HTTP status code

---

### `get_robot_baseframe(mechunit_name="ROB_1")`
**Returns the robot base frame information as JSON string.**

```python
baseframe, status = rws.get_robot_baseframe()
```

**Parameters:**
- `mechunit_name` (str, optional): Mechanical unit name (default: "ROB_1")

**Returns:**
- `str`: Base frame information as JSON string
- `int`: HTTP status code

---

### `get_robot_cartesian(mechunit_name="ROB_1")`
**Returns the robot cartesian position as JSON string.**

```python
cartesian, status = rws.get_robot_cartesian()
```

**Parameters:**
- `mechunit_name` (str, optional): Mechanical unit name (default: "ROB_1")

**Returns:**
- `str`: Cartesian position as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "x": 500.0,
    "y": 0.0,
    "z": 600.0,
    "q1": 1.0,
    "q2": 0.0,
    "q3": 0.0,
    "q4": 0.0
  }
]
```

---

### `get_robot_robtarget(mechunit_name="ROB_1")`
**Returns the robot robtarget position as JSON string.**

```python
robtarget, status = rws.get_robot_robtarget()
```

**Parameters:**
- `mechunit_name` (str, optional): Mechanical unit name (default: "ROB_1")

**Returns:**
- `str`: Robtarget position as JSON string
- `int`: HTTP status code

---

### `get_robot_jointtarget(mechunit_name="ROB_1")`
**Returns the robot joint target position as JSON string.**

```python
jointtarget, status = rws.get_robot_jointtarget()
```

**Parameters:**
- `mechunit_name` (str, optional): Mechanical unit name (default: "ROB_1")

**Returns:**
- `str`: Joint target position as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "rax_1": 0.0,
    "rax_2": 0.0,
    "rax_3": 0.0,
    "rax_4": 0.0,
    "rax_5": 90.0,
    "rax_6": 0.0
  }
]
```

---

## RAPID Methods

### `get_rapid_modules()`
**Returns the list of RAPID modules as JSON string.**

```python
modules, status = rws.get_rapid_modules()
```

**Returns:**
- `str`: List of RAPID modules as JSON string
- `int`: HTTP status code

---

### `get_rapid_execution_state()`
**Returns the current state of the RAPID execution as JSON string.**

```python
exec_state, status = rws.get_rapid_execution_state()
```

**Returns:**
- `str`: RAPID execution state as JSON string
- `int`: HTTP status code

**Example JSON Structure:**
```json
[
  {
    "ctrlexecstate": "stopped",
    "cycle": "forever",
    "running": false
  }
]
```

---

### `get_rapid_tasks()`
**Returns the list of RAPID tasks as JSON string.**

```python
tasks, status = rws.get_rapid_tasks()
```

**Returns:**
- `str`: List of RAPID tasks as JSON string
- `int`: HTTP status code

---

### `get_task_robtarget(task_name="T_ROB1")`
**Returns the robtarget information of a specific RAPID task as JSON string.**

```python
robtarget, status = rws.get_task_robtarget()
robtarget, status = rws.get_task_robtarget("T_ROB1")
```

**Parameters:**
- `task_name` (str, optional): Name of the RAPID task (default: "T_ROB1")

**Returns:**
- `str`: Robtarget information as JSON string
- `int`: HTTP status code

---

### `get_task_jointtarget(task_name="T_ROB1")`
**Returns the jointtarget information of a specific RAPID task as JSON string.**

```python
jointtarget, status = rws.get_task_jointtarget()
```

**Parameters:**
- `task_name` (str, optional): Name of the RAPID task (default: "T_ROB1")

**Returns:**
- `str`: Jointtarget information as JSON string
- `int`: HTTP status code

---

### `get_task_modules(task_name="T_ROB1")`
**Returns the list of modules in a specific RAPID task as JSON string.**

```python
modules, status = rws.get_task_modules()
```

**Parameters:**
- `task_name` (str, optional): Name of the RAPID task (default: "T_ROB1")

**Returns:**
- `str`: List of modules as JSON string
- `int`: HTTP status code

---

### `get_rapid_symbol(symbol_name, module_name, task_name="T_ROB1")`
**Returns the value of a specific RAPID symbol as string.**

```python
value, status = rws.get_rapid_symbol("counter", "MainModule")
value, status = rws.get_rapid_symbol("counter", "MainModule", "T_ROB1")
```

**Parameters:**
- `symbol_name` (str): Name of the RAPID symbol
- `module_name` (str): Name of the module containing the symbol
- `task_name` (str, optional): Name of the RAPID task (default: "T_ROB1")

**Returns:**
- `str`: Value of the RAPID symbol as string
- `int`: HTTP status code

---

### `get_rapid_symbol_properties(symbol_name, module_name, task_name="T_ROB1")`
**Returns the properties of a specific RAPID symbol as JSON string.**

```python
properties, status = rws.get_rapid_symbol_properties("counter", "MainModule")
```

**Parameters:**
- `symbol_name` (str): Name of the RAPID symbol
- `module_name` (str): Name of the module containing the symbol
- `task_name` (str, optional): Name of the RAPID task (default: "T_ROB1")

**Returns:**
- `str`: Properties of the RAPID symbol as JSON string
- `int`: HTTP status code

---

## IO System Methods

### `get_io_networks()`
**Returns the IO networks of the robot as JSON string.**

```python
networks, status = rws.get_io_networks()
```

**Returns:**
- `str`: IO networks as JSON string
- `int`: HTTP status code

---

### `get_io_signals()`
**Returns the IO signals of the robot as JSON string.**

```python
signals, status = rws.get_io_signals()
```

**Returns:**
- `str`: IO signals as JSON string
- `int`: HTTP status code

---

### `get_io_signal(signal_name, network="", device="")`
**Returns the IO signal state for a specific signal as JSON string.**

```python
signal_state, status = rws.get_io_signal("DO1")
signal_state, status = rws.get_io_signal("DO1", "DeviceNet", "Device1")
```

**Parameters:**
- `signal_name` (str): Name of the IO signal
- `network` (str, optional): Network name (default: "")
- `device` (str, optional): Device name (default: "")

**Returns:**
- `str`: IO signal state as JSON string
- `int`: HTTP status code

**Note:** For some signals, network and device do not have to be specified.

---

## DIPC Methods

### `get_dipc_queues()`
**Returns the information about the DIPC queues as JSON string.**

```python
queues, status = rws.get_dipc_queues()
```

**Returns:**
- `str`: DIPC queues information as JSON string
- `int`: HTTP status code

---

### `get_dipc_queue_info(queue_name="RMQ_T_ROB1")`
**Returns the information about a specific DIPC queue as JSON string.**

```python
queue_info, status = rws.get_dipc_queue_info()
queue_info, status = rws.get_dipc_queue_info("MyQueue")
```

**Parameters:**
- `queue_name` (str, optional): Name of the DIPC queue (default: "RMQ_T_ROB1")

**Returns:**
- `str`: DIPC queue information as JSON string
- `int`: HTTP status code

---

### `read_dipc_message(queue_name="RMQ_T_ROB1", timeout=0)`
**Reads a message from the specified DIPC queue.**

```python
message, status = rws.read_dipc_message()
message, status = rws.read_dipc_message("MyQueue", 5)
```

**Parameters:**
- `queue_name` (str, optional): Name of the DIPC queue (default: "RMQ_T_ROB1")
- `timeout` (int, optional): Timeout in seconds (default: 0 = no timeout)

**Returns:**
- `dict`: Message data from the DIPC queue
- `int`: HTTP status code

**Raises:**
- `RWSException`: If queue name is empty, timeout is invalid, or message reading fails

---

## Usage Examples

### Basic Robot Status Check
```python
# Initialize RWS Interface
rws = RWSInterface(host="192.168.125.1", username="Default User", password="robotics")

# Check controller state
controller_state, status = rws.get_controller_state()
if status == 200:
    print(f"Controller state: {controller_state}")

# Get operational mode
op_mode, status = rws.get_opmode_state()
if status == 200:
    print(f"Operational mode: {op_mode}")
```

### Robot Position Monitoring
```python
# Get current cartesian position
cartesian, status = rws.get_robot_cartesian()
if status == 200:
    position_data = json.loads(cartesian)
    print(f"Robot position: {position_data}")

# Get joint positions
joints, status = rws.get_robot_jointtarget()
if status == 200:
    joint_data = json.loads(joints)
    print(f"Joint positions: {joint_data}")
```

### RAPID Symbol Access
```python
# Read a RAPID variable
value, status = rws.get_rapid_symbol("counter", "MainModule")
if status == 200:
    print(f"Counter value: {value}")

# Get symbol properties
properties, status = rws.get_rapid_symbol_properties("counter", "MainModule")
if status == 200:
    prop_data = json.loads(properties)
    print(f"Symbol properties: {prop_data}")
```

### IO Signal Monitoring
```python
# Read IO signal
signal_state, status = rws.get_io_signal("DO1")
if status == 200:
    signal_data = json.loads(signal_state)
    print(f"DO1 state: {signal_data}")
```

---

## Error Handling

All methods return an HTTP status code as the second element of the tuple. Common status codes:

- **200**: Success
- **400**: Bad Request
- **401**: Unauthorized
- **404**: Not Found
- **500**: Internal Server Error

Always check the status code before processing the returned data:

```python
data, status = rws.get_clock()
if status == 200:
    # Process data
    print(f"Clock: {data}")
else:
    # Handle error
    print(f"Error: HTTP {status}")
```

---

## Notes

1. **JSON Parsing**: Most methods return JSON strings. Use `json.loads()` to parse them into Python objects.
2. **ROS Compatibility**: All these methods return string or basic data types, making them suitable for ROS message communication.
3. **Error Handling**: Some methods may raise `RWSException` for invalid parameters.
4. **Default Parameters**: Many methods have default parameter values for common use cases.
5. **Network Requirements**: Ensure proper network connectivity to the robot controller before calling these methods.
