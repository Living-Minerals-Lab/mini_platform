from uu import Error
import socketio
import abc
import time

class GantryController(metaclass=abc.ABCMeta):
    def __init__(self) -> None:
        pass
    
    @abc.abstractmethod
    def run_one_line_gcode(self, line: str):
        """
        this function takes a line of gcode as input
        and send that gcode to grbl to execute it 

        Args:
            line (str): a line of gcode to be executed

        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses
        """
        raise NotImplementedError('must define send_one_line_gcode to use this base class')

    @abc.abstractmethod
    def gantry_ready(self) -> bool:
        """
        this function returns a boolean indicating if the gantry system is ready for the next line of gcode

        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses

        Returns:
            bool: true: ready; false: not ready
        """
        raise NotImplementedError('must define gantry_ready to use this base class')

    def run_whole_gcode(self, gcode_whole: list[str]):
        """
        takes a complete gcode script as input,
        and executes it line by line following a execute-wait manner. 

        Args:
            gcode_whole (list[str]): list of lines of gcode to be executed.
        """
        for line in gcode_whole:
            # print(f'Executing gcode: {line}')
            self.run_one_line_gcode(line)
            while not self.gantry_ready():
                time.sleep(0.1)

class OpenBuildsGantryController(GantryController):
    def __init__(self, open_builds_ctrl_addr: str) -> None:
        """
        This class talks to grbl through the websocket established by openbuilding control using socket.io.
        A socket.io client connectted to openbuilds control is created when an object is instantiated. 
        Then communications are done simply by using socket.io "emit" and "on" functions.   

        Args:
            open_builds_ctrl_addr (str): ip address and port of the openbuilds control socket.io server.
        """
        super().__init__()
        # self.node = node
        self.client = socketio.Client()
        self.client.on('connect', self.on_connect)
        self.client.on('connect_error', self.on_connect_error)
        self.client.on('disconnect', self.on_disconnect)

        # self.connect_open_builds_ctrl(open_builds_ctrl_addr)
        self.client.connect(open_builds_ctrl_addr)

        self.gantry_status = ''
        self.gantry_position = {'x': -1.0, 'y': -1.0, 'z': -1.0}
        self.client.on('status', self.on_status)

        print(self.gantry_status)
    
    def on_connect(self) -> None:
        # self.node.get_logger().info('succeed')
        print(f'Connectted to OpenBuilds Control. SSID: {self.client.sid}.')
    
    def on_connect_error(self, data) -> None:
        print(f'Connection to OpenBuilds Control failed.')
        # self.node.get_logger().info('Connection to OpenBuilds Control failed.')

    def on_disconnect(self) -> None:
        print(f'Disconnected from Openbuilds Control.')

    def on_status(self, data):
        """
        callback function for the "status" event emitted from the OpenBuilds Control server.
        Set self.status according to the response from the server.    
        Args:
            data (dict): contains the status of the gantry system. Format shows below.
            Gantry status is in data['comms']['runStatus']. Possible responses are ['Run', 'Running', 'Idle', 'Pending'].
            Gantry position is in data['machine']['position']['work']['x', 'y', 'z']
            laststatus = {
                "driver": {
                    "version": "1.0.229",
                    "ipaddress": "192.168.87.100",
                    "operatingsystem": "windows"
                },
                "machine": {
                    "name": "minimill",
                    "inputs": [],
                    "overrides": {
                        "feedOverride": 100,
                        "spindleOverride": 100,
                        "realFeed": 0,
                        "realSpindle": 0,
                        "rapidOverride": 100
                    },
                    "tool": {
                        "nexttool": {
                            "number": 10,
                            "line": "M6 T10\n"
                        }
                    },
                    "probe": {
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "state": -1,
                        "plate": 0,
                        "request": {}
                    },
                    "position": {
                        "work": {
                            "x": 13.17,
                            "y": 3.22,
                            "z": 2.65,
                            "a": 0,
                            "e": 0
                        },
                        "offset": {
                            "x": 22.25,
                            "y": -138.79,
                            "z": -49.29,
                            "a": 0,
                            "e": 0
                        }
                    },
                    "firmware": {
                        "type": "grbl",
                        "version": "1.1g",
                        "date": "",
                        "buffer": [
                            "15",
                            "128"
                        ],
                        "features": [
                            "V"
                        ],
                        "blockBufferSize": "15",
                        "rxBufferSize": "128"
                    }
                },
                "comms": {
                    "connectionStatus": 2,
                    "runStatus": "Idle",
                    "queue": 0,
                    "blocked": false,
                    "paused": false,
                    "controllerBuffer": 0,
                    "interfaces": {
                        "ports": [
                            {
                                "path": "COM3",
                                "manufacturer": "FTDI",
                                "serialNumber": "5&222e27aa&0&6",
                                "pnpId": "FTDIBUS\\VID_0403+PID_6001+5&222E27AA&0&6\\0000",
                                "vendorId": "0403",
                                "productId": "6001"
                            }
                        ],
                        "activePort": "COM3",
                        "activeBaud": 115200
                    },
                    "alarm": "13 - undefined"
                }
            }
        """
        self.gantry_status = data['comms']['runStatus']
        self.gantry_position['x'] = float(data['machine']['position']['work']['x'])
        self.gantry_position['y'] = float(data['machine']['position']['work']['y'])
        self.gantry_position['z'] = float(data['machine']['position']['work']['z'])

    def run_one_line_gcode(self, gcode_line: str) -> None:
        """Emit the line of gcode to be executed to "runCommand". 

        Args:
            gcode_line (str): a single line of gcode to be executed. 
        """
        if not self.gantry_ready():
            raise Exception('Gantry system is aleady running!')
        else:
            print(f'Executing gcode: {gcode_line}')
            self.client.emit('runCommand', gcode_line)

    def gantry_ready(self) -> bool:
        """
            Check if the gantry system is in 'Idle' status.
        Returns:
            bool: if the gantry system is ready for actions.
        """
        # print(f'Current status: {self.gantry_status}.')
        return self.gantry_status == 'Idle'

    def set_zero(self, axis: str) -> None:
        """Emit the axis of the gantry to be set to zero to "setZero". 

        Args:
            axis (str): the axis of the gantry to be set to zero, in ['x', 'y', 'z']
        """
        self.client.emit('setZero', axis)


if __name__ == '__main__':
    a = OpenBuildsGantryController('http://128.3.118.197:3000')
    # a.send_one_line_gcode()