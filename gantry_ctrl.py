import socketio
import abc
import time
from threading import Timer

stats = False

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
    def get_response(self) -> bool:
        """
        this function gets response from grbl
        and checks if grbl is ready for the next line of gcode

        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses

        Returns:
            bool: true: grbl's status. true: ready; false: not ready
        """
        raise NotImplementedError('must define get_response to use this base class')

    def run_whole_gcode(self, gcode_whole: list[str]):
        """
        takes a complete gcode script as input,
        and executes it line by line following a execute-wait manner. 

        Args:
            gcode_whole (list[str]): list of lines of gcode to be executed.
        """
        for line in gcode_whole:
            print(f'Executing gcode: {line}')
            self.run_one_line_gcode(line)
            while not self.get_response():
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

        # self.client = socketio.Client()
        # self.connect()
        self.client = socketio.Client()
        self.connect_open_builds_ctrl(open_builds_ctrl_addr)

    def connect_open_builds_ctrl(self, open_builds_ctrl_addr: str) -> None:
        # self.client.connect(open_builds_ctrl_addr)
        print('connected')
        pass
    def disconnect_open_builds_ctrl(self) -> None:
        # self.client.disconnect()
        pass

    def run_one_line_gcode(self, gcode_line: str):
        # return super().send_one_line_gcode()
        # self.client.emit('runCommand', gcode_line)
        print(gcode_line)

    def get_response(self) -> bool:
        # return super().get_response()
        print(f'Current status: {stats}.')
        return stats

# def fake_status():
#     global stats 
#     stats = True

if __name__ == '__main__':
    a = OpenBuildsGantryController('123')
    t = Timer(10, fake_status)
    t.start()
    with open('gg', 'r') as f:
        gcode_whole = f.readlines()
        a.run_whole_gcode(gcode_whole)
    # a.send_one_line_gcode()