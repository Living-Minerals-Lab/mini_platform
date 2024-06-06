import socketio
import abc

class GantryController(metaclass=abc.ABCMeta):
    def __init__(self) -> None:
        pass
    
    @abc.abstractmethod
    def send_one_line_gcode(self):
        raise NotImplementedError('must define send_one_line_gcode to use this base class')

    def get_response(self):
        pass

    def send_whole_gcode(self):
        pass

class OpenBuildsGantryController(GantryController):
    def __init__(self) -> None:
        super().__init__()

        # self.client = socketio.Client()
        # self.connect()
        print('subclass')

    def send_one_line_gcode(self):
        return super().send_one_line_gcode()
    

if __name__ == '__main__':
    a = OpenBuildsGantryController()
    a.send_one_line_gcode()