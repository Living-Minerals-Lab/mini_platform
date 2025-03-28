import time
from utils.analytical_dev_ctrl import Z300Controller

def gen_gcode(x_interval: float, y_interval: float, x_points: int, y_points: int, z_safe: float) -> list[str]:
    """Generate a gcode script that visits positions on a grid.
    All units are millimeter.

    Args:
        x_interval (float): inverval between two adjacent positions on x-scale.
        y_interval (float): inverval between two adjacent positions on x-scale.
        x_points (int): number of positions on x-scale.
        y_points (int): number of positions on y-scale.
        z_safe (float): safety distance on z-scale used when traveling between positions.
    Returns:
        list[str]: generated gcode script.
    """
    gcode = ['G17 G21 G90' + '\n']


    x_pos, y_pos = [], []
 
    for i in range(x_points):
        x_pos.append(x_interval * i)

    for i in range(y_points):
        y_pos.append(y_interval * i)
    
    for x in x_pos:
        for y in y_pos:
            gcode.append(f'G00 Z{z_safe}' + '\n')
            gcode.append(f'G00 X{x} Y{y}' + '\n')
            gcode.append(f'G00 Z0.0' + '\n')

    gcode.append('M2')

def save():
    import socketio

    # standard Python
    sio = socketio.Client()

    sio.connect('http://localhost:3000')

    @sio.on('prbResult')
    def on_message(data):
        print(data)

    @sio.on('probe')
    def on_probe(data):
        print(data)

    while True:
        time.sleep(1)

if __name__ == '__main__':

    z = Z300Controller()
    z2 = Z300Controller()
    z.measure()
    print(z2.dev_status.value)