import socketio

sio = socketio.Client()

# @sio.event
# def connect():
#     print('connection established')

# @sio.event
# def my_message(data):
#     print('message received with ', data)
#     sio.emit('my response', {'response': 'my response'})

# @sio.event
# def disconnect():
#     print('disconnected from server')

sio.connect('http://192.168.137.214:3000')
# sio.connect('http://131.243.227.233:3000')
# sio.my_message()
print(sio.sid)
# sio.emit('maximize', {'range': '192.168.137.214'})
from socketio.exceptions import TimeoutError
try:  
    res = sio.call('runCommand', 'G00 X0 Y100 \n G04 P2 \n G00 X100 Y0 \n',
                   timeout=5)
except TimeoutError:
    print('timed out waiting for res')
else:
    print('received res:', res)
# sio.wait()
sio.disconnect()
