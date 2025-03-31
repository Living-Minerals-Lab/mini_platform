import abc
import time
from tkinter import NO
import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
import socketio

from multiprocessing import Process, Value

class AnalyticalDeviceController(metaclass=abc.ABCMeta):
    """abstract base class for any analytical devices
    such as LIBS Z300 and Horiba Raman
    """
    def __init__(self) -> None:
        pass
    
    @abc.abstractmethod
    def measure(self) -> None:
        """
        this function triggers a measurement using the device
        which obtains some data
        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses
        """
        raise NotImplementedError('must define measure to use this base class')

    @abc.abstractmethod
    def is_device_ready(self) -> bool:
        """
        this function returns a boolean indicating if the device is ready (in idle status)

        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses

        Returns:
            bool: true: ready; false: not ready
        """
        raise NotImplementedError('must define is_device_ready to use this base class')
    
    @abc.abstractmethod
    def export(self) -> None:
        """
        this function exports the data from the device
        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses
        """
        raise NotImplementedError('must define export to use this base class')
    
    @abc.abstractmethod
    def analyze(self) -> None:
        """
        this function analyzes the data from the device
        Raises:
            NotImplementedError: this is the abstract method in the base class and must be implemented separately for subclasses
        """
        raise NotImplementedError('must define analyze to use this base class')


class Z300Controller(AnalyticalDeviceController):
    """
    This class is the controller for LIBS Z300 gun from SciAps that inherits from the AnalyticalDeviceController base class.
    For Z300, a GUI-based automation approach is taken due to lack of APIs.
    """
    def __init__(self, z300_ctrl_addr: str) -> None:
        """Constructor.

        Args:
            z300_ctrl_addr: ip address and port of the openbuilds control socket.io server.
        """
        super().__init__()
        self.client = socketio.Client()
        self.client.on('connect', self.on_connect)
        self.client.on('connect_error', self.on_connect_error)
        self.client.on('disconnect', self.on_disconnect)
        self.client.on('status', self.on_status)
        
        self.dev_status = None
        self.res = {'measure': {'msg': None, 'val': None}, 
                    'export': {'msg': None, 'val': None}, 
                    'analyze': {'msg': None, 'val': None}}

        self.client.connect(z300_ctrl_addr)

    def on_connect(self) -> None:
        print('Connectted to Z300 server. SSID: {self.client.sid}.')
    
    def on_connect_error(self, data) -> None:
        print('Connection to Z300 server failed.')

    def on_disconnect(self) -> None:
        print('Disconnected from Z300 server.')

    def on_status(self, data) -> None:
        self.dev_status = data

    def measure(self) -> None:
        self.res['measure']['msg'] = None
        self.res['measure']['val'] = None
        return self.client.emit('measure', 'data', callback=self.on_measure)
    
    def export(self):
        self.res['export']['msg'] = None
        self.res['export']['val'] = None
        return self.client.emit('export', 'data', callback=self.on_export)
    
    def analyze(self):
        self.res['analyze']['msg'] = None
        self.res['analyze']['val'] = None
        return self.client.emit('analyze', 'data', callback=self.on_analyze)
    
    def on_measure(self, data):
        self.res['measure']['msg'] = data

    def on_export(self, data):
        self.res['export']['msg'] = data

    def on_analyze(self, data):
        self.res['analyze']['msg'] = data[0]
        self.res['analyze']['val'] = data[1]

    def is_device_ready(self) -> bool:
        """
            Check if the device is ready (in idle status).
        Returns:
            bool: true: ready; false: not ready. 
        """
        return self.dev_status == 'IDLE'

    