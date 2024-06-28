import abc
import time
from tkinter import NO
import cv2
import pyautogui
import numpy as np
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


class Z300Controller(AnalyticalDeviceController):
    """
    This class is the controller for LIBS Z300 gun from SciAps that inherits from the AnalyticalDeviceController base class.
    For Z300, a GUI-based automation approach is taken due to lack of APIs.
    """
    def __init__(self, measure_button_path: str='measure_button.png', measure_duration: float=10.0) -> None:
        """Constructor.

        Args:
            measure_button_path (str, optional): path to the image of the measure button on the device GUI. Defaults to 'measure_button.png'.
            duratoin (float, optional): The time (sec) that each measurement takes. This is a user-estimated value used to alter self.dev_status due to lack of APIs. Defaults to 10.0.
        """
        super().__init__()
        self.measure_button_path = measure_button_path
        
        self.measure_duration = measure_duration
        self.dev_status = Value('i', 1) # self.dev_status.value will be altered in a separate process, 1: 'Idle', 0: 'Measuring'.
        self.process = Process(target=alter_device_status, args=(self.dev_status, self.measure_duration))
    
    def measure(self) -> None:
        """
        Take a measurement to collect data by locating the measure button on GUI, moving cursor there and left-clicking.
        """
        if not self.is_device_ready():
            raise Exception('Z300 is already measuring!')
        else:
            self.dev_status.value = 0 # 0: 'Measuring'
            x, y = get_button_pos(self.measure_button_path)
            pyautogui.click(x, y)
            self.process.start() # alter self.dev_status back to 1: 'Idle' after self.duraton seconds.

    def is_device_ready(self) -> bool:
        """
            Check if the device is ready (in idle status).
        Returns:
            bool: true: ready; false: not ready. 
        """
        return self.dev_status.value == 1

def get_button_pos(button_template_path: str) -> tuple[int]:
    """
    returns the (x, y) coordinates of the center 
    of the requested button in the screen window

    Args:
        button_template_path (str): path to the image file of the button template

    Returns:
        tuple[int]: the (x, y) coordinates of the center of the button
    """
    screenshot = np.array(pyautogui.screenshot())
    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2GRAY)

    template = cv2.imread(button_template_path, cv2.IMREAD_GRAYSCALE) # return shape(height * width, y * x)

    res = cv2.matchTemplate(screenshot, template, cv2.TM_SQDIFF)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res) #min_loc is (width_pos, height_pos) or (x, y)

    x_center, y_center = min_loc[0] + template.shape[1] // 2, min_loc[1] + template.shape[0] // 2
    return x_center, y_center

def alter_device_status(status: Value, wait_time: float) -> None:
    """Alter status.value back to 1
    after waiting for wait_time secs. 

    Args:
        status (Value): a multiprocessing.Value object.
        wait_time (float): seconds need to wait for. 
    """
    time.sleep(wait_time)
    status.value = 1

if __name__ == '__main__':
    a = Z300Controller()
    print(f'Before meausre, device status is {a.dev_status.value}')
    print(f'Is device ready: {a.is_device_ready()}')

    a.measure()

    print(f'Just after meausre, device status is {a.dev_status.value}')
    print(f'Is device ready: {a.is_device_ready()}')

    try:
        a.measure()
    except Exception as e:
        raise e
    finally:
        a.process.join()

        print(f'After meausre is done, device status is {a.dev_status.value}')
        print(f'Is device ready: {a.is_device_ready()}')

    