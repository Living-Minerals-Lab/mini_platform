import abc
import time
from tkinter import NO
import cv2
import pyautogui
import numpy as np
import matplotlib.pyplot as plt
import imutils

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
    
    def measure(self) -> None:
        """
        Take a measurement to collect data by locating the measure button on GUI, moving cursor there and left-clicking.
        """
        if not self.is_device_ready():
            raise Exception('Z300 is already measuring!')
        else:
            self.dev_status.value = 0 # 0: 'Measuring'
            # x, y = get_button_pos_multi_scale(self.measure_button_path)
            # pyautogui.click(x, y)
            self._process = Process(target=match_click_alter, args=(self.dev_status, self.measure_duration, self.measure_button_path, ))
            self._process.start()

            # self._status_process = Process(target=alter_device_status, args=(self.dev_status, self.measure_duration, ))
            # self._status_process.start() # alter self.dev_status back to 1: 'Idle' after self.duraton seconds.

    def is_device_ready(self) -> bool:
        """
            Check if the device is ready (in idle status).
        Returns:
            bool: true: ready; false: not ready. 
        """
        return self.dev_status.value == 1

def _get_button_pos(button_template_path: str) -> tuple[int]:
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

def get_button_pos_multi_scale(button_template_path: str) -> tuple[int]:
    """
    returns the (x, y) coordinates of the center 
    of the requested button in the screen window

    Args:
        button_template_path (str): path to the image file of the button template

    Returns:
        tuple[int]: the (x, y) coordinates of the center of the button
    """
    screenshot = np.array(pyautogui.screenshot())
    gray = cv2.cvtColor(screenshot, cv2.COLOR_RGB2GRAY)

    template = cv2.imread(button_template_path, cv2.IMREAD_GRAYSCALE) # return shape(height * width, y * x)
    template = cv2.Canny(template, 50, 200)
    (template_height, template_width) = template.shape[:2]

    found = None
    # loop over the scales of the image
    for scale in np.linspace(0.2, 2.0, 40)[::-1]:
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
        resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1])
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < template_height or resized.shape[1] < template_width:
            break
        
        # detect edges in the resized, grayscale image and apply template
        # matching to find the template in the image
        edged = cv2.Canny(resized, 50, 200)
        result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
        (_, max_val, _, max_loc) = cv2.minMaxLoc(result)
        # # check to see if the iteration should be visualized
        # if args.get("visualize", False):
        #     # draw a bounding box around the detected region
        #     clone = np.dstack([edged, edged, edged])
        #     cv2.rectangle(clone, (max_loc[0], max_loc[1]),
        #         (max_loc[0] + template_width, max_loc[1] + template_height), (0, 0, 255), 2)
        #     cv2.imshow("Visualize", clone)
        #     cv2.waitKey(0)
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        if found is None or max_val > found[0]:
            found = (max_val, max_loc, r)
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, max_loc, r) = found
    (start_x, start_y) = (int(max_loc[0] * r), int(max_loc[1] * r))
    (end_x, end_y) = (int((max_loc[0] + template_width) * r), int((max_loc[1] + template_height) * r))
    
    # draw a bounding box around the detected result and display the image
    # cv2.rectangle(screenshot, (start_x, start_y), (end_x, end_y), (0, 0, 255), 2)
    # cv2.imshow("Image", screenshot)
    # cv2.waitKey(0)

    return (start_x + end_x) // 2, (start_y + end_y) // 2
            
def match_click_alter(status: Value, wait_time: float, button_template_path: str) -> None: #type: ignore
    """
    Match template and left click on the found location.
    Alter status.value back to 1 after waiting for wait_time secs. 
    Args:
        status (Value): a multiprocessing.Value object.
        wait_time (float): seconds need to wait for. 
        button_template_path (str): path to the image file of the button template
    """

    x, y = get_button_pos_multi_scale(button_template_path=button_template_path)
    pyautogui.click(x, y)
    time.sleep(wait_time)
    status.value = 1
    
    return

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
        a._process.join()
        print(f'After meausre is done, device status is {a.dev_status.value}')
        print(f'Is device ready: {a.is_device_ready()}')

    