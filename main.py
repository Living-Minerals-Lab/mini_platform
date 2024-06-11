import argparse
import time
import pyautogui
import cv2
import numpy as np

# TODO: implement the following four functions. They might need to be associated with other classes.
def get_button_pos(button_template: str) -> tuple[int]:
    """
    returns the (x, y) coordinates of the center 
    of the requested button in the screen window

    Args:
        button_template (str): path to the image file of the button template

    Returns:
        tuple[int]: the (x, y) coordinates of the center of the button
    """
    screenshot = np.array(pyautogui.screenshot())
    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2GRAY)

    template = cv2.imread(button_template, cv2.IMREAD_GRAYSCALE) # return shape(height * width, y * x)

    res = cv2.matchTemplate(screenshot, template, cv2.TM_SQDIFF)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res) #min_loc is (width_pos, height_pos) or (x, y)

    x_center, y_center = min_loc[0] + template.shape[1] // 2, min_loc[1] + template.shape[0] // 2
    return x_center, y_center

def get_run_command_button(command_button_template: str='command_button.png') -> tuple[int]:
    """get the (x, y) coordinates of the center of the run command button on the screen.

    Args:
        command_button_template (str, optional): path to the run command button template image. 
        Defaults to 'command_button.png'.

    Returns:
        tuple[int]: the (x, y) coordinates of the center of the run command button on the screen. 
    """
    return get_button_pos(command_button_template)

def get_measure_button(measure_button_template: str='measure_button.png') -> tuple[int]:
    """get the (x, y) coordinates of the center of measurement button on the screen.

    Args:
        measure_button_template (str, optional): path to the measurement button template image. 
        Defaults to 'measure_button.png'.

    Returns:
        tuple[int]: the (x, y) coordinates of the center of measurement button on the screen.
    """
    return get_button_pos(measure_button_template)


def go_to_one_sample():
    x, y = get_run_command_button()
    pyautogui.click(x, y)

def go_to_sample_done():
    pass

def conduct_measure():
    x, y = get_measure_button()
    pyautogui.click(x, y)

def measure_done():
    pass


def main(n_samples):
    """
    Repeat the following steps until all samples are measured.
    1. Go to the position of a specific sample
    2. Conduct measurement
    """
    # TODO: currently the process is pepeated for a mixed number of times
    # TODO: need to change to unitl a done signal is received

    for i in range(n_samples):
        print(f'going to sample {i+1}.')
        go_to_one_sample()

        # while not go_to_sample_done():
        #     time.sleep(0.1)

        time.sleep(5)
        
        print(f'measuring sample {i+1}.')
        conduct_measure()

        # while not measure_done():
        #     time.sleep(0.1)
        time.sleep(15)

    print(f'{n_samples} samples measurement done.')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Start automatic measurements.')
    parser.add_argument('n_samples', type=int, help='number of samples to be measured')
    args = parser.parse_args()
    # print(args.accumulate(args.integers))

    main(args.n_samples)