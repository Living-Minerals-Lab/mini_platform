import pyautogui

def move_lclick(x: int=0, y: int=0) -> None:
    """move the mouse to (x, y) and then left click.

    Args:
        x (int, optional): x coordinate of the destination. Defaults to 0.
        y (int, optional): y coordinate of the destination. Defaults to 0.
    """
    pyautogui.click(x, y)

if __name__ == '__main__':
    im = pyautogui.screenshot()
    im.show()
    # print(a)

