# from os import wait
import cv2 as cv
import numpy as np
import pyautogui


def cvfoo():

    screenshot = cv.imread("grid.png", cv.IMREAD_GRAYSCALE) # return shape(height * width)
    template = cv.imread("command_button.png", cv.IMREAD_GRAYSCALE)

    res = cv.matchTemplate(screenshot, template, cv.TM_SQDIFF)

    # threshold  = 0.1
    # loc = np.where (res >= threshold)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res) #min_loc is (width_pos, height_pos)

    start_point = min_loc
    end_point = (min_loc[0] + template.shape[1], min_loc[1] + template.shape[0])
    color = 0  # Blue color
    thickness = 2

# Draw the rectangle on the image
    image = cv.rectangle(screenshot, start_point, end_point, color, thickness)
    cv.imshow('found button', image)
    cv.waitKey(0)
    # print(min_loc)


if __name__ == '__main__':


    import ttkbootstrap as ttk
    from ttkbootstrap.constants import *

    app = ttk.Window()

    frame = ttk.Frame(padding=5)
    frame.pack(padx=10, pady=10)

    b1 = ttk.Button(frame, text="Button 1", bootstyle="success")
    b1.pack(side=LEFT, padx=5, pady=10)

    b2 = ttk.Button(frame, text="Button 2", bootstyle="info-outline")
    b2.pack(side=LEFT, padx=5, pady=10)

    app.mainloop()