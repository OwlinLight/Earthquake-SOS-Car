# -*- coding: utf-8 -*-

from time import sleep
from picamera import PiCamera


def capture_images():
    """
    不断捕获图像
    :return:
    """
    with PiCamera() as camera:
        camera.resolution = (320, 240)
        sleep(2)

        num = 0
        while True:
            camera.capture(str(num) + ".jpg")

            print(num)
            num += 1

if __name__ == '__main__':
    capture_images()
