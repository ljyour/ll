#!/home/sparkcharge2/miniforge3/bin/python3
import os
from PIL import Image
import cv2

if __name__ == '__main__':
    im = cv2.imread("bigroom.pgm")
    cv2.imshow("imname",im)
    print(im[20,20])
    cv2.waitKey()