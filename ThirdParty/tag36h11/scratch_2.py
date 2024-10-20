
#coding:utf-8
import cv2
import numpy as np
import glob as gb
import os

#对img_path目录下所有图像进行操作，处理后文件保存在img_ savepath目录下
img_path = gb.glob("/Users/111938/Desktop/file/*.png")
img_savepath = "/Users/111938/Desktop/file/ROIS"

if __name__ == '__main__':
    for path in img_path:
        #分离文件目录，文件名及文件后缀、
        (img_dir, tempfilename) = os.path.split(path)
        img = cv2.imread(path)
       #对图像进行处理
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # 截取图片中一块宽和高都是250的
        region = gray[68:748, 68:748]
        ret, binary = cv2.threshold(region, 127, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        cv2.imshow("ROIS", binary)
       # savepath为处理后文件保存的全路径
        savepath = os.path.join(img_savepath, tempfilename)
        cv2.imwrite(savepath, binary)
        cv2.waitKey(100)
