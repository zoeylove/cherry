#!/usr/bin/env python
import numbers as np 
import sys
print(sys.path)
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


folder_adress = "/home/tony/planercnn/test_image/"

if __name__ == "__main__":
    for i in range(7):
        src = cv2.imread(folder_adress+str(i+1)+".png",0)
        cv2.imshow("input",src)
        src_RGB = cv2.cvtColor(src,cv2.COLOR_GRAY2RGB)
        cv2.imshow("output",src_RGB)
        cv2.waitKey(0)
        # cv2.destroyWindow()
        cv2.imwrite(folder_adress+"/"+str(i)+".png",src_RGB)
    
