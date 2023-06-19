import time

import numpy
import numpy as np
import cv2 as cv
import math
import timeit


#masking Parameter
lower = np.array([9, 100, 100])
upper = np.array([25, 255, 255])

#ED Parameter
ed = cv.ximgproc.createEdgeDrawing()
# you can change parameters (refer the documentation to see all parameters)
EDParams = cv.ximgproc_EdgeDrawing_Params()
EDParams.MinPathLength = 0  # try changing this value between 5 to 1000
EDParams.PFmode = False  # defaut value try to swich it to True
EDParams.MinLineLength = 100  # try changing this value between 5 to 100
EDParams.NFAValidation = True  # defaut value try to swich it to False
ed.setParams(EDParams)

def drawHLT (lines,img1):
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv.line(img1, pt1, pt2, (0, 255, 0), 3, cv.LINE_AA)
            values.append([rho, theta])
    return img1

def drawED (lines,img2) :
    if lines is not None: # Check if the lines have been found and only then iterate over these and add them to the image
            lines = np.uint16(np.around(lines))
            for i in range(len(lines)):
                cv.line(img2, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv.LINE_AA)
    return img2

def edlines(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    # output = cv.bitwise_and(img,img, mask= mask)
    blur = cv.GaussianBlur(mask, (5, 5), 0)
    ed.detectEdges(blur)
    return ed.detectLines()

def HLTProc(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    #cv.imshow("mask",mask)
    #output = cv.bitwise_and(img,img, mask= mask)
    blur = cv.GaussianBlur(mask, (5, 5), 0)
    #cv.imshow("blur", blur)
    edges = cv.Canny(blur, 10, 150)
    kernel = np.ones((30,30), np.uint8)
    img_erode = cv.erode(blur, kernel, iterations=1)
    img_dilation = cv.dilate(img_erode, kernel, iterations=1)
    #cv.imshow("dila", img_dilation)
    edges2 = cv.Canny(img_dilation,10,150)

    #cv.imshow("canny", edges)
    #cv.imshow("canny2", edges2)

    return cv.HoughLines(edges, 1, np.pi / 180, 90, None, 0, 0)

def sobelFilter(src):
    hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    scale = 1
    delta = 0
    ddepth = cv.CV_16S
    gray = cv.GaussianBlur(mask, (3, 3), 0)
    #gray = cv.cvtColor(src2, cv.COLOR_BGR2GRAY)
    grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
    grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
    abs_grad_x = cv.convertScaleAbs(grad_x)
    abs_grad_y = cv.convertScaleAbs(grad_y)
    return cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)

def LSD_process(img3):
    hsv = cv.cvtColor(img3, cv.COLOR_BGR2HSV)
    img3a = cv.inRange(hsv, lower, upper)
    #img3a = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)
    LSD = cv.createLineSegmentDetector()
    lines3, width, prec, nfa = LSD.detect(img3a)
    return lines3

def canny_process(img4):
    hsv = cv.cvtColor(img3, cv.COLOR_BGR2HSV)
    img4 = cv.inRange(hsv, lower, upper)
    src = cv.GaussianBlur(img4, (3, 3), 0)
    return cv.Canny(src, 10, 150)

values = []
img = cv.imread('21.jpg')
img1 = cv.imread('1.jpg')
img2 = cv.imread('1.jpg')
img3 = cv.imread('1.jpg')
img4 = cv.imread('1.jpg')

#starttimerHLT = time.time()
linesHLT = HLTProc(img)
#drawHLT(linesHLT,img)


#stoptimerHLT = (time.time())
#timerHLT = stoptimerHLT - starttimerHLT

"""
starttimerED = time.time()
linesED = edlines(img1)
stoptimerED = (time.time())
timerED = (stoptimerED - starttimerED)

starttimerSobel = time.time()
sobel_img = sobelFilter(img2)
stoptimerSobel = (time.time())
timerSOBEL = (stoptimerSobel - starttimerSobel)

starttimerLSD = time.time()
linesLSD = LSD_process(img3)
stoptimerLSD = (time.time())
timerLSD = (stoptimerLSD - starttimerLSD)

starttimerCanny = time.time()
canny_img = canny_process(img4)
timerCanny = time.time() - starttimerCanny

print("HLT Time      = ",timerHLT*1000)
print("ED Time       = ",timerED*1000)
print("LSD Time      = ",timerLSD*1000)
print("SOBEL XY Time = ",timerSOBEL*1000)
print("Canny Time    = ",timerCanny*1000)

if timerHLT>timerED:
    print("ED win")
else :print("HLT win")

LSD = cv.createLineSegmentDetector()
cv.imshow("HLT",drawHLT(linesHLT,img))
cv.imshow("ED",drawED(linesED,img1))
cv.imshow("sobel",sobel_img)
cv.imshow("LSD",LSD.drawSegments(img3,linesLSD))
cv.imshow("canny",canny_img)
"""
cv.imshow("ori",img)
cv.imshow("HLT",drawHLT(linesHLT,img))

cv.waitKey()