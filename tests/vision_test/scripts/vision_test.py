#!/usr/bin/env python

import cv2
import numpy as np

# image = cv2.imread('caixa_pequena.jpg')
image = cv2.imread('caixa_grande.jpg')
# image = cv2.imread('both_boxes.jpg')
# image = cv2.imread('half_big_box.jpg')
image = cv2.imread('bigBoxcut.jpg')
cv2.imshow('image', image)

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Blur image to remove noise
blur = cv2.GaussianBlur(gray, (5, 5), 0)

#um valor de 30 permite distinguir a cor preta da caixa
ret,th1 = cv2.threshold(blur,30,255,cv2.THRESH_BINARY)
cv2.imshow("binary", th1)

#ao aplicar o gradiente obtemos o contorno da caixa
ee = np.ones((3,3),np.uint8)
gradient = cv2.morphologyEx(th1, cv2.MORPH_GRADIENT, ee)
cv2.imshow("Gradient", gradient)

# Find contours in the image
contours, _ = cv2.findContours(gradient, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

areas = []
# Iterate over contours
for contour in contours:
    # Calculate the area of the contour
    areas.append(cv2.contourArea(contour))

print(f'Total area: {areas}')
print(f'Area size vector: {len(areas)}')
if(len(areas) == 0):
    print("Error! Could not find any object!")
elif(len(areas) > 1):
    print("Error! More than one object identified!")
else:
    # check area value to identify box size
    if(110000 <= areas[0] < 200000):
        print("Caixa Pequena!")

    elif(areas[0] >= 200000):
        print("Caixa grande!")

    else:
        print("Error")

cv2.waitKey(0)
cv2.destroyAllWindows()