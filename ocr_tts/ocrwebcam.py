import cv2 as cv
import numpy as np
import pytesseract

pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

cap = None
ret = None
frame = None
key = None


print('START')
cap = cv.VideoCapture(0)
while True:
  ret , frame = cap.read()
  cv.imshow('Window',frame)
  print(pytesseract.image_to_string(frame))
  key = cv.waitKey(1000)
  if key == (ord('q')):
    break
cv.destroyAllWindows()
cap.release()
print('END')
