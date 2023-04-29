import cv2 as cv
import numpy as np
import pytesseract
from gtts import gTTS

pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

cap = None
ret = None
frame = None
key = None

# next library Pre-recognition PRLib, leptonica

print('START')
cap = cv.VideoCapture(0)
language = 'id'

while True:
  ret , frame = cap.read()
  cv.imshow('Window',frame)
  
  #PRE PROCESSING
# =============================================================================
#   frame = cv.resize(frame, None, fx=1.2, fy=1.2, interpolation=cv.INTER_CUBIC)
#   frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#   kernel = np.ones((1, 1), np.uint8)
#   #frame = cv.dilate(frame, kernel, iterations=1)
#   #frame = cv.erode(frame, kernel, iterations=1)
# =============================================================================

# =============================================================================
#   cv.threshold(cv.GaussianBlur(frame, (5, 5), 0), 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)[1]
#   cv.threshold(cv.bilateralFilter(frame, 5, 75, 75), 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)[1]
#   cv.threshold(cv.medianBlur(frame, 3), 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)[1]
#   cv.adaptiveThreshold(cv.GaussianBlur(frame, (5, 5), 0), 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 31, 2)
#   cv.adaptiveThreshold(cv.bilateralFilter(frame, 9, 75, 75), 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 31, 2)
#   cv.adaptiveThreshold(cv.medianBlur(frame, 3), 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 31, 2)
# =============================================================================

  norm_frame = np.zeros((frame.shape[0], frame.shape[1]))
  frame = cv.normalize(frame, norm_frame, 0, 255, cv.NORM_MINMAX)
  frame = cv.threshold(frame, 100, 255, cv.THRESH_BINARY)[1]
  frame = cv.GaussianBlur(frame, (1, 1), 0)
  
  cv.imshow('Processed',frame)
  
  mask = np.zeros(frame.shape[:2], np.uint8)
  mask[75:500, 125:650] = 255
  
  masked_frame = cv.bitwise_and(frame,frame,mask = mask)
  cv.imshow('Masked',masked_frame)
  
  text = pytesseract.image_to_string(masked_frame,lang='eng',config='-c tessedit_char_whitelist=" ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"')
  print(text)

  textfile = open("fileoutput.txt", "a+")
  textfile.write(text)
  #textfile.close()
  
  textlist = textfile.readlines()
  if text != "":
      audiofile = gTTS(text=text, lang=language, slow=False)
      #audiofile.save("audiofile.mp3")
      with open('audiofile.mp3', 'ab') as f:
          audiofile.write_to_fp(f)
      #os.system("start audiofile.mp3")

  key = cv.waitKey(200)
  
  if key == (ord('q')):
    break
cv.destroyAllWindows()
cap.release()
print('END')
