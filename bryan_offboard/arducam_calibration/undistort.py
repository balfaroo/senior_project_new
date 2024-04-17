import cv2
import numpy as np

mtx = np.array([[9.976951689991706189e+02,0.000000000000000000e+00,3.706909466083776010e+02], [0.000000000000000000e+00,9.295534157327572302e+02,6.416561442172160241e+02],
[0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])

dist = np.array([7.048291962046168702e-02,-3.102217646794556694e-01,-1.758718385264990310e-03,-1.727333056427756298e-03,1.105015354520910265e+00])

img = cv2.imread('images/img_8.jpg')
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('images/calib_8.jpg', dst)