import numpy as np
import cv2
import matplotlib.pyplot as plt

min_perimeter = 1
epsilon = 1
img = cv2.imread('IGVCmap_thresholded_flipped.png')
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 230, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(
    thresh,
    cv2.RETR_TREE,
    cv2.CHAIN_APPROX_SIMPLE
)

# open a file to print to
f = open('new_test.dat', 'w')
for i in xrange(len(contours)):
    cnt = contours[i]
    minlength = min(epsilon, 0.3*cv2.arcLength(cnt, True))
    # apx = cv2.approxPolyDP(cnt, minlength, True)
    # if (cv2.arcLength(apx, True) < min_perimeter):
        # continue
    # if (len(apx) < 2):
    #     continue
    f.write("%i\n" % ((len(cnt)+1)))
    for ii in xrange(len(cnt)):
        print ii
        # f.write("%2.0f %2.0f \n" %(contours[i][ii][0][0], contours[i][ii][0][1]))
        f.write("%f, %f\n" % (cnt[ii][0][0],
                                   cnt[ii][0][1]))
    f.write("%f, %f\n" % (cnt[0][0][0],
                                   cnt[0][0][1]))

plt.imshow(thresh, interpolation="nearest")
plt.show()
