import cv2
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())
filename =  args["image"]

image = cv2.imread(filename, 0)

params = cv2.SimpleBlobDetector_Params()

params.filterByArea = True
params.minArea = 2

params.filterByCircularity = True
params.minCircularity = 0.1

params.filterByConvexity = True
params.minConvexity = 0.2

params.filterByInertia = True
params.minInertiaRatio = 0.01

detector = cv2.SimpleBlobDetector_create(params)
keypoints = detector.detect(image)

blank = np.zeros((1, 1))
blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
number_of_blobs = len(keypoints)
text = "Circular Blobs: " + str(len(keypoints))
cv2.putText(blobs, text, (10, 100),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

cv2.imshow("Blobs Using Area", blobs)

cv2.imwrite("{0}_{1}.jpg".format(filename,text), blobs)


cv2.waitKey(0)
cv2.destroyAllWindows()
