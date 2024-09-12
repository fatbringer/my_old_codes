import cv2
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-source", "--source", required=True, help="Path to video source")
ap.add_argument('--interval', type=int, default=100, help='number of frames between each interval')
args = vars(ap.parse_args())

inputfilename = args["source"]
frameInterval = args["interval"]

vidcap = cv2.VideoCapture(inputfilename)
count = 0
success = True
#fps = int(vidcap.get(cv2.CAP_PROP_FPS))
fps = 30 #assume videos are 30 fps
timeInterval = 5 # time between each screenshot, in seconds

while success:
    success,image = vidcap.read()
    #print('read a new frame:',success)
    if count%(frameInterval) == 0 : #you can replace the frame interval manually too
        outputFrameName = inputfilename + "_frame" + str(count) + ".jpg"
        print("writing", outputFrameName)
        cv2.imwrite(outputFrameName, image)
    count+=1


# When everything done, release the capture
vidcap.release()
cv2.destroyAllWindows()
print("completed")
