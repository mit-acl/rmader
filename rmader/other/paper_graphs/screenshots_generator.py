# Importing all necessary libraries
import cv2
import os
import time
import sys

# remove images in paper_graphs
os.system("rm screenshots/*")

# Read the video from specified path
cam = cv2.VideoCapture("/home/kota/Research/rmader_ws/src/rmader/rmader/videos/test11.mp4")

try:
    # creating a folder named data
    if not os.path.exists('screenshots'):
        os.makedirs('screenshots')

    # if not created then raise error
except OSError:
    print('Error: Creating directory of data')

# initialization
currentframe = 0
image_num = 0

# how many frames per second?
frame_per_second = cam.get(cv2.CAP_PROP_FPS)

# screenshots frequency
ss_frq = 0.5 #[s]
# start and end time
start_time = 31
end_time = 55


while (True):
    # reading from frame
    ret, frame = cam.read()
    # if it exceeds end time, terminate
    if currentframe >= end_time * frame_per_second:
        break

    if currentframe >= start_time * frame_per_second:
        if ret:
            # writing the extracted images
            if not currentframe%(ss_frq * frame_per_second):  
                name = './screenshots/frame' + str(image_num) + '.jpg'
                cv2.imwrite(name, frame)
                # if video is still left continue creating images
                print('Creating...' + name)
                image_num += 1
        else:
            break
    currentframe += 1

# Release all space and windows once done
cam.release()
cv2.destroyAllWindows()