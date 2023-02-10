# Importing all necessary libraries
import cv2
import os
import time
import sys

# Read the video from specified path
path = "/home/kota/Dropbox (MIT)/Research/MIT/RMADER/rmader_ral_videos"
video = "6agent_mesh"; start_time = 23; end_time = 30 
# video = "2agent1obs"; start_time = 56; end_time = 61 #70 - 75s
# video = "4agent2obsGPR"; start_time = 47; end_time = 54 #47 - 54s
ss_path = f"/home/kota/Research/rmader_ws/src/rmader/rmader/other/paper_graphs/screenshots/{video}"
if not os.path.exists(ss_path):
    os.mkdir(ss_path)
cam = cv2.VideoCapture(path+f"/{video}.MP4") 

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
frame_per_second = round(cam.get(cv2.CAP_PROP_FPS))
# screenshots frequency
ss_frq = 0.5 #[s]

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
                name = f'./screenshots/{video}/frame' + str(image_num) + '.png'
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