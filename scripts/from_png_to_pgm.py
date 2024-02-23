import cv2 as cv
import os

linux_user = os.getlogin()
img = cv.imread(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/isaac_map.png", cv.IMREAD_GRAYSCALE)
cv.imwrite(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/isaac_map.pgm", img)
