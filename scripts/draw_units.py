import json
import sys
import os
from pathlib import Path
import cv2
import numpy as np
from include.helpers import showImage, loadUnits

def main(jsonFile, mapPath, px2m):

    # Load map image
    map = cv2.imread(mapPath)
    scale = 2000 / map.shape[0]
    map = cv2.resize(map, (2000,2000))

    # Load units data
    path = str(Path(__file__).parent.absolute() / jsonFile)
    units = loadUnits(path, scale)

    print("---------------------------- UNITS ----------------------------")

    minDist = -1
    for key,(x,y) in units.items():

        if x < 0 or y < 0 or x > map.shape[1] or y > map.shape[0]:
            print(f"[WARNING] Unit {key} is out of the map: ({x},{y}) for {(map.shape[1], map.shape[0])} sized map")
            continue


        # Check min distance
        for k,(other_x,other_y) in units.items():
            if k == key:
                break

            dist = np.linalg.norm((x - other_x, y - other_y))
            if dist < minDist or minDist == -1:
                minDist = dist

        # Draw unit
        cv2.circle(map, (x,y), 10, (0,0,255), -1)

        # Draw unit landing range
        overlay = map.copy()
        landingRange = 6000*scale/px2m # 6km range in px
        cv2.circle(overlay, (x,y), int(landingRange/2), (0,255,0), -2)
        map = cv2.addWeighted(overlay, 0.2, map, 0.8, 0)

        # Draw unit name
        (w,h),b = cv2.getTextSize(key, cv2.FONT_HERSHEY_DUPLEX, 2, 2)
        y_offset = h if y - h > 0 else -(h+b)
        if x - int(w/2) < 0:
            new_x = 0
        elif x + int(w/2) > map.shape[1]:
            new_x = map.shape[1] - w
        else:
            new_x = x - int(w/2)
        cv2.putText(map, key, (new_x,y - y_offset), cv2.FONT_HERSHEY_DUPLEX, 2, (255,0,0), 2)

    print("---------------------------------------------------------------")


    # Show min distance
    minDist = minDist / scale * px2m
    print(f"-> Minimum distance between 2 units: {minDist:0.2f}m")


    # Save image
    imgPath = str(Path(__file__).parent.absolute() / "output" / "units_map.png")
    cv2.imwrite(imgPath, map)
    
    # Show image
    showImage("Map", map)
    cv2.waitKey(0)



if __name__ == "__main__":
    # Load parameters
    unitsFileName = sys.argv[1] if len(sys.argv) > 1 else "units.json"
    mapsFolderPath = sys.argv[2] if len(sys.argv) > 2 else os.path.join("maps","cropped")
    px2m = sys.argv[2] if len(sys.argv) > 2 else 16.7269519188

    mapPath = str(Path(__file__).parent.absolute() / mapsFolderPath / "illumination.png")
    
    # Run program
    main(unitsFileName, mapPath, px2m)