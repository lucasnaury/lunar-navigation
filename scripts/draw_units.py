import json
import sys
import os
from pathlib import Path
import cv2
import numpy as np
from include.helpers import showImage, loadUnits


def drawPoints(map, units, px2m, size=None, showRange=False):

    if size is not None:
        map = cv2.resize(map, (size,size))



    # Define drawing parameters
    circleSize = max(1, int(10 * size / 2000))
    fontSize   = max(1, int(2  * size / 2000))


    for key,(x,y) in units.items():
        # Draw unit
        cv2.circle(map, (x,y), circleSize, (0,0,255), -1)

        # Draw unit landing range
        if showRange:
            overlay = map.copy()
            landingRange = 6000/px2m # 6km range in px
            cv2.circle(overlay, (x,y), int(landingRange/2), (0,255,0), -2)
            map = cv2.addWeighted(overlay, 0.2, map, 0.8, 0)

        # Draw unit name
        (w,h),b = cv2.getTextSize(key, cv2.FONT_HERSHEY_DUPLEX, fontSize, fontSize)
        y_offset = h if y - h > 0 else -(h+b)
        if x - int(w/2) < 0:
            new_x = 0
        elif x + int(w/2) > map.shape[1]:
            new_x = map.shape[1] - w
        else:
            new_x = x - int(w/2)
        cv2.putText(map, key, (new_x,y - y_offset), cv2.FONT_HERSHEY_DUPLEX, fontSize, (255,0,0), fontSize)

    return map

def main(jsonFile, mapPath):

    targetImgSize = 2000
    
    # Load units data
    path = str(Path(__file__).parent.absolute() / jsonFile)
    units, px2m,_ = loadUnits(path, targetImgSize)
    
    # Load map image
    map = cv2.imread(mapPath)

    # Draw units
    map = drawPoints(map, units, px2m, size=targetImgSize, showRange=True)


    # Compute min distance
    minDist = -1
    for key,(x,y) in units.items():
        for k,(other_x,other_y) in units.items():
            if k == key:
                break

            dist = np.linalg.norm((x - other_x, y - other_y))
            if dist < minDist or minDist == -1:
                minDist = dist

    minDist = minDist * px2m
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

    mapPath = str(Path(__file__).parent.absolute() / mapsFolderPath / "illumination.png")
    
    # Run program
    main(unitsFileName, mapPath)