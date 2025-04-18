import json
import sys
import os
from pathlib import Path
import cv2
import numpy as np
from include.helpers import showImage
from include.units import Unit


def drawPoints(map, units:list[Unit], px2m, size=None, showRange=False, showText=False):

    if size is not None:
        map = cv2.resize(map, (size,size))



    # Define drawing parameters
    circleSize = max(1, int(10 * size / 2000))
    fontSize   = max(1, int(3  * size / 2000))


    for unit in units:
        # Draw unit
        cv2.circle(map, unit.pos(), circleSize, (0,0,255), -1)

        # Draw unit landing range
        if showRange and unit.isFixed:
            overlay = map.copy()
            landingRange = 2000/px2m # 2km range in px
            cv2.circle(overlay, unit.pos(), int(landingRange/2), (0,255,0), -2)
            map = cv2.addWeighted(overlay, 0.2, map, 0.8, 0)

        if showText:
            # Draw unit name
            (w,h),b = cv2.getTextSize(unit.name, cv2.FONT_HERSHEY_DUPLEX, fontSize, fontSize)
            y_offset = h if unit.y - h > 0 else -(h+b)
            if unit.x - int(w/2) < 0:
                new_x = 0
            elif unit.x + int(w/2) > map.shape[1]:
                new_x = map.shape[1] - w
            else:
                new_x = unit.x - int(w/2)
            cv2.putText(map, unit.name, (new_x, unit.y - y_offset), cv2.FONT_HERSHEY_DUPLEX, fontSize, (255,0,0), fontSize)

    return map

def main(jsonFile:str, mapPath:str):

    targetImgSize = 2000
    
    # Load units data
    path = str(Path(__file__).parent.absolute() / "units" / jsonFile)
    units, px2m,_ = Unit.loadUnits(path, targetImgSize)
    
    # Load map image
    map = cv2.imread(mapPath)

    # Draw units
    map = drawPoints(map, units, px2m, size=targetImgSize, showRange=True,showText=True)

    # Draw routes
    # for unit in units:
    #     for r in unit.routes:
    #         cv2.line(map,unit.pos(),r.pos(), (0,0,255),5)

    # Compute min distance
    minDist = -1
    fixedUnits = [u for u in units if u.isFixed]
    for unit in fixedUnits:
        for other_unit in fixedUnits:
            if unit.id == other_unit.id:
                break

            dist = np.linalg.norm((unit.x - other_unit.x, unit.y - other_unit.y))
            if dist < minDist or minDist == -1:
                minDist = dist

    minDist = minDist * px2m
    print(f"-> Minimum distance between 2 fixed units: {minDist:0.2f}m")


    # Save image
    imgPath = str(Path(__file__).parent.absolute() / "output" / f"units_map_{jsonFile.replace('.json','')}.png")
    cv2.imwrite(imgPath, map)
    
    # Show image
    showImage("Map", map)
    cv2.waitKey(0)



if __name__ == "__main__":
    # Load parameters
    unitsFileName = sys.argv[1] if len(sys.argv) > 1 else "config1.json"
    mapsFolderPath = sys.argv[2] if len(sys.argv) > 2 else os.path.join("maps","cropped")

    mapPath = str(Path(__file__).parent.absolute() / mapsFolderPath / "illumination.png")
    
    # Run program
    main(unitsFileName, mapPath)