import sys
import cv2
import time
import numpy as np
from pathlib import Path
from include.helpers import showImage, normaliseImage
from include.a_star import astar


def drawPath(map, start, end, path=[]):
    # Visualisation
    colormap = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

    thickness = max(1,int(np.floor(20 * map.shape[0] / 9000)))

    # Draw line
    for i in range(len(path) - 1):
        p_start = (path[i][1], path[i][0])
        p_end = (path[i+1][1], path[i+1][0])
        cv2.line(colormap, p_start, p_end, (255,0,0), thickness)

    cv2.circle(colormap, (start[1], start[0]), thickness*2, (0,0,255), -1)
    cv2.circle(colormap, (end[1], end[0]),   thickness*2, (0,255,0), -1)

    return colormap



def main(map_folder_name:str, startPos, endPos, isDebug=False, gui=False):

    mapsPath = (Path(__file__).parent.absolute() / map_folder_name)

    # Load costmaps
    print("Loading costmaps...")
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)
    
    # Normalise costmaps
    slopeMap = normaliseImage(slopes)
    illuminationMap = normaliseImage(illumination)

    # Show start and end points
    if gui:
        targetImage = drawPath(illumination, startPos, endPos)
        showImage("Output", targetImage)

    # map = np.floor(map * 255).astype(np.uint8)

    # Test algorithm
    print("Starting A* algorithm")
    # path = astar(costmap, illuminationMap, startPos, endPos, test=False)
    before = time.time()
    path_modified = astar(slopeMap, illuminationMap, startPos, endPos, modified=True, isDebug=True, gui=gui)
    print(f"-> A* finished running in {time.time() - before}s")


    # # print("Same" if path == path2 else "not same")
    # diff = 0
    # for i in range(min(len(path),len(path2))):
    #     diff += path[i][0] + path[i][1] - (path2[i][0] + path2[i][1])
    # print("Average difference", diff / min(len(path),len(path2)))

    # Show results
    outputImg = drawPath(illumination, startPos, endPos, path_modified)
    if gui:
        showImage("Output", outputImg)
        cv2.waitKey(0)

    # Save results
    outputImgPath = str(Path(__file__).parent.absolute() / "output" / "path.png")
    cv2.imwrite(outputImgPath, outputImg)

    outputDataPath = str(Path(__file__).parent.absolute() / "output" / "path.txt")
    np.savetxt(outputDataPath, path_modified)


if __name__ == "__main__":
    map_folder_name = sys.argv[1] if len(sys.argv) > 1 else "maps"
    
    # Start and end positions (Y,X)
    startPos = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 5 else (4200, 4400)
    endPos   = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (5200, 6000)
 
    # Run program
    main(map_folder_name, startPos, endPos, isDebug=True, gui=True)