import cv2
import numpy as np
from pathlib import Path
from include.helpers import showImage, normaliseImage
from include.a_star import astar


def drawPath(map, start, end, path):
    # Visualisation
    colormap = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

    thickness = int(np.floor(20 * 9000 / map.shape[0]))

    # Draw line
    for i in range(len(path) - 1):
        p_start = (path[i][1], path[i][0])
        p_end = (path[i+1][1], path[i+1][0])
        cv2.line(colormap, p_start, p_end, (255,0,0), thickness)

    cv2.circle(colormap, (start[1], start[0]), thickness*2, (0,0,255), -1)
    cv2.circle(colormap, (end[1], end[0]),   thickness*2, (0,255,0), -1)

    return colormap



def main():

    mapsPath = (Path(__file__).parent.absolute() / "maps")

    # Load costmaps
    print("Loading costmaps...")
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)
    
    # Normalise costmaps
    slopeMap = normaliseImage(slopes)
    illuminationMap = normaliseImage(illumination)

    # showImage("Costmap",costmap)

    # map = np.floor(map * 255).astype(np.uint8)

    # Set start and end positions
    startPos = (4200, 4400)   # (Y,X)
    endPos = (5200, 6000)     # (Y,X)


    # Test algorithm
    print("Starting A* algorithm")
    # path = astar(costmap, illuminationMap, startPos, endPos, test=False)
    path_modified = astar(slopeMap, illuminationMap, startPos, endPos, modified=True, isDebug=True)


    # # print("Same" if path == path2 else "not same")
    # diff = 0
    # for i in range(min(len(path),len(path2))):
    #     diff += path[i][0] + path[i][1] - (path2[i][0] + path2[i][1])
    # print("Average difference", diff / min(len(path),len(path2)))

    # Show results
    outputImg = drawPath(illumination, startPos, endPos, path_modified)
    # showImage("Output", outputImg)
    # cv2.waitKey(0)

    # Save results
    outputImgPath = str(Path(__file__).parent.absolute() / "output" / "path.png")
    cv2.imwrite(outputImgPath, outputImg)

    outputDataPath = str(Path(__file__).parent.absolute() / "output" / "path.txt")
    np.savetxt(outputDataPath, path_modified)


if __name__ == "__main__":
    main()