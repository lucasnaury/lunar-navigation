import sys
import os
from pathlib import Path
import numpy as np
import cv2
from include.helpers import showImage



# ---------------------------------------------------------------- #
#                          INDIVIDUAL MAPS
# ---------------------------------------------------------------- #
def getSlopeMap(heightmap, method):
    

    if method == 0: # My with diagonal weights

        # Create kernel
        diag = 1/np.sqrt(2)
        sum = 1*4 + diag*4
        kernel = np.array([[-diag, -1, -diag],
                           [-1,    sum,   -1],
                           [-diag, -1, -diag]]) / sum
        
        # Apply kernel
        slopeMap = cv2.filter2D(heightmap, -1, kernel)
        slopeMap = np.absolute(slopeMap)
        
    elif method == 1: # My test without diagonal weights
        # Create kernel
        kernel = np.array([[-1, -1, -1],
                           [-1,  8, -1],
                           [-1, -1, -1]]) / 8
        
        # Apply kernel
        slopeMap = cv2.filter2D(heightmap, -1, kernel)
        slopeMap = np.absolute(slopeMap)

    elif method == 2: # NUMPY GRADIENT
        slopeMap = np.gradient(heightmap,edge_order=1)
        slopeMap = np.absolute(slopeMap)
        slopeMap = cv2.addWeighted(slopeMap[0], 0.5, slopeMap[1], 0.5, 0)

    elif method == 3: # TEST cv2 SOBEL
        gX = cv2.Sobel(heightmap, cv2.CV_64F, dx=1, dy=0, ksize=5)
        gY = cv2.Sobel(heightmap, cv2.CV_64F, dx=0, dy=1, ksize=5)

        gX = cv2.convertScaleAbs(gX)
        gY = cv2.convertScaleAbs(gY)
        # combine the gradient representations into a single image
        slopeMap = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)


    # Blur slope map
    slopeMap = cv2.blur(slopeMap,(15,15))

    # Normalise slope map
    slopeMap = (slopeMap - np.min(slopeMap)) / (np.max(slopeMap) - np.min(slopeMap))

    # Threshold
    slopeMap = np.where(slopeMap < 0.4, 0, slopeMap)
    
    return slopeMap


def getIlluminationMap():
    
    # Get images folder
    path = str(Path().parent.absolute() / "illumination")

    # Filter images
    files = [os.path.join(path, f) for f in os.listdir(path) if f[-5:]== "0.png"]


    # Get image size
    size = cv2.imread(files[0]).shape
    print(size)

    # Initialise illumination map
    illuminationMap = np.zeros(size[0:2])


    for f in files:

        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)

        illuminationMap += img

    return illuminationMap / len(files)



# ---------------------------------------------------------------- #
#                                MAIN
# ---------------------------------------------------------------- #
def main(path, elevationLow, elevationHigh):
    # Import moon surface heightmap
    heightmap = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    print("Heightmap dim:", heightmap.shape)

    # Set 0 height to for unknown data
    # heightmap = np.where(heightmap > 253, 0, heightmap)

    showImage("Heightmap", heightmap)

    heightmap = heightmap * (elevationHigh - elevationLow) / 255.0 + elevationLow
    # heightmap = cv2.blur(heightmap,(3,3))
    # heightmap = heightmap.astype(np.float64) / 255.0
    
    
    # -> Slope map
    slopeMap = cv2.imread("slopemap_screen.png")

    hsv_slopemap = cv2.cvtColor(slopeMap,cv2.COLOR_BGR2HSV)
    (H, _, _) = cv2.split(hsv_slopemap)

    slopeMap_v1 = 255 - H
    slopeMap = (slopeMap_v1 - np.min(slopeMap_v1)) / (np.max(slopeMap_v1) - np.min(slopeMap_v1))
    showImage("Slope map normalised", slopeMap, 300)


    cv2.waitKey(0)








if __name__ == "__main__":
    # Check if STL path is given as arg
    filepath = sys.argv[1] if len(sys.argv) > 1 else "heightmap_screen.png"
    elevationLow = sys.argv[2] if len(sys.argv) > 2 else -8225
    elevationHigh = sys.argv[3] if len(sys.argv) > 3 else 3700
    
    # Run program
    main(filepath, elevationLow, elevationHigh)