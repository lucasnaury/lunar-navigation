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
    
    diag = 1/np.sqrt(2)
    sum = 1*4 + diag*4
    kernel = np.array([[-diag, -1, -diag],
                       [-1,    sum,   -1],
                       [-diag, -1, -diag]]) / sum
    
    # kernel = np.array([[0, -1, 0],
    #                 [-1, 1, -1],
    #                 [0, -1, 0]])


    slopeMap = None
    if method == 0: # My test
        slopeMap = cv2.filter2D(heightmap, -1, kernel)
        slopeMap = np.absolute(slopeMap)

    elif method == 1: # NUMPY GRADIENT
        slopeMap = np.gradient(heightmap,edge_order=1)
        slopeMap = np.absolute(slopeMap)
        slopeMap = cv2.addWeighted(slopeMap[0], 0.5, slopeMap[1], 0.5, 0)

    elif method == 2: # TEST cv2 SOBEL
        gX = cv2.Sobel(heightmap, cv2.CV_64F, dx=1, dy=0, ksize=5)
        gY = cv2.Sobel(heightmap, cv2.CV_64F, dx=0, dy=1, ksize=5)

        gX = cv2.convertScaleAbs(gX)
        gY = cv2.convertScaleAbs(gY)
        # combine the gradient representations into a single image
        slopeMap = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)


    # slopeMap = cv2.GaussianBlur(slopeMap,(21,21),5)
    slopeMap = cv2.blur(slopeMap,(15,15))


    print(slopeMap)
    print(slopeMap.shape)

    

    
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
    
    # Initialise costmap
    costMap = np.zeros(heightmap.shape)
    

    # -> Slope map
    slopeMapsNormalised = np.zeros((3,heightmap.shape[0],heightmap.shape[1]))
    for i in range(3):
        slopeMap = getSlopeMap(heightmap,i)

        slopeMapsNormalised[i] = (slopeMap - np.min(slopeMap)) / (np.max(slopeMap) - np.min(slopeMap))

        # print("Slope map: ",slopeMap)
        # showImage("Slope map", slopeMap)
        # print("Normalised map: ",slopeNormalised)
        showImage(f"Slope map {i} normalised", slopeMapsNormalised[i])

    # # -> Illumination map
    # illuminationMap = getIlluminationMap()
    # showImage("Illumination map", illuminationMap)



    cv2.waitKey(0)








if __name__ == "__main__":
    # Check if STL path is given as arg
    filepath = sys.argv[1] if len(sys.argv) > 1 else "heightmap_screen.png"
    elevationLow = sys.argv[2] if len(sys.argv) > 2 else -8225
    elevationHigh = sys.argv[3] if len(sys.argv) > 3 else 3700
    
    # Run program
    main(filepath, elevationLow, elevationHigh)