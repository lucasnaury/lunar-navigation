import cv2
import os
from pathlib import Path
import sys

def main(inputFolder, outputFolder, newSize):    
    mapsPath = Path(__file__).parent.absolute() / "maps" / inputFolder

    # Load images
    height = cv2.imread(str(mapsPath / "heightmap.png"), cv2.IMREAD_GRAYSCALE)
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)


    # Resize images
    heightMap = cv2.resize(height, (newSize,newSize))
    slopeMap = cv2.resize(slopes, (newSize,newSize))
    illuminationMap = cv2.resize(illumination, (newSize,newSize))

    # Create output folder
    outputPath = Path(__file__).parent.absolute() / "maps" / outputFolder
    outputPath.mkdir(parents=True, exist_ok=True)

    # Save images
    cv2.imwrite(str(outputPath / "heightmap.png"), heightMap)
    cv2.imwrite(str(outputPath / "slopemap.png"), slopeMap)
    cv2.imwrite(str(outputPath / "illumination.png"), illuminationMap)




if __name__ == "__main__":
    inputFolder  = sys.argv[1] if len(sys.argv) > 1 else "downloaded"
    outputFolder = sys.argv[2] if len(sys.argv) > 2 else "resized"
    newSize = int(sys.argv[3]) if len(sys.argv) > 3 else 256

    main(inputFolder, outputFolder, newSize)