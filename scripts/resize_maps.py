import cv2
from pathlib import Path
import sys

def main(newSize):    
    mapsPath = Path(__file__).parent.absolute() / "maps"

    # Load images
    height = cv2.imread(str(mapsPath / "heightmap.png"), cv2.IMREAD_GRAYSCALE)
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)


    # Resize images
    heightMap = cv2.resize(height, (newSize,newSize))
    slopeMap = cv2.resize(slopes, (newSize,newSize))
    illuminationMap = cv2.resize(illumination, (newSize,newSize))

    # Save images
    outputPath = Path(__file__).parent.absolute() / "test_maps"
    cv2.imwrite(str(outputPath / "heightmap.png"), heightMap)
    cv2.imwrite(str(outputPath / "slopemap.png"), slopeMap)
    cv2.imwrite(str(outputPath / "illumination.png"), illuminationMap)




if __name__ == "__main__":
    newSize = int(sys.argv[1]) if len(sys.argv) > 1 else 256
    main(newSize)