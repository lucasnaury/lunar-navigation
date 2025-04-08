import cv2
from pathlib import Path
import sys

def main(xStart, yStart, newSize):    
    mapsPath = Path(__file__).parent.absolute() / "maps" / "downloaded"

    # Load images
    height = cv2.imread(str(mapsPath / "heightmap.png"), cv2.IMREAD_GRAYSCALE)
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)


    # Resize images
    heightMap = height[yStart:yStart+newSize, xStart:xStart+newSize]
    slopeMap = slopes[yStart:yStart+newSize, xStart:xStart+newSize]
    illuminationMap = illumination[yStart:yStart+newSize, xStart:xStart+newSize]


    # Create output folder
    outputPath = Path(__file__).parent.absolute() / "maps" / "cropped"
    outputPath.mkdir(parents=True, exist_ok=True)

    # Save images
    cv2.imwrite(str(outputPath / "heightmap.png"), heightMap)
    cv2.imwrite(str(outputPath / "slopemap.png"), slopeMap)
    cv2.imwrite(str(outputPath / "illumination.png"), illuminationMap)




if __name__ == "__main__":
    xStart = int(sys.argv[1]) if len(sys.argv) > 2 else 0
    yStart = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    newSize = int(sys.argv[3]) if len(sys.argv) > 3 else 512

    main(xStart, yStart, newSize)