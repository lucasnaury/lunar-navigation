import sys
from pathlib import Path
import cv2
from include.helpers import showImage


# ---------------------------------------------------------------- #
#                                MAIN
# ---------------------------------------------------------------- #
def main(path):
    
    # Load colourd slope map
    slopeMap = cv2.imread(path)

    # Remove noise
    slopeMap = cv2.medianBlur(slopeMap, 5)

    # Only extract the Hue channel, which represents the variation in color and thus slope
    hsv_slopemap = cv2.cvtColor(slopeMap,cv2.COLOR_BGR2HSV)
    (H, _, _) = cv2.split(hsv_slopemap)

    # Invert slope map
    slopeMap = 255 - H

    # Save image
    outputPath = str(Path(__file__).parent.absolute() / "maps" / "downloaded" / "slopemap.png")
    cv2.imwrite(outputPath, slopeMap)

    # Show image
    showImage("Slope map normalised", slopeMap, 500)
    cv2.waitKey(0)








if __name__ == "__main__":

    default_path = str(Path(__file__).parent.absolute() / "maps" / "downloaded" / "slopemap_color.png")

    # Check if STL path is given as arg
    filepath = sys.argv[1] if len(sys.argv) > 1 else default_path
    
    # Run program
    main(filepath)