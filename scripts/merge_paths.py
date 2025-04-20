from pathlib import Path
import sys
from include.units import Unit
from include.helpers import drawPath, showImage, randomColor
from draw_units import drawPoints
import numpy as np
import cv2

def main(unitsJsonfile, mapFolder, outputFolder):
    # Paths
    jsonPath = str(Path(__file__).parent.absolute() / "units" / unitsJsonfile)
    mapPath  = Path(__file__).parent.absolute() / "maps" / mapFolder / "illumination.png"
    outputFolderPath = Path(__file__).parent.absolute() / "output" / outputFolder 

    targetImgSize = 2000

    # Load units data
    units, px2m, scale = Unit.loadUnits(jsonPath, targetImgSize)
    
    # Load map image
    map = cv2.imread(mapPath)
    initialSize = map.shape[0]
    map = cv2.resize(map, (targetImgSize,targetImgSize))

    
    # Reduce opacity
    map = np.floor(map * 0.8 + 255*0.2)
    map = map.astype(np.uint8)

    # Check all paths
    for unit in units:
        for other_unit in unit.routes:

            # Load path
            pathFile = str(outputFolderPath / f"route_{unit.id}-{other_unit.id}" / "path.txt")
            path = np.loadtxt(pathFile)

            # Path image
            pathFile = str(outputFolderPath / f"route_{unit.id}-{other_unit.id}" / "path.png")
            pathImg = cv2.imread(pathFile)
            astarScale = initialSize / pathImg.shape[0]

            # Change path coordinates
            path *= scale * astarScale
            path = path.astype(int)

            # Draw path
            map = drawPath(map, path, randomColor())

    # Draw points
    map = drawPoints(map, units, px2m, size=targetImgSize)

    # Save image
    outputPath = str(outputFolderPath / "merged_paths.png")
    cv2.imwrite(outputPath, map)

    showImage("Merged paths", map)
    cv2.waitKey(0)



if __name__ == "__main__":
    unitsFileName = sys.argv[1] if len(sys.argv) > 1 else "config1.json"
    mapFolder     = sys.argv[2] if len(sys.argv) > 2 else "cropped_resized"
    outputFolder  = sys.argv[3] if len(sys.argv) > 3 else "cropped_resized"

    main(unitsFileName,mapFolder,outputFolder)