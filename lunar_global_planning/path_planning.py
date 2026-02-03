import sys
import os
import cv2
import time
import numpy as np
from pathlib import Path
from include.units import Unit
from include.helpers import showImage, normaliseImage, writeToFile, drawPath
from include.a_star import astar


# -------------------------------------------------------------------------
#                                  HELPERS 
# -------------------------------------------------------------------------

def loadMaps(map_folder_name):
    # Get maps path
    mapsPath = Path(__file__).parent.absolute() / map_folder_name
    
    # Load costmaps
    print("Loading costmaps...")
    slopes = cv2.imread(str(mapsPath / "slopemap.png"), cv2.IMREAD_GRAYSCALE)
    illumination = cv2.imread(str(mapsPath / "illumination.png"), cv2.IMREAD_GRAYSCALE)
    
    # Normalise costmaps
    slopeMap = normaliseImage(slopes)
    illuminationMap = normaliseImage(illumination)

    return slopes, slopeMap, illumination, illuminationMap


def drawWaypointsAndPath(map, start, end, path=None):
    # Visualisation
    colormap = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

    thickness = max(1,int(np.floor(20 * map.shape[0] / 9000)))
    
    # Draw waypoints
    cv2.circle(colormap, (start[1], start[0]), thickness*2, (0,0,255), -1)
    cv2.circle(colormap, (end[1], end[0]),   thickness*2, (0,255,0), -1)

    # Draw path
    if path is not None:
        drawPath(colormap,path)

    return colormap







# -------------------------------------------------------------------------
#                                  MAIN 
# -------------------------------------------------------------------------

def debug(map_folder_name:str, output_folder_name:str, startPos, endPos):
    """To test specific weights, and debug algorithm"""

    # Load maps
    slopes, slopeMap, illumination, illuminationMap = loadMaps(map_folder_name)

    # Show start and end points
    slopePathImage = drawPath(slopes, startPos, endPos)
    showImage("Slope output", slopePathImage)

    illuminationPathImage = drawPath(illumination, startPos, endPos)
    showImage("Output", illuminationPathImage)


    weights = [
        2.0,    # Distance weight
        1.0,    # Steering weight
        10.0,   # Slope weight
        1.0,    # Illumination weight

        0.8,    # Traveled distance weight
        0.2     # Heuristic distance weight 
    ]


    # Test algorithm
    print("Starting A* algorithm")
    before = time.time()
    path_modified, explored = astar(slopeMap, illuminationMap, startPos, endPos, weights, isDebug=True, gui=True)
    print(f"-> A* finished running in {time.time() - before}s")





    # Show results
    slopePathImage = drawWaypointsAndPath(slopes, startPos, endPos, path_modified)
    illuminationPathImage = drawWaypointsAndPath(illumination, startPos, endPos, path_modified)
    
    showImage("Slope output", slopePathImage)
    showImage("Output", illuminationPathImage)
    cv2.waitKey(0)

    
    # Check that result folder exists or create it
    outputFolder = Path(__file__).parent.absolute() / "output" / output_folder_name
    outputFolder.mkdir(parents=True, exist_ok=True)

    # Save results
    outputImgPath = str(outputFolder / "path.png")
    cv2.imwrite(outputImgPath, illuminationPathImage)

    outputImgPath = str(outputFolder / "slope_path.png")
    cv2.imwrite(outputImgPath, slopePathImage)

    outputDataPath = str(outputFolder / "path.txt")
    np.savetxt(outputDataPath, path_modified)



def hpc_weights(map_folder_name:str, output_folder_name:str, startPos, endPos):
    """To run on the HPC to compare all configurations of A* weights"""

    # Load maps
    slopes, slopeMap, illumination, illuminationMap = loadMaps(map_folder_name)

    # Compared weights
    weights_list = [
        # Distance, steering, slope, illumination, G,   H
        # High G cost
        [2.0,       1.0,      15.0,  1.0,         0.8, 0.2], # Very high slope cost
        [2.0,       1.0,      10.0,  1.0,         0.8, 0.2], # High slope cost
        [2.0,       1.0,      5.0,   1.0,         0.8, 0.2], # Medium slope cost
        [2.0,       1.0,      5.0,   5.0,         0.8, 0.2], # Medium slope cost, medium illumination cost
        [5.0,       1.0,      10.0,  1.0,         0.8, 0.2], # High slope cost, medium distance cost
        # Medium G Cost
        [2.0,       1.0,      15.0,  1.0,         0.65, 0.35], # Very high slope cost
        [2.0,       1.0,      10.0,  1.0,         0.65, 0.35], # High slope cost
        [2.0,       1.0,      5.0,   1.0,         0.65, 0.35], # Medium slope cost
        [2.0,       1.0,      5.0,   5.0,         0.65, 0.35], # Medium slope cost, medium illumination cost
        [5.0,       1.0,      10.0,  1.0,         0.65, 0.35], # High slope cost, medium distance cost
    ]

    for i,weights in enumerate(weights_list):

        # Test algorithm
        print(f"Starting A* algorithm with weights N°{i}: {weights}")
        before = time.time()
        path_modified, explored = astar(slopeMap, illuminationMap, startPos, endPos, weights, isDebug=True, gui=False)
        print(f"-> A* finished running in {time.time() - before}s")

        if path_modified is None:
            continue

        # Draw results
        slopePathImage = drawWaypointsAndPath(slopes, startPos, endPos, path_modified)
        illuminationPathImage = drawWaypointsAndPath(illumination, startPos, endPos, path_modified)



        # Check that result folder exists or create it
        outputFolder = Path(__file__).parent.absolute() / "output" / output_folder_name / f"weights_{i}"
        outputFolder.mkdir(parents=True, exist_ok=True)

        # Save results
        outputImgPath = str(outputFolder / "path.png")
        cv2.imwrite(outputImgPath, illuminationPathImage)

        outputImgPath = str(outputFolder / "slope_path.png")
        cv2.imwrite(outputImgPath, slopePathImage)

        if explored is not None:
            outputImgPath = str(outputFolder / "explored.png")
            cv2.imwrite(outputImgPath, explored)
        

        outputDataPath = str(outputFolder / "path.txt")
        np.savetxt(outputDataPath, path_modified)


def hpc_all_routes(map_folder_name:str, output_folder_name:str, jsonUnitFile:str):
    """To run on the HPC to calculate all routes between ISRU units"""

    # Load maps
    slopes, slopeMap, illumination, illuminationMap = loadMaps(map_folder_name)

    # Compared weights
    weights = [2.0, 1.0, 5.0, 5.0, 0.8, 0.2] # Medium slope cost, medium illumination cost

    path = str(Path(__file__).parent.absolute() / "units" / jsonUnitFile)

    units,_,_ = Unit.loadUnits(path, slopes.shape[0])

    # Test all routes
    for unit in units:
        for otherUnit in unit.routes:

            # Define points
            startPos = unit.YXpos()
            endPos   = otherUnit.YXpos()

            # Test algorithm
            print(f"Starting A* algorithm from {unit.name} to {otherUnit.name}")
            before = time.time()
            path_modified, explored = astar(slopeMap, illuminationMap, startPos, endPos, weights, isDebug=True, gui=False)
            print(f"-> A* finished running in {time.time() - before}s")

            if path_modified is None:
                continue

            # Draw results
            slopePathImage = drawWaypointsAndPath(slopes, startPos, endPos, path_modified)
            illuminationPathImage = drawWaypointsAndPath(illumination, startPos, endPos, path_modified)



            # Check that result folder exists or create it
            outputFolder = Path(__file__).parent.absolute() / "output" / output_folder_name / f"route_{unit.id}-{otherUnit.id}"
            outputFolder.mkdir(parents=True, exist_ok=True)

            # Save results
            outputImgPath = str(outputFolder / "path.png")
            cv2.imwrite(outputImgPath, illuminationPathImage)

            outputImgPath = str(outputFolder / "slope_path.png")
            cv2.imwrite(outputImgPath, slopePathImage)

            if explored is not None:
                outputImgPath = str(outputFolder / "explored.png")
                cv2.imwrite(outputImgPath, explored)


            outputDataPath = str(outputFolder / "path.txt")
            np.savetxt(outputDataPath, path_modified)

            writeToFile(str(outputFolder / "route_info.txt"), f"Route from \n{unit.name} with ID {unit.id}\nto\n{otherUnit.name} with ID {otherUnit.id}")





if __name__ == "__main__":
    map_folder_name = sys.argv[1] if len(sys.argv) > 1 else "maps"
    output_folder_name = sys.argv[2] if len(sys.argv) > 2 else "output"
    


    if len(sys.argv) > 3 and ".json" in sys.argv[3]:
        # Run all routes if JSON file given
        jsonFileName = sys.argv[3]
        hpc_all_routes(map_folder_name, output_folder_name, jsonFileName)
    
    else:
        # Start and end positions (Y,X)
        startPos = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 6 else (4200, 4400)
        endPos   = (int(sys.argv[5]), int(sys.argv[6])) if len(sys.argv) > 6 else (5200, 6000)
    
        # Run program to test weights
        # debug(map_folder_name, output_folder_name, startPos, endPos)

        # Run program on HPC to compare all configurations
        hpc_weights(map_folder_name, output_folder_name, startPos, endPos)
    
