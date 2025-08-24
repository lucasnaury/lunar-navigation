import os
import numpy as np
from pathlib import Path
import sys
from include.units import Unit
from include.a_star import astar
from path_planning import loadMaps


def scorePath(path, slopeMap, illuminationMap, px2m):
    
    # Compute path dist
    # -> Calculate distances between consecutive points
    diffs = path[1:] - path[:-1]
    distances = np.sqrt(np.sum(diffs**2, axis=1))
    
    # -> Sum up all distances
    dist_cost = np.sum(distances)

    # Compute path other costs from maps
    slope_cost = np.sum(slopeMap[path])
    illumination_cost = np.sum(1.0 - illuminationMap[path])

    # Compute path other costs from heading
    headings = np.arctan2(diffs[:,1], diffs[:,0])
    headings_diffs = headings[1:] - headings[:-1]
    steering_cost = np.sum(np.abs(headings_diffs) / np.pi)

    total_cost = dist_cost + slope_cost + illumination_cost + steering_cost

    return dist_cost * px2m, total_cost



def main(unitsJsonfile, mapFolder, pathsFolder):
    # Paths
    jsonPath = str(Path(__file__).parent.absolute() / "units" / unitsJsonfile)
    mapFolderPath = os.path.join("maps", mapFolder)
    pathsFolderPath = Path(__file__).parent.absolute() / "output" / pathsFolder

    # Load maps
    _, slopeMap, _, illuminationMap = loadMaps(mapFolderPath)

    # Load units data
    units, px2m, _ = Unit.loadUnits(jsonPath, len(slopeMap))

    


    # ------------------------------
    # NAIVE STRAIGHT PATH COSTS
    # ------------------------------
    print("Computing naive costs...")
    totalNaiveDistCost = 0.0
    totalNaiveGridDistCost = 0.0
    totalNaiveCost = 0.0
    for unit in units:
        for otherUnit in unit.routes:
            # Distance cost
            naiveDistCost = np.linalg.norm(np.array(unit.pos()) - np.array(otherUnit.pos())) * px2m
            totalNaiveDistCost += naiveDistCost

            # Generate straight path
            straight_path, _ = astar(slopeMap, illuminationMap, unit.YXpos(), otherUnit.YXpos(), [1.0, 0.0, 0.0, 0.0, 0.0, 1.0], isDebug=False, gui=False)
            # Compute other costs
            distCost, cost = scorePath(np.array(straight_path), slopeMap, illuminationMap, px2m)
            totalNaiveGridDistCost += distCost
            totalNaiveCost += cost

    # ------------------------------
    # GLOBAL PATH COSTS
    # ------------------------------
    print("Computing global planner paths costs...")

    # Load global paths from files
    global_paths = []
    subdirectories = [item for item in pathsFolderPath.iterdir() if item.is_dir()]
    for filepath in subdirectories:
        pathData = np.loadtxt(os.path.join(filepath, "path.txt"))
        global_paths.append(pathData.astype(int))

    # Process paths
    totalDistCost = 0.0
    totalCost = 0.0
    for path in global_paths:
        distCost, cost = scorePath(path, slopeMap, illuminationMap, px2m)
        totalDistCost += distCost
        totalCost += cost


    # ------------------------------
    # PRINT RESULT
    # ------------------------------
    print("------------------ NAIVE PATH (STRAIGHT) ------------------")
    print(f"Total distance: \t{totalNaiveDistCost:0.2f}m")
    print(f"Total grid distance: \t{totalNaiveGridDistCost:0.2f}m")
    print(f"Total cost: \t\t{totalNaiveCost:0.2f}")

    print("------------------- GLOBAL PATH (A STAR) ------------------")
    print(f"Total distance: \t{totalDistCost:0.2f}m")
    print(f"Total cost:\t\t{totalCost:0.2f}")


if __name__ == "__main__":
    unitsFileName = sys.argv[1] if len(sys.argv) > 1 else "config1.json"
    mapFolder     = sys.argv[2] if len(sys.argv) > 2 else "cropped"
    pathsFolder   = sys.argv[3] if len(sys.argv) > 3 else "all_full_conf1"


    main(unitsFileName, mapFolder, pathsFolder)