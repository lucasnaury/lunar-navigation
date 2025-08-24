import os
import numpy as np
from pathlib import Path
import sys
from include.units import Unit
from path_planning import loadMaps


def scorePath(path, weights, slopeMap, illuminationMap, px2m):
    
    # Calculate differences between consecutive points
    diffs = np.diff(path, axis=0)
    dist_costs = np.linalg.norm(diffs, axis=1)
    total_dist = np.sum(dist_costs)

    # Slope and illumination costs for each point
    rows, cols = path[:, 0].astype(int), path[:, 1].astype(int)
    slope_costs = slopeMap[rows, cols]
    illumination_costs = 1.0 - illuminationMap[rows, cols]

    # Steering costs (change in heading)
    headings = np.arctan2(diffs[:, 0], diffs[:, 1])
    steering_diffs = np.diff(headings)
    steering_costs = np.abs(steering_diffs) / np.pi

    # Pad steering_costs and dist_costs to align with path length
    steering_costs = np.insert(steering_costs, 0, 0.0)
    dist_costs = np.insert(dist_costs, 0, 0.0)

    # Weights
    w_dist, w_steering, w_slope, w_light = weights

    # Total cost (normalized by sum of weights)
    g_cost = (
        np.sum(w_dist * dist_costs + w_slope * slope_costs + w_light * illumination_costs) +
        np.sum(w_steering * steering_costs)
        ) / (w_dist + w_slope + w_light + w_steering)


    return total_dist * px2m, g_cost

def bresenham_line(start, end):
    """
    Returns a list of (y, x) coordinates on a grid between start and end using Bresenham's algorithm.
    start, end: tuples/lists of (y, x)
    """
    y0, x0 = start
    y1, x1 = end
    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((y, x))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((y, x))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((y1, x1))
    return points

def main(unitsJsonfile, mapFolder, pathsFolder):
    # Paths
    jsonPath = str(Path(__file__).parent.absolute() / "units" / unitsJsonfile)
    mapFolderPath = os.path.join("maps", mapFolder)
    pathsFolderPath = Path(__file__).parent.absolute() / "output" / pathsFolder

    # Load maps
    _, slopeMap, _, illuminationMap = loadMaps(mapFolderPath)

    # Load units data
    units, px2m, _ = Unit.loadUnits(jsonPath, len(slopeMap))

    
    weights = [2.0, 1.0, 5.0, 5.0]

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
            # straight_path, _ = astar(slopeMap, illuminationMap, unit.YXpos(), otherUnit.YXpos(), [1.0, 0.0, 0.0, 0.0, 0.0, 1.0], isDebug=False, gui=False)
            straight_path = bresenham_line(unit.YXpos(), otherUnit.YXpos())

            # Compute other costs
            distCost, cost = scorePath(np.array(straight_path), weights, slopeMap, illuminationMap, px2m)
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
        distCost, cost = scorePath(path, weights, slopeMap, illuminationMap, px2m)
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