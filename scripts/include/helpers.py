import numpy as np
import json
import cv2

def showImage(windowName, img, max_dim=600):
    """Show a scaled down version of an image"""

    cv2.namedWindow(windowName,cv2.WINDOW_KEEPRATIO)
    cv2.imshow(windowName, img)
    

    if img.shape[1] > img.shape[0]:
        newWidth = max_dim
        newHeight = int(newWidth / img.shape[1] * img.shape[0])
    else:        
        newHeight = max_dim
        newWidth = int(newHeight / img.shape[0] * img.shape[1])

    cv2.resizeWindow(windowName, (newWidth, newHeight))


def normaliseImage(img:np.matrix) -> np.matrix:
    return (img - 1.0*np.min(img)) / (np.max(img) - np.min(img))


def writeToFile(filepath:str, txt:str):
    f = open(filepath, "w")
    f.write(txt)
    f.close()


def randomColor():
    return tuple(np.random.choice(range(256), size=3).tolist())


def loadUnits(path, targetSize=None) -> tuple[dict, float]:
    with open(path) as f:
        # Read JSON as dict
        data = json.load(f)

        # Separate data
        metadata = data["metadata"]
        units_data = data["units"]

        # Calculate scaling factor
        if targetSize == None:
            scale = 1
        else:
            scale = targetSize / int(metadata["mapSize"])

        px2m = float(metadata["px2m"]) / scale

        # Clean data
        units = dict()
        for val in units_data:
            # Extract position for each unit
            name = val['name']
            x = int(val['x'])
            y = int(val['y'])
            
            # Store it in dict
            units[name] = (int(scale*x), int(scale*y))

        return units, px2m, scale
    
    print("[ERROR] Couldn't open JSON file")
    return {}, 0, 0


def drawPath(map, path, color=(255,0,0)):
    # Convert to BGR if grayscale
    isGrayscale = (len(map.shape) == 2)
    colormap = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR) if isGrayscale else map
    
    # Compute thickness based on image size
    thickness = max(1,int(np.floor(1 * map.shape[0] / 200)))

    # Draw line
    for i in range(len(path) - 1):
        p_start = (path[i][1], path[i][0])
        p_end = (path[i+1][1], path[i+1][0])
        cv2.line(colormap, p_start, p_end, color, thickness)

    return colormap