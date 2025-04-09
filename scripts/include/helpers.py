import numpy as np
import json

def showImage(windowName, img, max_dim=600):
    """Show a scaled down version of an image"""
    import cv2

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

def loadUnits(path, scale=1) -> dict:
    with open(path) as f:
        # Read JSON as dict
        data = json.load(f)

        # Clean data
        units = dict()
        for key,val in data.items():
            # Extract position for each unit
            x = int(val['x'])
            y = int(val['y'])
            print(f"{key}: ({x},{y})")
            units[key] = (int(scale*x), int(scale*y))

        return units
    
    print("[ERROR] Couldn't open JSON file")
    return {}