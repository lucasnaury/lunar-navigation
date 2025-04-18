import numpy as np
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



color_list = [
    (107,206,35),
    (130,41,126),
    (79,84,240),
    (44,167,236),
    (34,86,238),
    (243,164,32),
    (116,118,238),
    (119,167,4),
    (69,128,201),
]
returned_colors = []
def randomColor():
    global returned_colors

    while True:
        randomIndex = np.random.randint(0,len(color_list))
        
        if randomIndex not in returned_colors:
            break

    returned_colors.append(randomIndex)

    if len(returned_colors) == len(color_list):
        returned_colors = []

    return color_list[randomIndex]



def drawPath(map, path, color=(255,0,0)):
    # Convert to BGR if grayscale
    isGrayscale = (len(map.shape) == 2)
    colormap = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR) if isGrayscale else map
    
    # Compute thickness based on image size
    thickness = max(1,int(np.floor(1 * map.shape[0] / 150)))

    # Reduce opacity
    colormap = np.floor(colormap * 0.96 + 255*0.04)
    colormap = colormap.astype(np.uint8)

    # Draw line
    for i in range(len(path) - 1):
        # if i%10 > 1:
        #     continue
        p_start = (path[i][1], path[i][0])
        p_end = (path[i+1][1], path[i+1][0])
        cv2.line(colormap, p_start, p_end, color, thickness)

    return colormap