import sys
import os
from pathlib import Path
import numpy as np
import cv2
from stl.mesh import Mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# ---------------------------------------------------------------- #
#                            VISUALISATION
# ---------------------------------------------------------------- #
def plotMesh(mesh:Mesh):
    # Show STL model
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')

    # Load the STL files and add the vectors to the plot
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))

    # Auto scale to the mesh size
    scale = mesh.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)

    # Show the plot to the screen
    plt.show()




# ---------------------------------------------------------------- #
#                          INDIVIDUAL MAPS
# ---------------------------------------------------------------- #
def getSlopeMap(mesh, size):

    # Initialise empty map
    slopeMap = np.zeros((size, size))
    
    # Extract normal vectors
    normals = mesh.normals  # Shape (N, 3), where N is the number of triangles

    # Define the horizontal plane normal (z-up)
    horizontal = np.array([0, 0, 1])

    # Compute slopes as the angle between the triangle normal and the horizontal
    cos_theta = np.einsum('ij,j->i', normals, horizontal) / np.linalg.norm(normals, axis=1)
    cos_theta = np.clip(cos_theta, -1, 1)  # Avoid numerical issues

    slopes = np.abs(np.arccos(cos_theta))  # Absolute radians

    # Remove mesh offset
    # faces = mesh.vectors[:,:,0:2]
    # faces -= np.min(np.min(faces, axis=1), axis=0)
    
    # Calculate normal cell index
    normalsPositions = np.floor(np.average(faces, axis=1)).astype(int) # Get cell index based on face center
    
    # Store average of slope (2 triangles for 1 cell) in cell
    np.add.at(slopeMap, (normalsPositions[:, 0], normalsPositions[:, 1]), slopes/2)

    return slopeMap


def getIlluminationMap(mesh):
    
    # Get images folder
    path = str(Path().parent.absolute() / "illumination")

    # Filter images
    files = [os.path.join(path, f) for f in os.listdir(path) if f[-5:]== "0.png"]


    # Get image size
    size = cv2.imread(files[0]).shape
    print(size)

    # Initialise illumination map
    illuminationMap = np.zeros(size[0:2])


    for f in files:

        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)

        illuminationMap += img

    return illuminationMap / len(files)



# ---------------------------------------------------------------- #
#                                MAIN
# ---------------------------------------------------------------- #
def main(path):
    # Import moon surface STL file
    print("Loading STL file...")
    mesh = Mesh.from_file(path)
    print(Path().parent.absolute())

    # Plot 3D mesh
    # plotMesh(mesh)



    # Initialise costmap
    outputDim = int(np.floor(np.sqrt(mesh.vectors.shape[0] / 2)))
    print("Output dimension:", outputDim)

    costMap = np.zeros((outputDim, outputDim))
    

    # -> Slope map
    slopeMap = getSlopeMap(mesh, outputDim)

    slopeNormalised = (slopeMap - np.min(slopeMap)) / (np.max(slopeMap) - np.min(slopeMap))

    print("Slope map: ",slopeMap)
    print("Normalised map: ",slopeNormalised)
    showImage("Slope map", slopeNormalised)



    # -> Illumination map
    illuminationMap = getIlluminationMap(mesh)
    showImage("Illumination map", illuminationMap)



    cv2.waitKey(0)








if __name__ == "__main__":
    # Check if STL path is given as arg
    filepath = sys.argv[1] if len(sys.argv) > 1 else "../euro2moon\worlds\models\moon_surface\moon_cropped.stl"
    # filepath = sys.argv[1] if len(sys.argv) > 1 else "C://Users//lucas//Desktop//surface_test.stl"
    
    # Run program
    main(filepath)