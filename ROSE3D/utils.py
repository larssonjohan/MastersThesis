from collections import defaultdict
import matplotlib.image as img

class minMaxValues:
    """
    Metric bounding box of coordinates found in loaded point cloud.
    """
    def __init__(self):
        self.minX = 0.0
        self.maxX = 0.0
        self.minY = 0.0
        self.maxY = 0.0
        self.minZ = 0.0
        self.maxZ = 0.0

class Offsets:
    """
    Stores the offset for each point to convert a pointcloud with negative
    coordinates to a png with coordinates [0, inf[
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


def nestedDict(n, type):
    """
    Creates a nested dictionary of depth n with a specific type
    """
    if n == 1:
        return defaultdict(type)
    else:
        return defaultdict(lambda: nestedDict(n - 1, type))


def calculateOffsets(minmax):
    """
        Calculates offsets for the given metric minimums and maximums
    """
    offsets = Offsets()
    if minmax.minX < 0.0:
        offsets.x = abs(minmax.minX)
    if minmax.minY < 0.0:
        offsets.y = abs(minmax.minY)
    if minmax.minZ < 0.0:
        offsets.z = abs(minmax.minZ)
    
    return offsets


def calculateDimensions(minmax, offsets, resolution):
    """
        Calculats dimensions of the resulting grid map
    """
    height = round((minmax.maxY + offsets.y) / resolution)
    width = round((minmax.maxX + offsets.x) / resolution)

    return height, width


def writeImageToFile(gridmap, filename):
    """
        Writes grid map to file
    """
    img.imsave(filename, gridmap, cmap="gray")

def calculateBounds(pointCloud):
    """
        Calculates bounds of point cloud (min/max x, y, z)
    """
    minmax = minMaxValues()

    min = pointCloud.get_min_bound()
    max = pointCloud.get_max_bound()
    
    minmax.minX = min[0]
    minmax.maxX = max[0]

    minmax.minY = min[1]
    minmax.maxY = max[1]

    minmax.minZ = min[2]
    minmax.maxZ = max[2]
    
    return minmax