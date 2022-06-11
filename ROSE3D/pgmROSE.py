import ROSE as R
import jsonHandler
import utils
import matplotlib.pyplot as plt
import matplotlib.image as img
import open3d as o3d
import numpy as np
import yaml
from PIL import Image
import math


class ROSE:
    def __init__(self, settings, params, vis):
        self.__roseMap = "input/input_map.png"
        self.__roseJSON = "input/rose_json.json"
        self.__blackAndWhite = "input/black_and_white_pgm.png"
        self.__json = jsonHandler.JSON()

        self.__settings = settings
        self.__params = params
        self.__vis = vis

        self.__mapping = utils.nestedDict(2, list)

        self.__corners = {"L": {"T": self.__topLeftCorner, "B": self.__bottomLeftCorner}, "R": {"T": self.__topRightCorner, "B": self.__bottomRightCorner}}

    
    def __initJSON(self):
        """
            Specifies path to grid map used by ROSE to declutter and JSON used by ROSE detailing
            parameter values
        """
        self.__json.setImageFile(self.__roseMap)
        self.__json.setJSONFile(self.__roseJSON)


    def __bottomLeftCorner(self, x, y):
        """
            Transformation function if origin is bottom left corner
        """
        return not self.__pgm[self.__yOffset - y][x]


    def __bottomRightCorner(self, x, y):
        """
            Transformation function if origin is bottom right corner
        """
        return not self.__pgm[self.__yOffset - y][self.__xOffset - x]


    def __topRightCorner(self, x, y):
        """
            Transformation function if origin is top right corner
        """
        return not self.__pgm[y][self.__xOffset - x]

    def __topLeftCorner(self, x, y):
        """
            Transformation function if origin is top left corner
        """
        return not self.__pgm[y][x]

    def __convertPGM(self):
        """
            Converts PGM to black and white
        """
        newPGM = np.ones((self.__pgm.shape[0], self.__pgm.shape[1]), dtype=np.bool8)

        for row in range(self.__pgm.shape[0]):
            for column in range(self.__pgm.shape[1]):
                if not self.__pgm[row, column]:
                    newPGM[row, column] = 0
        
        self.__pgm = newPGM

    def __loadPGM(self):
        """
            Loads PGM and converts it to black and white
        """
        with open(self.__settings['pgm'], 'rb') as f:
            self.__pgm = plt.imread(f)
        
        self.__convertPGM()
        img.imsave(self.__blackAndWhite, self.__pgm, cmap="gray")
        self.__pgm = np.asarray(Image.open(self.__blackAndWhite).convert("1"))


    def __loadYAML(self):
        """
            Loads YAML detailing origin and resolution of specified grid map
        """
        with open(self.__settings['yaml'], 'r') as f:
            y = yaml.safe_load(f)
            self.__resolution = y['resolution']
            self.__origin = utils.Offsets()
            self.__origin.x = y['origin'][0]
            self.__origin.y = y['origin'][1]

    def __generateGridMap(self):
        """
            Generates grid map from point cloud using occupied pixels in the specified 
            pgm grid map. Points are kept for decluttering iff they are occupied
        """
        print(">>> Generating grid map")
        width = self.__pgm.shape[1]
        height = self.__pgm.shape[0]

        pointList = []
        self.__gridmap = np.ones((height, width), dtype=np.float32)

        self.__xOffset = 0
        self.__yOffset = 0

        if self.__settings['corner'][0] != "L":
            self.__xOffset = width
        
        if self.__settings['corner'][1] != "T":
            self.__yOffset = height

        pixel = self.__corners[self.__settings['corner'][0]][self.__settings['corner'][1]]

        for point in self.__pointCloud.points:
            x = math.floor((point[0] - self.__origin.x) / self.__resolution)
            y = math.floor((point[1] - self.__origin.y) / self.__resolution)

            if x < 0 or x >= width:
                continue

            if y < 0 or y >= height:
                continue

            if pixel(x, y):
                self.__mapping[y][x].append(point)
                self.__gridmap[y][x] = 0
                pointList.append(point)
        
        if self.__vis['visualize_filtered_pgm']:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(pointList)
            o3d.visualization.draw_geometries([cloud])

        
    def __reconstruct(self, declutteredMap):
        """
            Reconstructs the decluttered point cloud using remaining
            pixels in the decluttered grid map
        """
        print(">>> Reconstructing decluttered point cloud")
        remainingPoints = []

        for row in range(declutteredMap.shape[0]):
            for column in range(declutteredMap.shape[1]):
                if declutteredMap[row, column]:
                    for point in self.__mapping[row][column]:
                        remainingPoints.append(point)

        declutteredCloud = o3d.geometry.PointCloud()
        declutteredCloud.points = o3d.utility.Vector3dVector(remainingPoints)

        return declutteredCloud


    def run(self, pointCloud):
        """
            Decluttering by ROSE using PGM to filter points to keep for decluttering
        """
        print(">>> Running PGM")
        self.__pointCloud = pointCloud

        self.__loadYAML()
        self.__loadPGM()
        self.__initJSON()
        self.__generateGridMap()
        utils.writeImageToFile(self.__gridmap, self.__roseMap)
        print(">>> Running ROSE")
        declutteredMap = R.externalROSE(self.__roseJSON, "input_schema.json")
        declutteredCloud = self.__reconstruct(declutteredMap)

        return declutteredCloud, declutteredMap

    