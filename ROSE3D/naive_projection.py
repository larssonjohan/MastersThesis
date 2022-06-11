import utils
import numpy as np
import ROSE as R
import jsonHandler
import open3d as o3d


class ROSE:
    def __init__(self, settings, params):
        self.__settings = settings
        self.__params = params
        self.__mappings = utils.nestedDict(2, list)
        self.__roseMap = "input/input_map.png"
        self.__roseJSON = "input/rose_json.json"
        self.__json = jsonHandler.JSON()


    def __generateGridMap(self):
        """
            Generates a grid map from all points in the given point cloud
        """
        print(">>> Generating grid map [resolution: {}].".format(self.__settings['resolution']))
        minmax = utils.calculateBounds(self.__pointCloud)
        offsets = utils.calculateOffsets(minmax)
        height, width = utils.calculateDimensions(minmax, offsets, self.__settings['resolution'])
        self.__gridmap = np.ones((width + 1, height + 1), dtype=np.float32)
        for point in self.__pointCloud.points:
            x = round((point[0] + offsets.x) / self.__settings['resolution'])
            y = round((point[1] + offsets.y) / self.__settings['resolution'])
            
            self.__gridmap[x, y] = 0.0
            
            self.__mappings[x][y].append(point)
        

    def __initJSON(self):
        """
            Initiates the JSON used by ROSE by specifying the grid map to clutter
            and which JSON ROSE should use
        """
        self.__json.setImageFile(self.__roseMap)
        self.__json.setJSONFile(self.__roseJSON)


    def __reconstruct(self, declutteredMap):
        """
            Reconstructs the point cloud using the two-dimensional mapping between discrete pixel coordinates
            to continous point cloud coordinates.
        """
        print(">>> Reconstructing decluttered point cloud.")
        remainingPoints = []

        for row in range(declutteredMap.shape[0]):
            for column in range(declutteredMap.shape[1]):
                if(declutteredMap[row, column]):
                    for point in self.__mappings[row][column]:
                        remainingPoints.append(point)

        declutteredCloud = o3d.geometry.PointCloud()
        declutteredCloud.points = o3d.utility.Vector3dVector(remainingPoints)

        return declutteredCloud


    def run(self, pointCloud):
        """
            Runs naive projection ROSE by projecting the remaining points onto the xy plane
        """
        print(">>> Running naive projection")
        self.__pointCloud = pointCloud

        self.__generateGridMap()
        self.__initJSON()
        self.__json.createJSON(self.__params['peak_height'], self.__params['sigma'], self.__params['filter_level'])
        utils.writeImageToFile(self.__gridmap, self.__roseMap)
        print(">>> Running ROSE")
        declutteredMap = R.externalROSE(self.__roseJSON, "input_schema.json")
        declutteredCloud = self.__reconstruct(declutteredMap)

        return declutteredCloud, declutteredMap