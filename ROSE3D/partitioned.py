import utils
import math
import open3d as o3d
import numpy as np
import jsonHandler
import ROSE as R


class ROSE:
    def __init__(self, settings, params):
        self.__partitionPath = "input/partition_"
        self.__roseJSON = "input/rose_json.json"
        self.__settings = settings
        self.__params = params
        self.__declutteredMaps = []
        self.__json = jsonHandler.JSON()


    def __initializeGridMaps(self, mapHeight, width, height):
        """
            Initializes x grid maps, x mappings and a counter used to make sure that 
            partitions containing 0 points remains untouched
        """
        nrMaps = math.ceil(mapHeight / self.__settings['partition_height'])
        self.__mergedGridMap = np.ones((width + 1, height + 1), dtype=np.float32)
        self.__gridmaps = []
        self.__nrTimes = []
        self.__mappings = []

        for _ in range(nrMaps):
            self.__gridmaps.append(np.ones((width + 1, height + 1), dtype=np.float32))
            self.__nrTimes.append(0)
            self.__mappings.append(utils.nestedDict(2, list))


    def __generateGridMaps(self):
        """
            Generates X grid maps from the specified point cloud
        """
        minmax = utils.calculateBounds(self.__pointCloud)
        offsets = utils.calculateOffsets(minmax)
        height, width = utils.calculateDimensions(minmax, offsets, self.__settings['resolution'])
        mapHeight = abs(minmax.maxZ) + abs(minmax.minZ)
        
        self.__initializeGridMaps(mapHeight, width, height)
        print(">>> Creating {} grid maps [partition_height: {}]".format(len(self.__gridmaps), self.__settings['partition_height']))
        for point in self.__pointCloud.points:
            x = round((point[0] + offsets.x) / self.__settings['resolution'])
            y = round((point[1] + offsets.y) / self.__settings['resolution'])
            index = math.floor((point[2] + offsets.z) / self.__settings['partition_height'])
            
            self.__nrTimes[index] += 1
            self.__mergedGridMap[x, y] = 0.0
            self.__gridmaps[index][x, y] = 0.0
            self.__mappings[index][x][y].append(point)


    def __callROSE(self):
        """
            Calls ROSE once for each generated grid map iff there is at least
            one point residing in the grid map.
        """
        for i in range(len(self.__gridmaps)):
            print(">>> ROSE call ({}/{})".format(i + 1, len(self.__gridmaps)))
            if self.__nrTimes[i] == 0:
                continue
            else:
                imageFile = "{}{}.png".format(self.__partitionPath, i)
                self.__json.setImageFile(imageFile)
                self.__json.setJSONFile(self.__roseJSON)
                self.__json.createJSON(self.__params['peak_height'], self.__params['sigma'], self.__params['filter_level'])
                utils.writeImageToFile(self.__gridmaps[i], imageFile)
                declutteredMap = R.externalROSE(self.__roseJSON, "input_schema.json")
                self.__declutteredMaps.append((i, declutteredMap))
    

    def __reconstruct(self):
        """
            Reconstructs the declutter partitions into one, decluttered, point cloud.
            Merges the decluttered partitioned grid maps into one by OR as well.
        """
        print(">>> Reconstructing decluttered point cloud and grid map.")
        remainingPoints = []
        mergedGridMap = np.zeros((self.__declutteredMaps[0][1].shape[0], self.__declutteredMaps[0][1].shape[1]), np.float32)
        for container in self.__declutteredMaps:
            index = container[0]
            map = container[1]
            for row in range(map.shape[0]):
                for column in range(map.shape[1]):
                    if(map[row, column]):
                        mergedGridMap[row, column] = 1
                        for point in self.__mappings[index][row][column]:
                            remainingPoints.append(point)
        
        declutteredCloud = o3d.geometry.PointCloud()
        declutteredCloud.points = o3d.utility.Vector3dVector(remainingPoints)
        
        return declutteredCloud, mergedGridMap


    def run(self, pointCloud):
        """
            Generates partitioned grid maps, calls ROSE for each partition and reconstructs
            the decluttered point cloud by merging partitions.
        """
        print(">>> Running partitioned")
        self.__pointCloud = pointCloud
        self.__generateGridMaps()
        self.__callROSE()

        declutteredCloud, declutteredGridMap = self.__reconstruct()
        print("Point cloud in: {}, decluttered cloud: {}, removed points: {}".format(len(self.__pointCloud.points), len(declutteredCloud.points), len(self.__pointCloud.points) - len(declutteredCloud.points)))
        utils.writeImageToFile(self.__mergedGridMap, "input/merged_grid_map.png")
        return declutteredCloud, declutteredGridMap