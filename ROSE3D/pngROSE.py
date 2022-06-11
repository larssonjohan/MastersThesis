import utils
import open3d as o3d
import numpy as np
from PIL import Image
import jsonHandler
import ROSE as R


class ROSE:
    def __init__(self, settings, params, vis):
        self.__roseMap = "input/input_map.png"
        self.__roseJSON = "input/rose_json.json"
        self.__json = jsonHandler.JSON()
        
        self.__settings = settings
        self.__params = params
        self.__vis = vis
        
        self.__png = np.asarray(Image.open(self.__settings['png']).convert('1'))
        
        self.__mapping = utils.nestedDict(2, list)
    

    def __initJSON(self):
        """
            Inits the JSON used by ROSE specifying path to generated grid map and the
            JSON detailing parameter values.
        """
        self.__json.setImageFile(self.__roseMap)
        self.__json.setJSONFile(self.__roseJSON)


    def __generateGridMap(self):
        """
            Generates a filtered grid map by matching point cloud coordinates to png coordinates
        """
        print(">>> Generating grid map [resolution: {}]".format(self.__settings['resolution']))
        
        minmax = utils.calculateBounds(self.__pointCloud)
        offsets = utils.calculateOffsets(minmax)
        height, width = utils.calculateDimensions(minmax, offsets, self.__settings['resolution'])
        
        pointList = []
        self.__gridmap = np.ones((self.__png.shape[0], self.__png.shape[1]), dtype=np.float32)
        
        for point in self.__pointCloud.points:
            x = round((point[0] + offsets.x) / self.__settings['resolution'])
            y = round((point[1] + offsets.x) / self.__settings['resolution'])

            if x < 0 or x > width - 1:
                continue
            if y < 0 or y > height - 1:
                continue

            if not self.__png[x][y]:
                self.__mapping[x][y].append(point)
                self.__gridmap[x, y] = 0.0
                pointList.append(point)
            
        if self.__vis['visualize_filtered_png']:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(pointList)
            o3d.visualization.draw_geometries([cloud])


    def __reconstruct(self, declutteredMap):
        """
            Rebuilds the decluttered point cloud
        """
        print(">>> Reconstructing decluttered point cloud")
        remainingPoints = []
        for row in range(declutteredMap.shape[0]):
            for column in range(declutteredMap.shape[1]):
                if(declutteredMap[row, column]):
                    for point in self.__mapping[row][column]:
                        remainingPoints.append(point)
        
        declutteredCloud = o3d.geometry.PointCloud()
        declutteredCloud.points = o3d.utility.Vector3dVector(remainingPoints)

        return declutteredCloud
    
    def run(self, pointCloud):
        """
            Filters point cloud coordinates by checking the occupancy in a grid map. Point cloud
            coordinates are kept iff they are occupied in the specified grid map.
        """
        print(">>> Running PNG")
        self.__pointCloud = pointCloud
        self.__generateGridMap()
        
        self.__initJSON()
        self.__json.createJSON(self.__params['peak_height'], self.__params['sigma'], self.__params['filter_level'])
        
        utils.writeImageToFile(self.__gridmap, self.__roseMap)
        
        declutteredMap = R.externalROSE(self.__roseJSON, "input_schema.json")
        
        declutteredCloud = self.__reconstruct(declutteredMap)

        return declutteredCloud, declutteredMap