import utils
import inverter
import open3d as o3d
import naive_projection as hfr
import partitioned as pr
import pgmROSE as pgmr
import pngROSE as pngr


class ROSE3D:
    def __init__(self, settings):
        self.__settings = settings
        self.__declutteredMap = "result/decluttered_map.png"
        self.__declutteredCloud = "result/decluttered_cloud.pcd"
        self.__floor = []
        self.__ceiling = []


    def __loadPointCloud(self):
        """
            Loads point cloud
        """
        self.__pointCloud = o3d.io.read_point_cloud(self.__settings['pointcloud'])
        

    def __preparePointCloud(self):
        """
            Prepares the point cloud by; filtering based on specified min and max height and
            downsamples point cloud by specified voxel size.
        """
        print(">>> Preparing input point cloud.")
        if self.__settings['preparations']['height_filtered']:
            print(">>> Removing points outside of ]{}, {}[".format(self.__settings['preparations']['min_height'], self.__settings['preparations']['max_height']))
            pointList = []
            for point in self.__pointCloud.points:
                if point[2] <= self.__settings['preparations']['min_height']:
                    self.__floor.append(point)
                elif point[2] >= self.__settings['preparations']['max_height']:
                    self.__ceiling.append(point)
                else:
                    pointList.append(point)
            
            self.__pointCloud.points = o3d.utility.Vector3dVector(pointList)
        
        if self.__settings['preparations']['downsample']:
            print(">>> Downsampling [{}]".format(self.__settings['preparations']['voxel_size']))
            self.__pointCloud = self.__pointCloud.voxel_down_sample(self.__settings['preparations']['voxel_size'])
        

    def __parseSettings(self):
        """
            Parses settings to initialize used grid map generation method
        """
        if self.__settings['mode'] == 0:
            self.__ROSE = hfr.ROSE(self.__settings['naive_projection_settings'], self.__settings['ROSE'])
        elif self.__settings['mode'] == 1:
            self.__ROSE = pr.ROSE(self.__settings['partitioned_settings'], self.__settings['ROSE'])
        elif self.__settings['mode'] == 2:
            self.__ROSE = pgmr.ROSE(self.__settings['grid_map_pgm_settings'], self.__settings['ROSE'], self.__settings['visualization'])
        else:
            self.__ROSE = pngr.ROSE(self.__settings['grid_map_png_settings'], self.__settings['ROSE'], self.__settings['visualization'])


    def __reinsertion(self, declutteredCloud):
        pointList = []
        
        rebuild = self.__settings['reconstruction']['insert_floor'] or\
        self.__settings['reconstruction']['insert_ceiling']
        
        if rebuild:
            for point in declutteredCloud.points:
                pointList.append(point)

            if self.__settings['reconstruction']['insert_ceiling']:
                for point in self.__ceiling:
                    pointList.append(point)

            if self.__settings['reconstruction']['insert_floor']:
                for point in self.__floor:
                    pointList.append(point)
            
            declutteredCloud.points = o3d.utility.Vector3dVector(pointList)
        
        return declutteredCloud

    def run(self):
        """
            Runs ROSE using specified grid map generation method
        """
        self.__parseSettings()
        self.__loadPointCloud()
        self.__preparePointCloud()
        if self.__settings['visualization']['visualize_input_cloud']:
            o3d.visualization.draw_geometries([self.__pointCloud])
        declutteredCloud, declutteredMap = self.__ROSE.run(self.__pointCloud)

        declutteredCloud = self.__reinsertion(declutteredCloud)
        if self.__settings['visualization']['visualize_decluttered_cloud']:
            o3d.visualization.draw_geometries([declutteredCloud])
        
        o3d.io.write_point_cloud(self.__declutteredCloud, declutteredCloud)
        utils.writeImageToFile(declutteredMap, self.__declutteredMap)
        inverter.invertImage(self.__declutteredMap)
        print(">>> Decluttered grid map and point cloud saved locally.")
