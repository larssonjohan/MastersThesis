import jsonHandler
import ROSE3D as r

def parseSettings():
    """
        Loads and adjusts settings used by 3D ROSE adaptation
    """
    json = jsonHandler.JSON()
    settings = json.getSettings("settings/run_settings.json")

    if settings['naive_projection']:
        settings['pointcloud'] = settings['naive_projection_settings']['pointcloud']
        settings['mode'] = 0
    
    elif settings['partitioned']:
        settings['pointcloud'] = settings['partitioned_settings']['pointcloud']
        settings['mode'] = 1
    
    elif settings['grid_map_pgm']:
        settings['pointcloud'] = settings['grid_map_pgm_settings']['pointcloud']
        settings['mode'] = 2

    else:
        settings['pointcloud'] = settings['grid_map_png_settings']['pointcloud']
        settings['mode'] = 3

    return settings


if __name__ == "__main__":
    settings = parseSettings()

    r = r.ROSE3D(settings)
    r.run()

