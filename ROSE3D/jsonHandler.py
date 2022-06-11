import json

from numpy.lib.type_check import imag

class JSON:
    def __init__(self, filenames, description="Testing"):
        self.__jsonFilename = filenames['JSON']
        self.__imageFilename = filenames['image']
        self.__description = description

    def __init__(self, description="Testing"):
        self.__description = description

    def setDescription(self, description):
        self.__description = description

    def getSettings(self, filename):
        settings = None
        with open(filename, 'r') as f:
            settings = json.load(f)
        
        return settings
    
    def setFilenames(self, filenames):
        self.__jsonFilename = filenames['JSON']
        self.__imageFilename = filenames['image']

    def setImageFile(self, imagefile):
        self.__imageFilename = imagefile
        
    def setJSONFile(self, jsonfile):
        self.__jsonFilename = jsonfile
    

    def createJSON(self, peakHeight, sigma, filterlevel):
        output = {}
        output['input_map'] = self.__imageFilename
        output['map_description'] = self.__description
        output['peak_extraction_parameters'] = {}
        output['peak_extraction_parameters']['peak_height'] = peakHeight
        output['peak_extraction_parameters']['smooth_histogram'] = True
        output['peak_extraction_parameters']['sigma'] = sigma
        output['filtering_parameters'] = {}
        output['filtering_parameters']['filter_level'] = filterlevel
        output['visualisation_flags'] = {}
        output['visualisation_flags']['Binary map'] = False
        output['visualisation_flags']['Simple Filtered Map'] = False
        output['visualisation_flags']['Unfolded FFT Spectrum'] = False
        output['visualisation_flags']['Map with directions'] = False
        output['visualisation_flags']['Map with walls'] = False
        output['visualisation_flags']['Wall lines with mbb'] = False
        output['visualisation_flags']['Map with slices'] = False
        output['visualisation_flags']['Map Scored Good'] = False

        with open(self.__jsonFilename, 'w') as file:
            json.dump(output, file)
            return True

        return False
    