from skimage import io
from skimage.util import img_as_ubyte

from extended_validator import ExtendedValidator
from fft_structure_extraction import FFTStructureExtraction as structure_extraction
from visualisation import visualisation

def externalROSE(json_file, schema_file):

    json_validation = ExtendedValidator(json_file, schema_file)

    success, config = json_validation.extended_validator()
    grid_map = img_as_ubyte(io.imread(config["input_map"]))
    rose = structure_extraction(grid_map, peak_height=config["peak_extraction_parameters"]["peak_height"],
                                smooth=config["peak_extraction_parameters"]["smooth_histogram"],
                                sigma=config["peak_extraction_parameters"]["sigma"])
    rose.process_map()

    filter_level = config["filtering_parameters"]["filter_level"]

    rose.simple_filter_map(filter_level)

    rose.generate_initial_hypothesis(type='simple', min_wall=5)

    rose.find_walls_flood_filing()
    
    plots = visualisation(rose)
    plots.show(config["visualisation_flags"])
    
    return rose.analysed_map
