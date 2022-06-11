from PIL import Image, ImageOps
import numpy as np

def invertImage(filename):
    """
        Inverts an image from occupied = white, unoccupied = black to
        unoccupied = white and occupied = black
    """
    im = Image.open(filename).convert('RGB')
    filenames = filename.split(".")
    fileout = filenames[:-1]
    im = ImageOps.invert(im)
    im.save("{}.png".format(fileout[0]), quality=100)