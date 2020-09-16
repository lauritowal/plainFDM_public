import sys
sys.path.append('..')
sys.path.append('.')

from FDM.geometryFDM.DatComGeometryBallClass import *
from FDM.geometryFDM.DatComGeometryC172Class import *


# keys are strings, values are classes

class Config(object):

    def __init__(self):

        self.fehlererkennung = True
        self.icing = True
        self.geometrieClass = "C172"

        self.mappingDict = {"Ball" : DatComGeometryBallClass,
                            "C172" : DatComGeometryC172Class}