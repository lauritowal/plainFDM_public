import sys
sys.path.append('..')
sys.path.append('.')

from FDM.geometryFDM.DatComGeometryBallClass import *
from FDM.geometryFDM.DatComGeometryC172Class import *
from FDM.geometryFDM.datComGeometry_beaver import DatComGeometry_beaver


# keys are strings, values are classes

class Config(object):

    def __init__(self):

        self.fehlererkennung = True
        self.icing = True
        self.geometrieClass = "Beaver"

        self.mappingDict = {"Ball": DatComGeometryBallClass,
                            "C172": DatComGeometryC172Class,
                            "Beaver": DatComGeometry_beaver}
