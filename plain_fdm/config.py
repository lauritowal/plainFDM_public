from plain_fdm.geometryFDM.dat_com_geometry_beaver import DatComGeometry_beaver
from plain_fdm.geometryFDM.dat_com_geometry_ball import DatComGeometryBall
from plain_fdm.geometryFDM.dat_com_geometry_c172 import DatComGeometryC172
# keys are strings, values are classes

class Config(object):

    def __init__(self):

        self.fehlererkennung = True
        self.icing = True
        self.geometrieClass = "C172"

        self.mappingDict = {"Ball": DatComGeometryBall,
                            "C172": DatComGeometryC172,
                            "Beaver": DatComGeometry_beaver}
