import numpy as np

class DatComGeometry_beaver(object):

    def __init__(self):

        self.mass = 2288.23100000000
        self.inertia = np.array([[5.787969e3, 0.0, 1.1764e2], [0, 6.92893e3, 0], [-1.1764e2, 0, 1.1578329e4]])
        # transversal
        # alpha
        self.cx_alpha = 0.00292000000000000
        self.cx_alpha2 = 5.45900000000000
        self.cx_alpha3 = -5.16200000000000
        self.cz_alpha = -5.57800000000000
        self.cz_alpha3 = 3.44200000000000

        # beta
        self.cy_beta = -0.767800000000000

        # moments
        # alpha
        self.cm_alpha = -0.602800000000000
        self.cm_alpha2 = -2.14000000000000

        # beta
        self.cl_beta = -0.0618000000000000
        self.cm_beta2 = 0.692100000000000
        self.cn_beta = 0.00671900000000000
        self.cn_beta3 = 0.137300000000000

        # steuerflächen
        # aileron
        self.cx_da = 0
        self.cy_da = -0.0295600000000000
        self.cz_da = 0

        self.cl_da = -0.0991700000000000
        self.cl_da_alpha = -0.0826900000000000
        self.cm_da = 0
        self.cn_da = -0.00387200000000000

        # elevator
        self.cz_de_beta = -15.9300000000000
        self.cz_de = -0.398000000000000

        self.cm_de = -1.92100000000000

        # rudder

        self.cx_dr = 0.0341200000000000
        self.cy_dr = 0.115800000000000
        self.cy_dr_alpha = 0.523800000000000

        self.cl_dr = 0.00693400000000000
        self.cn_dr = -0.0826500000000000
        
        # p
        self.cy_p = -0.124000000000000
        self.cl_p = -0.504500000000000
        self.cn_p = -0.158500000000000

        # q
        self.cx_q = -0.674800000000000
        self.cz_q = -2.98800000000000
        self.cm_q = -15.5600000000000
        self.cn_q = 0.159500000000000

        # r
        self.cy_r = 0.366600000000000
        self.cl_r = 0.169500000000000
        self.cm_r = -0.311800000000000
        self.cn_r = -0.111200000000000

        # Reference Span, length
        self.span = 14.6300000000000
        self.chord = 1.58750000000000

        # trottle to RPM
        # todo: besser = Leerlauf
        self.stellbereichThrust = np.array([0.0, 0.16666, 0.3333, 0.5, 0.66666, 0.83333, 1.0])
        self.rpmMotor = np.array([-0.03457, 35.0, 269.0, 1149.0, 2030.0, 2264.0, 2300.0])  # min RPM & max RPM
        self.manifold = np.array([0, 0.4, 3.0, 13.0, 22.5735, 25.7, 26])

        # dpt
        self.cx_dpt = 0.116100000000000
        self.cx_dpt2_alpha = 0.145300000000000
        self.cy_dpt = 0
        self.cz_dpt = -0.156300000000000

        self.cl_dpt = 0
        self.cl_dpt_alpha2 = -0.0140600000000000
        self.cm_dpt = -0.0789500000000000
        self.cn_dpt = -0.0789500000000000
        self.cn_dpt3 = -0.00302600000000000





