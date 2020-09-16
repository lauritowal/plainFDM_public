import pandas as pd

import matplotlib.pyplot as plt
import numpy as np
import pandas_bokeh
from bokeh.plotting import figure, output_file, save, show
import itertools
from bokeh.palettes import Dark2_5 as palette

#colors has a list of colors which can be used in plots
colors = itertools.cycle(palette)




class PlotState(object):

    def __init__(self):
        self.df = pd.DataFrame(columns=['u', 'v', 'w', 'x', 'y', 'z', 'p', 'q', 'r', 'phi/grad', 'theta/grad',
                                        'psi/grad', 'forceX', 'forceY', 'forceZ', 'momentsX', 'momentsY', 'momentsZ', 'alpha',
                                        'beta', 'deltaElevator', 'deltaAileron', 'deltaRudder', 'deltaThrust',
                                        'tVerlauf'])
        self.df_target = pd.DataFrame(columns=['target', 'tVerlauf'])


    def addData(self, state, force, moments, alpha, beta, deltaControls, i):
        self.df = self.df.append(pd.Series([state[0], state[1], state[2], state[3], state[4], state[5], state[6],
                                            state[7], state[8], np.rad2deg(state[9]), np.rad2deg(state[10]), np.rad2deg(state[11]), force[0], force[1],
                                            force[2], moments[0], moments[1], moments[2], alpha, beta, deltaControls[0],
                                            deltaControls[1], deltaControls[2], deltaControls[3], i],
                                           index=self.df.columns), ignore_index=True)

    def add_data_Ziel(self, target, i):
        self.df_target = self.df_target.append(pd.Series([target, i],
                                           index=self.df_target.columns), ignore_index=True)

    def plot(self, listData2Plot):


        TOOLTIPS = [
            ("Name", '$name'),
            ("index", "$index"),
            ("Wert", "$y")]
        p = figure(title="simple line example", x_axis_label='Datapoints', y_axis_label='Data', tooltips=TOOLTIPS)

        for items in listData2Plot:
            p.line(self.df[items].index.values, self.df[items], name=items, legend_label=items, line_width=2, color=next(colors))
        p.line(self.df_target['target'].index.values, self.df_target['target'], name="target")
        save(p)
