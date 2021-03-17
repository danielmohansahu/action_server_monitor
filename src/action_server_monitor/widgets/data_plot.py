#!/usr/bin/env python

# Copyright (c) 2014, Austin Hendrix
# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout

from .mat_data_plot import MatDataPlot

class DataPlotException(Exception):
    pass

class DataPlot(QWidget):
    """A widget for displaying a plot of data

    The DataPlot widget displays a plot on one a MatPlot backend.
    
    The DataPlot widget manages the plot backend internally, and can save
    and restore the internal state using `save_settings` and `restore_settings`
    functions.

    Currently, the user MUST call `restore_settings` before using the widget,
    to cause the creation of the enclosed plotting widget.
    """

    # pre-defined colors:
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    _colors = [Qt.blue, Qt.red, Qt.cyan, Qt.magenta, Qt.green,
               Qt.darkYellow, Qt.black, Qt.darkCyan, Qt.darkRed, Qt.gray]

    limits_changed = Signal()
    _redraw = Signal()
    _add_curve = Signal(str, str, 'QColor', bool)

    def __init__(self, parent=None):
        """Create a new, empty DataPlot
        """
        super(DataPlot, self).__init__(parent)

        # initialize miscellaneous class variables
        self._color_index = 0
        self._scrollwidth = 10

        # determine whether or not to scroll through the window (in X)
        self._autoscroll = True

        # initialize limits
        # @TODO move static Y values elsewhere? Kinda breaks modularity...
        self._x_limits = [0, 1.0]
        self._static_ylimits = [-0.5, 9.5]
        
        # initialize currently tracked data
        self._curves = {}

        # initialize various input signals and their callbacks and widgets
        self._redraw.connect(self._do_redraw)
        self._layout = QHBoxLayout()
        self.setLayout(self._layout)

        # initialize backend data processor (based on matplotlib)
        self._data_plot_widget = MatDataPlot(self)
        self._data_plot_widget.limits_changed.connect(self.limits_changed)
        self._add_curve.connect(self._data_plot_widget.add_curve)
        self._layout.addWidget(self._data_plot_widget)

        # update axes
        self._data_plot_widget.set_xlim(self._x_limits)
        self._data_plot_widget.set_ylim(self._static_ylimits)
        self.redraw()

        # display current data
        self.show()

    # interface out to the managing GUI component: get title, save, restore,
    # etc
    def getTitle(self):
        """get the title of the current plotting backend"""
        return "MatPlot"

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    # interface out to the managing DATA component: load data, update data,
    # etc
    def autoscroll(self, enabled=True):
        """Enable or disable autoscrolling of the plot"""
        self._autoscroll = enabled

    def redraw(self):
        """ Trigger a redraw.

        @TODO investigate why this is a signal; is anyone using it externally?
        """
        self._redraw.emit()

    def _do_redraw(self):
        """Redraw the underlying plot

        This causes the underlying plot to be redrawn. This is usually used
        after adding or updating the plot data"""
        self._merged_autoscale()
        for curve_id in self._curves:
            curve = self._curves[curve_id]
            try:
                self._data_plot_widget.set_values(curve_id, curve['x'], curve['y'])
            except KeyError:
                # skip curve which has been removed in the mean time
                pass
        self._data_plot_widget.redraw()

    def _get_curve(self, curve_id):
        if curve_id in self._curves:
            return self._curves[curve_id]
        else:
            raise DataPlotException("No curve named %s in this DataPlot" % (curve_id))

    def add_curve(self, curve_id, curve_name, data):
        """Add a new, named curve to this plot

        Add a curve named `curve_name` to the plot, with initial data series
        `data_x` and `data_y`.

        Future references to this curve should use the provided `curve_id`

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        # temporarily ignore other data (to make sure upstream is working)
        data_x = []
        data_y = []
        for v in data.values():
            for m in v:
                data_x.append(m[0])
                data_y.append(m[1])

        curve_color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1

        self._curves[curve_id] = {'x': numpy.array(data_x),
                                  'y': numpy.array(data_y),
                                  'name': curve_name,
                                  'color': curve_color}
        self._add_curve.emit(curve_id, curve_name, curve_color, True)

    def remove_curve(self, curve_id):
        """Remove the specified curve from this plot"""
        # TODO: do on UI thread with signals
        if curve_id in self._curves:
            del self._curves[curve_id]
        self._data_plot_widget.remove_curve(curve_id)

    def update_values(self, curve_id, values, sort_data=True):
        """Append new data to an existing curve

        `values_x` and `values_y` will be appended to the existing data for
        `curve_id`

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.

        If `sort_data` is set to False, values won't be sorted by `values_x`
        order.
        """
        # temporarily ignore other data (to make sure upstream is working)
        values_x = []
        values_y = []
        for v in values.values():
            for m in v:
                values_x.append(m[0])
                values_y.append(m[1])

        curve = self._get_curve(curve_id)
        curve['x'] = numpy.append(curve['x'], values_x)
        curve['y'] = numpy.append(curve['y'], values_y)

        # check if this new curve has a newer X data point than we've previously seen
        curve_x_max = numpy.max(curve['x'])
        if curve_x_max > self._x_limits[1]:
            self._x_limits[1] = curve_x_max

        if sort_data:
            # sort resulting data, so we can slice it later
            sort_order = curve['x'].argsort()
            curve['x'] = curve['x'][sort_order]
            curve['y'] = curve['y'][sort_order]

    def clear_values(self):
        """Clear the values for all curves

        Note that the plot is not redrawn automatically; call `redraw()` to make
        any changes visible to the user.
        """
        # clear internal curve representation
        for curve_id in self._curves:
            self._curves[curve_id]['x'] = numpy.array([])
            self._curves[curve_id]['y'] = numpy.array([])
        self._x_limits = [0, 1.0]

    def _merged_autoscale(self):
        """ Update the X axes to include the latest data.

        If `autoscroll`, we keep the last `self.scrollwidth` seconds
        of data in the field of view. If not, we keep all data in the
        field of view.
        """
        
        if self._autoscroll:
            # set xlimit lower bound to fit our window
            self._x_limits[0] = max(0, self._x_limits[1] - self._scrollwidth)

        # update underlying axes object's limits
        self._data_plot_widget.set_xlim(self._x_limits)
