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
    _add_curve = Signal(str, str, float, 'QColor', bool)

    def __init__(self, parent=None):
        """Create a new, empty DataPlot
        """
        super(DataPlot, self).__init__(parent)

        # initialize miscellaneous class variables
        self._color_index = 0
        self._scrollwidth = 10
        self._dropwidth = 60    # drop data this much older than our window's lower bound

        # determine whether or not to scroll through the window (in X)
        self._autoscroll = True

        # initialize limits
        # @TODO move static Y values elsewhere? Kinda breaks modularity...
        self._x_limits = [0, 1.0]
        self._static_ylimits = [-0.5, 9.5]
        
        # initialize currently tracked data ({topic: {goal_id: [timestamp, id]}})
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
        for topic_value in self._curves.values():
            for goal_value in topic_value["goals"].values():
                # here we check if the given curve should be dropped (i.e. if we have no data within our visible range for it)
                if numpy.max(goal_value['x']) < (self._x_limits[0] - self._dropwidth):
                    # delete this curve
                    self._data_plot_widget.remove_curve(goal_value["name"])
                else:
                    # update
                    self._data_plot_widget.set_values(goal_value["name"], goal_value['x'], goal_value['y'])
        self._data_plot_widget.redraw()

    def add_topic(self, topic_name, data):
        """Add a new topic to this plot.

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        topic_color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1

        # initialize topic object
        self._curves[topic_name] = {
            "goals": {},
            "name": topic_name,
            "color": topic_color}

        self.update_values(topic_name, data)

    def remove_topic(self, topic_name):
        """Remove the specified curve from this plot"""
        if topic_name in self._curves:
            # remove all associated goal curves
            for goal_id in self._curves[topic_name]["goals"].keys():
                self._data_plot_widget.remove_curve(goal_id)

            # delete this topic object
            self._curves.pop(topic_name)

    def update_values(self, topic_name, values, sort_data=True):
        """Append new data to an existing topic

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.

        If `sort_data` is set to False, values won't be sorted by `values_x`
        order.
        """
        # ignore if we aren't tracking this topic
        if topic_name in self._curves:
            # iterate through all the new goals and data
            for goal_id, data in values.items():
                # parse data
                new_x, new_y, time_sent = [list(v) for v in zip(*data)]
                time_sent = max(time_sent)

                # for each goal in the new values we need to check if it's new and therefore
                #  warrants adding a new curve
                if goal_id not in self._curves[topic_name]["goals"]:
                    self._curves[topic_name]["goals"][goal_id] = {
                        "x": numpy.array([]),
                        "y": numpy.array([]),
                        "name": goal_id,
                        "sent": time_sent,
                        "color": self._curves[topic_name]["color"]
                    }
                    # update QT with knowledge of this new curve
                    self._add_curve.emit(topic_name, goal_id, time_sent, self._curves[topic_name]["color"], True)

                # check if this new curve has a newer X data point than we've previously seen
                #  we use this to update our upper bound
                curve_x_max = max(new_x)
                if curve_x_max > self._x_limits[1]:
                    self._x_limits[1] = curve_x_max

                # add new data to the given goal id's curve
                self._curves[topic_name]["goals"][goal_id]['x'] = numpy.append(
                    self._curves[topic_name]["goals"][goal_id]['x'], new_x)
                self._curves[topic_name]["goals"][goal_id]['y'] = numpy.append(
                    self._curves[topic_name]["goals"][goal_id]['y'], new_y)

                if sort_data:
                    # sort resulting data, so we can slice it later
                    sort_order = self._curves[topic_name]["goals"][goal_id]['x'].argsort()
                    self._curves[topic_name]["goals"][goal_id]['x'] = self._curves[topic_name]["goals"][goal_id]['x'][sort_order]
                    self._curves[topic_name]["goals"][goal_id]['y'] = self._curves[topic_name]["goals"][goal_id]['y'][sort_order]

    def clear_values(self):
        """Clear the values for all curves

        Note that the plot is not redrawn automatically; call `redraw()` to make
        any changes visible to the user.
        """
        # clear internal curve representation
        for topic_values in self._curves.values():
            for goal_values in topic_values["goals"].values():
                goal_values["x"] = numpy.array([])
                goal_values["y"] = numpy.array([])
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
