#!/usr/bin/env python

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

import os
import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

import rospy

from rqt_py_common import topic_helpers

from action_server_monitor.goal_status.status_completer import StatusCompleter
from action_server_monitor.goal_status.statusplot import StatusData, StatusPlotException
from action_server_monitor.widgets.data_plot import DataPlot

class PlotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self):
        # initialize base widget class
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')

        # load base UI
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('action_server_monitor'), 'resource', 'plot.ui')
        loadUi(ui_file, self)

        # update button pictures
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('list-add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))

        # update button initial states
        self._remove_topic_menu = QMenu()
        self.subscribe_topic_button.setEnabled(False)

        # initialize StatusCompleter: helps auto-correct valid topics for the user.
        #  and add to the edit line
        self._status_completer = StatusCompleter(self.topic_edit)
        self._status_completer.update_topics()
        self.topic_edit.setCompleter(self._status_completer)

        # initialize some other objects
        self._start_time = rospy.get_time()
        self._statusdata = {}

        # initialize our embedded plot widget (the actual lines)
        self.data_plot = DataPlot(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._status_completer.update_topics()

        # check if this is a valid topic
        plottable, message = StatusData.is_valid(topic_name)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscroll_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)
        if checked:
            self.data_plot.redraw()

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    def _subscribed_topics_changed(self):
        # update relevant widgets
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._statusdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._statusdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._statusdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def update_plot(self):
        """ Core update callback; called on a Timer.
        """
        needs_redraw = False
        for topic_name, rosdata in self._statusdata.items():
            try:
                data_x, data_y = rosdata.next()
                if data_x or data_y:
                    self.data_plot.update_values(topic_name, data_x, data_y)
                    needs_redraw = True
            except StatusPlotException as e:
                qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
        if needs_redraw:
            self.data_plot.redraw()

    def add_topic(self, topic_name):
        """ Attempt to add a new topic to the tracked list.
        """
        # check if we're already tracking this message
        if topic_name in self._statusdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            return

        self._statusdata[topic_name] = StatusData(topic_name, self._start_time)
        if self._statusdata[topic_name].error is not None:
            qWarning(str(self._statusdata[topic_name].error))
            del self._statusdata[topic_name]
        else:
            data_x, data_y = self._statusdata[topic_name].next()
            self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
            self._subscribed_topics_changed()

    def remove_topic(self, topic_name):
        """ Remove the given topic from our tracked list.
        """
        self._statusdata[topic_name].close()
        del self._statusdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clear_plot(self):
        """ Clear the plot and start from scratch.
        """
        for topic_name in self._statusdata.keys():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        """ Shut down all subscriber objects.
        """
        for topic_name, status_subscriber in self._statusdata.items():
            status_subscriber.close()
            self.data_plot.remove_curve(topic_name)
        self._statusdata = {}
        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        """ Start or Stop plot updating.
        """
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
