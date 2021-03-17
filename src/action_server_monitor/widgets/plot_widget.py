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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

import rospy

from action_server_monitor.goal_status.status_completer import StatusCompleter
from action_server_monitor.goal_status.status_tracker import StatusTracker, StatusTrackerException
from action_server_monitor.widgets.data_plot import DataPlot

class PlotWidget(QWidget):
    """ This QWidget handles the callbacks for all buttons in the GUI and passes callbacks to the Data class.
    """
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
        self._tracked_topics = {}

        # initialize our embedded plot widget (the actual lines)
        self.data_plot = DataPlot(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())
        self.data_plot.limits_changed.connect(self._limits_changed)

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._status_completer.update_topics()

        # check if this is a valid topic
        plottable, message = StatusTracker.is_valid(topic_name)
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

    def _limits_changed(self):
        # pause the system if any of the underlying limits have changed
        if not self.pause_button.isChecked():
            self.pause_button.click()

    def _subscribed_topics_changed(self):
        # update relevant widgets
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._tracked_topics)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._tracked_topics.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._tracked_topics) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def update_plot(self):
        """ Core update callback; called on a Timer.
        """
        needs_redraw = False
        for topic_name, tracker in self._tracked_topics.items():
            try:
                data = tracker.next()
                if data:
                    self.data_plot.update_values(topic_name, data)
                    needs_redraw = True
            except StatusTrackerException as e:
                qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
        if needs_redraw:
            self.data_plot.redraw()

    def add_topic(self, topic_name):
        """ Attempt to add a new topic to the tracked list.
        """
        # check if we're already tracking this message
        if topic_name in self._tracked_topics:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            return

        # if we're adding a new tracker (i.e. 0->1) we should reset the start time
        if len(self._tracked_topics) == 0:
            self._start_time = rospy.get_time()

        self._tracked_topics[topic_name] = StatusTracker(topic_name, self._start_time)
        if self._tracked_topics[topic_name].error is not None:
            qWarning(str(self._tracked_topics[topic_name].error))
            del self._tracked_topics[topic_name]
        else:
            data = self._tracked_topics[topic_name].next()
            self.data_plot.add_topic(topic_name, data)
            self._subscribed_topics_changed()

    def remove_topic(self, topic_name):
        """ Remove the given topic from our tracked list.
        """
        self._tracked_topics[topic_name].close()
        del self._tracked_topics[topic_name]
        self.data_plot.remove_topic(topic_name)

        self._subscribed_topics_changed()

    def clear_plot(self):
        """ Clear the plot and start from scratch.
        """
        self.data_plot.clear_values()
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        """ Shut down all subscriber objects.
        """
        for topic_name, status_subscriber in self._tracked_topics.items():
            status_subscriber.close()
            self.data_plot.remove_topic(topic_name)
        self._tracked_topics = {}
        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        """ Start or Stop plot updating.
        """
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
