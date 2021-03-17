#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import sys
import threading
import time
from collections import defaultdict

import rosgraph
import roslib.message
import roslib.names
import rospy

from rqt_py_common import topic_helpers

from actionlib_msgs.msg import GoalStatusArray

class StatusTrackerException(Exception):
    pass

class StatusTracker(object):

    """
    Subscriber to actionlib_msgs/GoalStatusArray topic that buffers incoming data
    """

    def __init__(self, topic, start_time):
        self.name = topic
        self.start_time = start_time
        self.error = None

        self.lock = threading.Lock()
        self.buffer = defaultdict(list)
        
        # sanity check we're given something reasonable
        assert(self.is_valid(topic))

        self.sub = rospy.Subscriber(topic, GoalStatusArray, self._ros_cb)

    @staticmethod
    def is_valid(topic_name):
        """ Returns True if the topic is a actionlib_msgs/GoalStatusArray, False if not.
        """
        topic_type, _, _ = topic_helpers.get_topic_type(topic_name)
        if topic_type == "actionlib_msgs/GoalStatusArray":
            return True, ""
        else:
            return False, "Topic {} is not an 'actionlib_msgs/GoalStatusArray'".format(topic_type)

    def close(self):
        self.sub.unregister()

    def _ros_cb(self, msg):
        """
        ROS subscriber callback
        :param msg: GoalStatusArray callback
        """
        # skip empty messages
        if len(msg.status_list) == 0:
            return

        # parse out the latest statuses into a dictionary
        stamp = msg.header.stamp.to_sec() - self.start_time
        data = {}
        for status in msg.status_list:
            sent_stamp = status.goal_id.stamp.to_sec() - self.start_time 
            data[status.goal_id.id] = (stamp, status.status, sent_stamp)

        # add these messages to our queue
        with self.lock:
            for k,v in data.items():
                self.buffer[k].append(v)

    def next(self):
        """
        Get the next data in the series. This is each item in the given
         status list along with a corresponding list of timestamps.

        :returns: {goal_id: [[timestamps], [status]}
        """
        if self.error:
            raise self.error

        with self.lock:
            buffer = dict(self.buffer)
            self.buffer.clear()

        return buffer
