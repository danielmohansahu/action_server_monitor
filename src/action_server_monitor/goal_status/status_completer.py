""" Modification of rqt_py_common.topic_completer.TopicCompleter to only look for actionlib topics.
"""

import roslib
import rospy

from python_qt_binding.QtCore import qWarning

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.tree_model_completer import TreeModelCompleter

from actionlib_msgs.msg import GoalStatusArray

class StatusCompleter(TreeModelCompleter):

  def __init__(self, parent=None):
    super(StatusCompleter, self).__init__(parent)
    self.setModel(MessageTreeModel())

  def splitPath(self, path):
    # to handle array subscriptions, e.g. /topic/field[1]/subfield[2]
    # we need to separate array subscriptions by an additional /
    return super(StatusCompleter, self).splitPath(path.replace('[', '/['))

  def update_topics(self):
    self.model().clear()
    topic_list = self.filter_non_actionlib(rospy.get_published_topics())
    for topic_path, topic_type in topic_list:
        topic_name = topic_path.strip('/')
        message_class = roslib.message.get_message_class(topic_type)
        if message_class is None:
            qWarning('StatusCompleter.update_topics(): '
                      'could not get message class for topic type "%s" on topic "%s"' %
                      (topic_type, topic_path))
            continue
        message_instance = message_class()
        self.model().add_message(message_instance, topic_name, topic_type, topic_path)

  def filter_non_actionlib(self, topic_list):
    # first, filter out non-actionlib topics
    actionlib_topics = []
    actionlib_suffixes = ("goal", "result", "feedback", "cancel", "status")

    for topic_path, topic_type in topic_list:
      (prefix,suffix) = topic_path.rsplit("/",1)
      if suffix in actionlib_suffixes:
        actionlib_topics.append([topic_path, topic_type])

    # then, check that all 5 topics are published (should be a much smaller list now)
    valid_topics = []
    for topic_path, topic_type in actionlib_topics:
      # ignore non-status messages
      (prefix,suffix) = topic_path.rsplit("/",1)
      if suffix == "status" and topic_type == "actionlib_msgs/GoalStatusArray":
        # check if there are all 4 of the other required topics being published
        valid = True
        for required_suffix in actionlib_suffixes[:-1]:
          if "/".join([prefix, required_suffix]) not in [path for path,_ in actionlib_topics]:
            valid = False
            break
        # if we're still valid, add it to the list
        if valid:
          valid_topics.append([topic_path, topic_type])

    # return the list of   
    return valid_topics

