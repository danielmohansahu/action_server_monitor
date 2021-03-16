#! /usr/bin/env python
""" Creates a dummy action server that publishes various random statuses

Shamelessly copied from the actionlib_tutorials Tutorial.
"""

import random
import argparse
import rospy
import actionlib

import actionlib_tutorials.msg

class MockAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing')
        
        # start executing the action
        count = 0
        while not rospy.is_shutdown() and count <= goal.order:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            rospy.logdebug("Server sleeping...")
            rospy.sleep(random.random()*0.1)

            count += 1
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

def parse_args():
    parser = argparse.ArgumentParser(description='Mock Actionlib Server.')
    parser.add_argument("-s", "--server-name", type=str, default="fibonacci",
                        help='the action server name')
    args,_ = parser.parse_known_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    random.seed()

    rospy.init_node("mock_server")
    server = MockAction(args.server_name)
    rospy.spin()
 
