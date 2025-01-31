#! /usr/bin/env python
""" Mock Action Client

This client sends random goals and cancels (out of spite?).
"""

import argparse
import random
import rospy

import actionlib
import actionlib_tutorials.msg

class SpasticClient(object):

    def __init__(self, server_name):
        self._client = actionlib.SimpleActionClient(server_name, actionlib_tutorials.msg.FibonacciAction)
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError("Failed to connect to server {}".format(server_name))

    def sendGoal(self, block=False):
        goal = actionlib_tutorials.msg.FibonacciGoal(order=random.randint(0,100))
        self._client.send_goal(goal)

        if block:
            self._client.wait_for_result()

    def sendCancel(self):
        self._client.cancel_goal()

def parse_args():
    parser = argparse.ArgumentParser(description='Mock Actionlib Client.')
    parser.add_argument("-s", "--server-name", type=str, default="fibonacci",
                        help='the action server to connect with')
    args,_ = parser.parse_known_args()
    return args

if __name__ == '__main__':
    # get input args
    args = parse_args()

    # seed random number generator
    random.seed()

    # initialize node
    rospy.init_node('fibonacci_client_py')
    client = SpasticClient(args.server_name)

    # send random goals / cancels until shutdown
    while not rospy.is_shutdown():
        if random.random() > 0.75:
            # send a new goal
            rospy.loginfo("Sending new goal to {}".format(args.server_name))
            client.sendGoal(block = random.random() > 0.1)
        elif random.random() < 0.25:
            # send a cancel
            rospy.loginfo("Sending cancel to {}".format(args.server_name))
            client.sendCancel()

        # sleep some random amount of time
        rospy.sleep(random.random())
