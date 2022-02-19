#!/usr/bin/env python3

from __future__ import print_function

import rospy
import sys


from action_tutorial.srv import CounterGoal, CounterResult
from action_tutorial.msg import Counter

ASSIGNMENT = True

def usage():
    str = "%s <counter_end_value>"%sys.argv[0]
    return str if not ASSIGNMENT else str + " <counter_start_value> <rate_of_counting>"


def counter_feedback_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "the value of the counter is currently %d", msg.counter_value)


def setup_node():
    rospy.init_node("counter_client", anonymous=True)
    rospy.Subscriber("/feedback_topic", Counter, counter_feedback_callback)


def counter_goal_client(count,start,rate):
    rospy.wait_for_service("counter_goal_server")

    try:
        counterHandle = rospy.ServiceProxy("counter_goal_server", CounterGoal)
        goalResponse = counterHandle(count,start,rate)
        if not goalResponse.result:
            print("Server failed to accept goal request!")
        return goalResponse.result
    except rospy.ServiceException as e:
        print("Service call for goal failed: %s"%e)
        return False


def counter_result_client():
    rospy.wait_for_service("counter_result_server")

    try:
        resultHandle = rospy.ServiceProxy("counter_result_server", CounterResult)
        resultResponse = resultHandle(True)
        print("Server has finished till %d..."%resultResponse.counted)
        return resultResponse.counted
    except rospy.ServiceException as e:
        print("Service call for result failed: %s"%e)
        return False


def main():
    if len(sys.argv) == 4:

        count = int(sys.argv[1])
        start = int(sys.argv[2])
        rate = float(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)
    
    setup_node()
    print("Requesting a count to %d starting from %d at the rate of %f"%(count,start,rate))

    if counter_goal_client(count,start,rate):
        counter_result_client()
        rospy.spin()
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
    