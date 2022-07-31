#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(f, x):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(f, x)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        f = sys.argv[1]
        x = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s , %s"%(f,x))
    print("%s , %s"%(x , add_two_ints_client(f, x)))