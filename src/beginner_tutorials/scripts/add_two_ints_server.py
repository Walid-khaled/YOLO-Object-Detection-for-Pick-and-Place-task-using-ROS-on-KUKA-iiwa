#!/usr/bin/env python3

from __future__ import print_function

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

var = ['N', -1]

def handle_add_two_ints(req):
    print(req)
    print("Returning %s , %s"%(req.f, req.x))
    var[0] = req.f
    var[1] = req.x
    return AddTwoIntsResponse(req.x)


def add_two_ints_server():
    print("init")
    rospy.init_node('add_two_ints_server')
    print("After")
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rate = rospy.Rate(1) # 10hz
    print("Before while")
    while not rospy.is_shutdown():
        if(var[0] == 'S'):
            print('Sum mode')
            if(var[1] != -1):
                print(var[1])
                var[1] = -1
        elif var[0] == 'M':
            print('Mull mode')
            if(var[1] != -1):
                print(var[1])
                var[1] = -1
        else:
            print('Null Mode')
        rate.sleep()

if __name__ == "__main__":
    try:
        add_two_ints_server()
    except rospy.ROSInterruptException:
        pass