#!/usr/bin/env python
import roslib; roslib.load_manifest('mln_predicates')

import sys

import rospy
from mln_predicates.srv import *
from mln_predicates.msg import *

def mln_predicates_client():
    rospy.wait_for_service('MLNPredicates')
    try:
        mln_predicates = rospy.ServiceProxy('MLNPredicates', MLNPredicates)
        resp1 = mln_predicates(None)
        return resp1.predicates
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    predicates = mln_predicates_client()
    for predicate in predicates:
	print predicate.name, " => ", predicate.domain
