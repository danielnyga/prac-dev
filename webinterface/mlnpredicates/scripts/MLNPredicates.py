#!/usr/bin/env python

from mln_predicates.srv import *
from mln_predicates.msg import *
import rospy

from prac.core import PRAC

def handle_predicates_request(req):
	prac = PRAC()
	p = prac.mln.predicates
	myPredicates = []
	for name,domain in p.iteritems():
		myPredicates.append(MLNPredicate(name, domain))
    	return MLNPredicatesResponse(myPredicates)

def mln_predicates_server():
    rospy.init_node('mln_predicates_server')
    s = rospy.Service('MLNPredicates', MLNPredicates, handle_predicates_request)
    print "Ready to supply mln predicates."
    rospy.spin()

if __name__ == "__main__":
    mln_predicates_server()
