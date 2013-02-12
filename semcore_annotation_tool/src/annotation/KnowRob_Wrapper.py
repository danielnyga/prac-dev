'''
Created on Jul 24, 2012

@author: meyer
'''
"""
import os
import roslib
roslib.load_manifest('json_prolog')
import rospy
import threading

import json_prolog
from std_msgs.msg import String

class KnowRob(threading.Thread):
    '''
    Start roscore
    Make sure json_rosprolog is running with the following command:
    rosrun rosprolog run_with_prolog_env annotation_tool $(rospack find json_prolog)/bin/json_prolog
    best to use the startup script in utils/startup.sh
    '''

    def __init__(self, package, useReceivedData):
        '''
        map --- Path to an owl file that contains a semantic map, needs to be within the ias_semantic_map
        '''
        
        rospy.init_node('annotationTool_node')
        #Function handle to fill the click into the QT GUI
        self.useData = useReceivedData
        self.prolog = json_prolog.Prolog()
        self.packageName = roslib.packages.get_pkg_dir(package)
        #self.loadRosPackage('mod_vis')
        qstr = 'register_ros_package({pack})'.format(pack = 'mod_vis')
        res = self.prolog.query(qstr)
        res.finish()
        #qStr = "owl_parser:owl_parse('%s', false, false, true)" % (os.path.join(self.packageName, 'owl', map))
        #res = self.doIndividualQuery(qStr)
        que = self.prolog.query('visualisation_canvas(C)')
        que.finish()

    def run(self):
        self.listener()
    
    def listener(self):
        rospy.Subscriber("mod_vis/mod_vis_clicked", String, self.callback)
        rospy.spin()
        
    def callback(self, data):
        rospy.loginfo(rospy.get_name() + " Got msg: {data}".format(data = data.data))
        return data
        
    def loadRosPackage(self, package):
        qstr = 'register_ros_package({pack})'.format(pack = package)
        res = self.prolog.query(qstr)
        res.finish()
        return res
    
    def knowrobQuery(self, qry):
        Send a synchronous query.
        res = self.prolog.query(qry)
        res.finish()
        return res
    
#    def doIndividualQuery(self,userQuery):
Do an individual Prolog query to the knowrob database
#        
#        query = self.prolog.query(userQuery)
#        result = []
#        for r in query.solutions():
#            result.append(r)
#        query.finish()
#        return result
        
if __name__ == "__main__":
    kr = KnowRob('ias_semantic_map','ccrl2_semantic_map.owl')
    #kr.loadRosPackage('mod_vis')
    #kr.doIndividualQuery('visualisation_canvas(C)')
    print 'hallo'
    #kr.loadRosPackage('ias_semantic_map')
   # kr.doIndividualQuery('visualisation_canvas(C).')
   # res = kr.doIndividualQuery("owl_individual_of(A,knowrob:'SpatialThing')")
    #i = 0
    #for r in res:
    #    print "%s: %s"%(i,r)
     #   i+=1
     """