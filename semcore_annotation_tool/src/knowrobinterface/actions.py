'''
Created on Jul 24, 2012

@author: meyer
'''
import os
import roslib
roslib.load_manifest('json_prolog')
import rospy

import json_prolog
from std_msgs.msg import String
from PyQt4 import QtCore, QtGui, uic, Qt

class KnowRob(QtCore.QThread):
    '''
    Start roscore
    Make sure json_rosprolog is running with the following command:
    rosrun rosprolog run_with_prolog_env annotation_tool 
                                    $(rospack find json_prolog)/bin/json_prolog
    '''

    def __init__(self, package, parent = None):
        """
        map --- Path to an owl file that contains a semantic map, 
        needs to be within the ias_semantic_map
        
        """
        QtCore.QThread.__init__(self, parent)
        rospy.init_node('annotationTool_node')
        #Function handle to fill the click into the QT GUI

        self.prolog = json_prolog.Prolog()
        self.packageName = roslib.packages.get_pkg_dir(package)
        #self.loadRosPackage('mod_vis')

        
        #Laod my kitchen
        qstr = """owl_parse('/usr/stud/meyer/ontologies/florian_kitchen/
                    florian_kitchen.owl',false, false, true)"""
        res = self.prolog.query(qstr)
        res.finish()
    
    def run(self):
        self.listener()
    
    def listener(self):
        rospy.Subscriber("mod_vis/mod_vis_clicked", String, self.callback)
        rospy.spin()
        
    def callback(self, data):
        rospy.loginfo(rospy.get_name() + 
                      " Got msg: {data}".format(data = data.data))
        self.emit(QtCore.SIGNAL("updateKnowRobList(QString)"), data.data)
        
        
    def loadRosPackage(self, package):
        qstr = 'register_ros_package({pack})'.format(pack = package)
        res = self.prolog.query(qstr)
        res.finish()
        return res
    
    def knowrobQuery(self, qry):
        """Send a synchronous query."""
        res = self.prolog.query(qry)
        outcome = [x for x in res.solutions()]
        res.finish()
        return outcome

        
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