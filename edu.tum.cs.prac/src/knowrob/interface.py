'''
Created on Aug 16, 2012

@author: nyga
'''

import os
import roslib 
roslib.load_manifest('json_prolog')
import rospy
from std_msgs.msg import String
import json_prolog
import thread

class KnowRob(object):
    """
        An interface wrapper to json_prolog to the KnowRob KB. 
    """
    
    def __init__(self, rosNodeName):
        # Create a new ros node
        self.nodeName = rosNodeName
        rospy.init_node(rosNodeName, anonymous=True)
        self.prolog = json_prolog.Prolog()
    
    def registerForModVisClicked(self, callback):
        """
        Subscribes to the '/mod_vis_clicked' topic containing a string with
        the RDF identifier of the object clicked on in the semantic map visualization.
        """
#        def spinSubscriber(callback):
        rospy.Subscriber("/mod_vis_clicked", String, callback)
        rospy.spin()
#        try:
#            thread.start_new_thread(spinSubscriber, (callback,))
#        except:
#            print 'Unable to start new thread.'
    
    def loadOWLOntology(self, path):
        """
        Loads the OWL file at the path given into KnowRob.
        """
        return self.sendQuery("owl_parse('%s', false, false, true)" % path)
    
    def sendQuery(self, queryStr):
        """
        Sends an arbitrary Prolog query to KnowRob and
        returns an iterator over the solutions found.
        """
        query = self.prolog.query(queryStr)
        return query.solutions()
        
    def getIndividualsOfType(self, type):
        """
        Returns all individuals of a particular type.
        """
        query = self.prolog.query("owl_individual_of(A,%s)" % type)
        return query.solutions()
    
    def highlightIndividuals(self, objects):
        """
        Highlights the objects (given by RDF identifiers) in the semantic map visualization.
        """
        for o in objects:
            self.sendQuery('add_object(%s,$C), highlight_object(%s,$C)' % (o,) * 2)
    
    def openVisualization(self):
        return self.sendQuery("visualisation_canvas(C)")
    
if __name__ == '__main__':
    
    semanticMapPath = roslib.packages.get_pkg_dir('ias_semantic_map')
    knowrob = KnowRob('PRAC')
    knowrob.loadOWLOntology(os.path.join(semanticMapPath, 'owl', 'ccrl2_semantic_map.owl'))
    for s in knowrob.getIndividualsOfType("knowrob:'SpatialThing'"):
        print s
    knowrob.openVisualization()
    
    
#    
#    qStr = "owl_parse('%s', false, false, true)" % (os.path.join(tutName, 'owl', 'ccrl2_map_objects.owl'))
#    print qStr
#    query = prolog.query(qStr)
#    for sol in query.solutions():
#        print sol
#    
##    query = prolog.query("use_ros_module('mod_vis', 'mod_vis')")
##    for sol in query.solutions():
##        print sol
#    query = prolog.query("visualisation_canvas(C),owl_individual_of(A,knowrob:'SpatialThing-Localized'),add_object(A,C)")
#    
#    query = prolog.query("owl_individual_of(A,knowrob:'Milk'),highlight_object(A,$C)")
#    
#    for sol in query.solutions():
#        print sol
#    
#    query.finish()