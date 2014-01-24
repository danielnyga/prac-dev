import roslib
roslib.load_manifest('json_prolog')
import rospy
import json_prolog

class Visualization:
    def __init__(self, instance_to_kr_mapping):
        """Load the knowrob visualization."""
        
        self._mapping = instance_to_kr_mapping
        
        rospy.init_node('annotation_tool_init_visual')
        self.prolog = json_prolog.Prolog()
        
        qstr = 'register_ros_package({pack})'.format(pack = 'mod_vis')
        res = self.prolog.query(qstr)
        res.finish()
        que = self.prolog.query("visualisation_canvas({canvas})".format(
                                                                canvas = "C"))
        que.finish()
        
        
    def show_instance(self, kr_instance, canvas="_"):
        """Show a kr instance in the visualization canvas."""
        
        instance = self._get_kr_instance_path_from_instance_name(kr_instance)
        query = "add_object('{obj}', {canvas})".format(obj=instance, 
                                                      canvas=canvas)
        res = self.prolog.query(query)
        res.finish()
    
    def _get_kr_instance_path_from_instance_name(self, kr_instance):
        """get the full kr instance name from the mln instance name."""
        
        for idx, value in self._mapping.iteritems():
            if kr_instance == value:
                return idx
        return None


if __name__=='__main__':
    mapping = {"http://de.tum.ias/kr/florian_kitchen.owl#Blender1": "Blender1"}
    vis = Visualization(mapping)
    vis.show_instance("Blender1")
    print "done"