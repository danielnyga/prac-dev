# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2013 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import scipy.io as scipy
import os
import fnmatch
import numpy
from numpy.linalg import inv
import roslib
from trajectories import TrajReader
roslib.load_manifest('knowrob_motion_constraints')
import json_prolog
import rospy
import tf

from std_msgs.msg import Header
from std_msgs.msg import Time
#from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Pose
#from geometry_msgs.msg import Point
#from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from constraint_msgs.msg import Feature

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
# import matplotlib.pyplot as plt
import time
from itasc import *

if __name__ == '__main__':
    
    trajReader = TrajReader(os.path.join('.', 'data', 'scenario1'))
    
    # start json-prolog         
    rospy.init_node('tracking2knowrob', anonymous=True)
    knowrob = json_prolog.Prolog()
    
    # start ros node and broadcast all poses for each single frame
    pub = rospy.Publisher('handtracking', Feature)
    
    #rospy.init_node('handtracking')
    for frame_idx, frame in enumerate(trajReader.frames):
        rot_mats = frame['mats']
        timestamp = Time(time.time())
        for i, (obj, rot_mat) in enumerate(zip(trajReader.object_store, rot_mats)):
            if len(obj.features) == 0: continue
            #obj_pos = rot_mat[0:3,3]
            #obj_dir = rot_mat[0:3,0:3]
            for feature in obj.iterFeaturePoses(rot_mat):
                msg_frame = 'ref_frame_name'
                pos_vect = Vector3(float(feature.pos[0]), float(feature.pos[1]), float(feature.pos[2]))
                dir_vect = Vector3(float(feature.dir[0]), float(feature.dir[1]), float(feature.dir[2]))
                contactdir_vect = Vector3(0, 0, 0)
                feat_msg = Feature(msg_frame, feature.type, feature.name, pos_vect, dir_vect, contactdir_vect)
                print "publishing... "
                print feat_msg
                pub.publish(feat_msg)
                #header = Header(frame_idx, timestamp, feature.name)
                #pos_coord = map(lambda x: '%d'%x[0], feature.pos.tolist())
                #position = Point(pos_coord[0], pos_coord[1], pos_coord[2])
                #dir_coord = map(lambda x: '%d'%x[0], feature.dir.tolist())
                #print dir_coord
                #orientation = Quaternion(dir_coord[0], dir_coord[1], dir_coord[2], dir_coord[3])
                #pose = Pose(position, orientation)
        rospy.sleep(0.5)

#    fig = plt.figure()
#    ax = fig.gca(projection='3d')
#    x1=[]
#    y1=[]
#    z1=[]
#    x2=[]
#    y2=[]
#    z2=[]
#    for frame_idx, frame in enumerate(frames):
#        rot_mats = frame['mats']
#        
#        rot_mat1 = rot_mats[objects.index('spatula.obj_0')]
#        x1.append(rot_mat1[0,3])
#        y1.append(rot_mat1[1,3])
#        z1.append(rot_mat1[2,3])
#        
#        rot_mat2 = rot_mats[objects.index('pancake.obj_0')]
#        x2.append(rot_mat2[0,3])
#        y2.append(rot_mat2[1,3])
#        z2.append(rot_mat2[2,3])
#        
#    ax.plot(x1, y1, z1, color='red' )
#    ax.plot(x2, y2, z2, color='blue' )
#    
#    plt.show()
#        
        

    
