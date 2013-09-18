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
from itasc import ITaSCObject, ITaSCFeature
import roslib
roslib.load_manifest('json_prolog')
import json_prolog
import logging
import praclog
import rospy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Trajectory(object):
    '''
    Represents an abtract trajectory given by a multidimensional
    sequence of points.
    '''
    
    def __init__(self, points=None):
        if points is None:
            self.points = []
        else:
            self.points = points
            
    def iterpoints(self):
        '''
        Iterates over the all points.
        '''
        for p in self.points:
            yield p
            
    def getProjection(self, dim):
        '''
        Returns a projection of this trajectory in the specified dimension dim.
        '''
        if len(self.points) == 0:
            logging.getLogger('traj').exception('Trajectory is empty.')
        elif len(self.points[0]) <= dim:
            logging.getLogger('traj').exception('Specified dimension (%d) exceeds the dimensionality of the trajectory (%d)' % (dim, len(self.points[0])))
        proj = []
        for p in self.points:
            proj.append(p[dim])
        return proj 

class TrajReader(object):
    '''
    Loads trajectories from a file and provides methods for accessing them.
    '''
    
    concept_map = {'plate.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#DinnerPlate'",
               'spatula.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Spatula'",
               'pancake.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Pancake'",
               'mixer.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#PancakeMaker'"}
    
    def __init__(self, path):
        self.log = logging.getLogger('traj')
        self.loadScene(path)
        
    def loadScene(self, path):
        '''
        Loads a scene.
        '''
        # extract the frames
        self.frames = map(lambda f: scipy.loadmat(os.path.join(path, f)), sorted(fnmatch.filter(os.listdir(path), 'frame*.mat')))
        
#         for f in self.frames:
#             print f
        
        '''
        'issue_id' is an integer and refers to the type of an object
        'instance_id' is an integer and refers to the index of an instance
        for a given issue_id. issue_id=2, instance_id=1 means the the first instance of the second type
        '''
        
        # extract object classes from data
        meshes = scipy.loadmat(os.path.join(path, 'mesh_filenames'))['filenames']
        idx2type = {}
        for i, m in enumerate(meshes):
            for p in m: break
            obj = p[0].split('\\')[-1]
            self.log.debug('found mesh: %s with idx: %d' % (str(obj), i))
            idx2type[i] = obj
        
        assert len(self.frames) > 0
        
        # extract objects    
        object2type = {}
        object2matIdx = {}
        matIdx2object = {}
        self.log.debug('Read %d objects in scene.' % len(self.frames[0]['instance_ids']))
        for i, (instIdx, typeIdx) in enumerate(zip(self.frames[0]['instance_ids'], self.frames[0]['issue_ids'])):
            for typeIdx in typeIdx: typeIdx -= 1 # do this since indices in matlab are 1-based
            for instIdx in instIdx: instIdx -= 1
            obj_name = '%s_%d' % (idx2type[typeIdx], instIdx)
            object2type[obj_name] = idx2type[typeIdx]
            object2matIdx[obj_name] = i
            matIdx2object[i] = obj_name

        # the object2type dict also contains the hand. in a separate obect store
        # we store only interesting objects that can be mapped to knowrob concepts.            
        self.itascObjStore = {} # maps the object name to its corresponding itasc object
        for o in object2type:
            o_type = object2type[o]
            if not o_type in TrajReader.concept_map: 
                self.log.debug('skipping %s (idx: %d) since not mapped to knowrob' % (o_type, object2matIdx[o]))
                continue
            obj = ITaSCObject(o)
            self.itascObjStore[obj.name] = obj
            self.log.debug('Object: %s (idx: %d)' % (obj, object2matIdx[o]))
        
        self.log.debug(self.itascObjStore)

        # extract the trajectories
        self.obj2traj = {}
        for _, frame in enumerate(self.frames):
            rot_mats = frame['mats']
            for obj in self.itascObjStore.values():
                rot_mat = rot_mats[object2matIdx[obj.name]]
                position = rot_mat[0:3,3].tolist()
                # obj_dir = rot_mat[0:3,0:3]
                traj = self.obj2traj.get(obj, Trajectory())
                self.obj2traj[obj] = traj
                traj.points.append(position)
                
        for o, t in self.obj2traj.iteritems():
            self.log.debug('obj: %s %s' % (o, str(t.points)))
            
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        colors = ['red', 'blue', 'green', 'orange', 'yellow']
        for c, (o, t) in enumerate(self.obj2traj.iteritems()):
            x = t.getProjection(0)
            y = t.getProjection(1)
            z = t.getProjection(2)
            self.log.debug('%s: %s' % (colors[c], o))
            ax.plot(x, y, z, color=colors[c])
        
        plt.show()

        
        
        # retrieve object features from knowrob 
#         rospy.init_node('pracmotions', anonymous=True)
#         knowrob = json_prolog.Prolog()
#         self.object_store = []
#         for o in self.objects:
#             o_type = self.object2type[o]
#             obj = ITaSCObject(o)
#             self.object_store.append(obj)
#             if not o_type in TrajReader.concept_map: continue
#             self.log.debug('Object: %s' % obj.name)
#             pl_query = "object_feature(%s, ITaSCFeatureT, Pos, Dir)" % TrajReader.concept_map[o_type]
#             self.log.error(pl_query)
#             
#             query = knowrob.query(pl_query)
#             counters = [1] * 3
#             for s in query.solutions():
#                 featureT = s['ITaSCFeatureT']
#                 pos = numpy.matrix(s['Pos']).transpose()
#                 dir = numpy.matrix(s['Dir']).transpose()
#                 feat_name = '%s/%s.%d'%(obj.name, ITaSCFeature.NAMES[featureT], counters[featureT])
#                 f = ITaSCFeature(featureT, feat_name, pos, dir)
#                 obj.features.append(f)
# #                 print '  ITaSCFeature: %s, %s, pos:%s, dir:%s' % (feat_name, ITaSCFeature.NAMES[featureT], str(pos), str(dir))
#                 counters[featureT] += 1
        
    
if __name__ == '__main__':
    logging.getLogger('traj').info('running.')
    reader = TrajReader('data/scenario5')
    