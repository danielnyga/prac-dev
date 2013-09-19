'''
Created on Sep 18, 2013

@author: nyga
'''
import logging
import os
import fnmatch
import scipy.io as scipy
from itasc import ITaSCObject
import matplotlib.pyplot as plt
from trajectories import Trajectory
from numpy.linalg import inv


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
                rot_mat[0:3,0:3] = inv(rot_mat[0:3, 0:3]).transpose()
                position = rot_mat[0:3,3].tolist()
                orientation = rot_mat[0:3,0:3]
                traj = self.obj2traj.get(obj, Trajectory())
                self.obj2traj[obj] = traj
                traj.points.append(rot_mat)
                
#         for o, t in self.obj2traj.iteritems():
#             self.log.debug('obj: %s %s' % (o, str(t.points)))
            
#         fig = plt.figure()
#         ax = fig.gca(projection='3d')
#         colors = ['red', 'blue', 'green', 'orange', 'yellow']
#         for c, (o, t) in enumerate(self.obj2traj.iteritems()):
#             x = t.getProjection(0)
#             y = t.getProjection(1)
#             z = t.getProjection(2)
#             self.log.debug('%s: %s' % (colors[c], o))
#             ax.plot(x, y, z, color=colors[c])
#         
#         plt.show()

        