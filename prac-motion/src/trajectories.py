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

import scipy
import os
import fnmatch
from itasc import ITaSCObject, ITaSCFeature
import roslib
roslib.load_manifest('json_prolog')
import json_prolog
import numpy

class TrajReader(object):
    '''
    Loads trajectories from a file and provides methods for accessing them.
    '''
    concept_map = {'plate.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#DinnerPlate'",
               'spatula.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Spatula'",
               'pancake.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Pancake'",
               'mixer.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#PancakeMaker'"}
    
    def __init__(self, path):
        self.loadScene(path)
    
    def loadScene(self, path):
        # extract the frames
        self.frames = map(lambda f: scipy.loadmat(path), sorted(fnmatch.filter(os.listdir(path), 'frame*.mat')))
        
        # extract object classes from data
        meshes = scipy.loadmat(os.path.join('path', 'mesh_filenames'))['filenames']
        self.types = []
        for m in meshes:
            for p in m: pass
            print p[0]
            obj = p[0].split('\\')[-1]
            self.types.append(obj)
            
        # extract objects    
        self.object2type = {}
        self.objects = []
        for inst, issue in zip(self.frames[0]['instance_ids'], self.frames[0]['issue_ids']):
            for inst_number in inst: inst_number -= 1
            for issue_number in issue: issue_number -= 1
            obj_name = '%s_%d' % (self.types[issue_number], inst_number)
            self.object2type[obj_name] = self.types[issue_number]
            self.objects.append(obj_name)
            
        # retrieve object features from knowrob 
        knowrob = json_prolog.Prolog()
        self.object_store = []
        for o in self.objects:
            o_type = self.objects2type[o]
            obj = ITaSCObject(o)
            self.object_store.append(obj)
            if not o_type in TrajReader.concept_map: continue
#             print 'Object: %s' % obj.name
            query = knowrob.query("object_feature(%s, ITaSCFeatureT, Pos, Dir)" % TrajReader.concept_map[o_type])
            counters = [1] * 3
            for s in query.solutions():
                featureT = s['ITaSCFeatureT']
                pos = numpy.matrix(s['Pos']).transpose()
                dir = numpy.matrix(s['Dir']).transpose()
                feat_name = '%s/%s.%d'%(obj.name, ITaSCFeature.NAMES[featureT], counters[featureT])
                f = ITaSCFeature(featureT, feat_name, pos, dir)
                obj.features.append(f)
#                 print '  ITaSCFeature: %s, %s, pos:%s, dir:%s' % (feat_name, ITaSCFeature.NAMES[featureT], str(pos), str(dir))
                counters[featureT] += 1
    
    