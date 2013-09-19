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

import logging
import praclog

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
    
    def __len__(self):
        return len(self.points)

    def __getitem__(self, i):
        return self.points[i]
    
    def __getslice__(self, i, j):
        return self.points[i:j]
    
    def getMaxXYZ(self):
        '''
        Returns the maximum values in x, y, z directions.
        '''
        return [max(self.getProjection(0)), max(self.getProjection(1)), max(self.getProjection(2))]



        
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


    