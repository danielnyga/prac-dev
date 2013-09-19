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

import numpy
import logging
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import mpl_toolkits.mplot3d as a3
import matplotlib.animation as animation
import math
from matplotlib import colors
import scipy

class ITaSCObject():
    '''
    Class representing a tracked object, which has a certain amount
    of iTaSC features (i.e., lines, planes or points)
    '''
    
    def __init__(self, name, features=None):
        self.name = name
        if features is None:
            self.features = []
        else:
            self.features = features
        
    def iterFeaturePoses(self, rot_mat):
        for feature in self.features:
            pos = numpy.vstack((feature.pos,[1.0],))
            pos = (rot_mat * pos)[:3]
            dir = rot_mat[0:3,0:3].transpose() * feature.dir
            yield ITaSCFeature(feature.name, feature.type, pos, dir)
            
    def __repr__(self):
        return 'iTaSC-Obj: %s' % self.name
    
    
class ITaSCFeature():
    '''
    Represents an iTaSC feature and provides methods for transforming
    its coordinates relative to the parent object.
    '''
    LINE = 0
    PLANE = 1
    POINT = 2    
    NAMES = {0: 'line', 1: 'plane', 2: 'point'}
    
    def __init__(self, name, type, pos, ori):
        '''
        Initializes an iTaSC feature:
        - name:    abstract identifier for the feature
        - type:    the type of the feature (i.e. LINE, PLANE, POINT)
        - pos:     the position vector of the feature
        - ori:     the orientation vector of the feature/
        '''
        self.name = name
        self.type = type
        self.pos = pos
        self.ori = ori
        

class ITaSCScene(object):
    '''
    Represents a scene consisting of multiple iTaSC objects
    and their motion trajectories over time.
    '''
    
    def __init__(self, objects=None, traj=None):
        '''
        - objects:    list of ITaSC objects
        - traj:       list of trajectories to be assigned to the objects 
                      (must be in the same order as objects.
        '''
        if objects is not None and traj is not None and len(objects) != len(traj):
            logging.getLogger('iTaSC').exception('iTaSC objects and trajectories do not match!')
        if objects is None:
            self.objects = []
        else:
            self.objects = objects
        if traj is None:
            self.obj2traj = {}
        else:
            self.obj2traj = dict([(obj, trj) for (obj, trj) in zip(objects, traj)])
        self.sceneLen = None
        if traj is not None:
            self.sceneLen = len(traj[0])
            assert not any(map(lambda x: len(x) != self.sceneLen, traj))
        
    def addTrajectoryForObject(self, obj, trj):
        '''
        Assigns the given trajectory trj to the respective object in obj.
        '''
        if obj not in self.objects:
            self.objects.append(obj)
        self.obj2traj[obj] = trj
        if self.sceneLen is None:
            self.sceneLen = len(trj)
        if not self.sceneLen == len(trj):
            logging.getLogger('iTaSC').exception('Trajectory data invalid.')
        
    def iterFrames(self):
        '''
        Iterates over the frames of this scene.
        '''
        for i in range(self.sceneLen):
#             logging.getLogger('iTaSC').debug('frame %d/%d' % (i, self.sceneLen))
            yield dict([(obj, self.obj2traj[obj][i]) for obj in self.objects])
        
    def iterFramesCumulative(self):
        '''
        Iterates over the frames of this scene.
        '''
        for i in range(self.sceneLen):
#             logging.getLogger('iTaSC').debug('frame %d/%d' % (i, self.sceneLen))
            yield dict([(obj, self.obj2traj[obj][:i+1]) for obj in self.objects])
    
class ITaSCAnimation():
    '''
    Represents an animation wrapper around iTaSC scenes.
    '''
    
    
    def __init__(self, scene):
        self.scene = scene
        self.fig = plt.figure()
        self.obj2point = {}
        self.log = logging.getLogger('iTaSC')
#         self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax = p3.Axes3D(self.fig)
        self.data = self.scene.iterFrames()
        self.anim = animation.FuncAnimation(self.fig, 
                                            self.animate,
                                            25, 
                                            interval=50, 
                                            init_func=self.init_anim,
                                            blit=False)

    def init_anim(self):
        obj2trj = next(self.data)
        self.obj2line = {}
        for obj in self.scene.objects:
            rot_mat = obj2trj[obj]
#             self.log.debug(rot_mat)
#             self.log.debug(np.array([1., 1., 1.]).transpose())
            start_position = rot_mat[0:3,3]
            mat = rot_mat[0:3, 0:3]
            orientation = np.dot(mat, [100,100,100])
            end_position = start_position + orientation
            end_position = np.add(start_position, orientation)
#             self.log.debug('end: %s' % str(end_position))
#             self.obj2line[obj] = self.ax.plot([start_position[0], end_position[0]], 
#                                               [start_position[1], end_position[1]],
#                                               [start_position[2], end_position[2]])[0]
#             coll = a3.art3d.Poly3DCollection([self.getCircleCoords(rot_mat)])
#             coll.set_edgecolor('k')
#             self.ax.add_collection3d(coll)
#             col = colors.rgb2hex(scipy.rand(4))
#             self.log.debug('%s: %s' % (obj, col))
#             coll.set_color(col)
            coord = self.getSphereCoords(rot_mat)
            coll = self.ax.plot_wireframe(coord[0], coord[1], coord[2], color="r")
            self.obj2line[obj] = coll
        return map(lambda x: self.obj2line[x], self.scene.objects)

    
    def animate(self, i):
        log = logging.getLogger('iTaSC')
        obj2trj = next(self.data)
        for obj in obj2trj:
            rot_mat = obj2trj[obj]
            start_position = rot_mat[0:3,3]
            mat = rot_mat[0:3, 0:3]
            orientation = np.dot(mat, [100,100,100])
            end_position = start_position + orientation
#             line = self.obj2line[obj]
#             line.set_data([start_position[0], end_position[0]], 
#                           [start_position[1], end_position[1]])
#             line.set_3d_properties([start_position[2], end_position[2]])
            coll = self.obj2line[obj]
#             coll.set_verts([self.getCircleCoords(rot_mat)])
            c = self.getSphereCoords(rot_mat)
            self.log.debug(len(c))
#             coll.set_segments(c)
        return map(lambda x: self.obj2line[x], self.scene.objects)

    def getCircleCoords(self, rot_mat, radius=20):        
        coords = []
        segments = 50
        alpha_d = (2. * math.pi)/segments
        for i in range(segments):
            alpha = alpha_d * i
            x = math.cos(alpha)
            y = math.sin(alpha)
            z = 1.
            v = np.dot(rot_mat[0:3, 0:3], np.array([x, y, z]))
            v *= radius
            v = v + rot_mat[0:3, 3]
            coords.append(v)
        return coords
    
    def getSphereCoords(self, rot_mat):
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        x = np.cos(u) * np.sin(v) + rot_mat[0,3]
        y = np.sin(u) * np.sin(v) + rot_mat[1,3]
        z = np.cos(v) + rot_mat[2,3]
        return x, y, z

        
    def run(self):
        '''
        Runs the animation in a separate window.
        '''
        plt.show()

    
if __name__ == '__main__':
#     logging.getLogger('traj').info('running.')
    from reader import TrajReader

    reader = TrajReader('../data/scenario2')
    objs = reader.obj2traj.keys()
    scene = ITaSCScene(objs, map(lambda x: reader.obj2traj[x], objs))
    anim = ITaSCAnimation(scene)
    anim.run()
     