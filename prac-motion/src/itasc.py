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
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class ITaSCObject():
    '''
    Class representing a tracked object, which has a certain amount
    of iTaSC features (i.e., lines, planes or points)
    '''
    log = logging.getLogger('iTaSC')
    
    def __init__(self, name, features=None):
        self.name = name
        if features is None:
            self.features = {}
        else:
            self.features = dict([(f.name, f) for f in features])
        self.rot_mat = None
        
    def iterGlobalFeaturePoses(self, rot_mat):
        '''
        Iterates over all features of this iTaSC object. Each feature
        is a copy of the original (which is relative to the object's origin),
        in global coordinates.
        ''' 
        for _, feature in self.features.iteritems():
#             pos = numpy.vstack((feature.pos,[1.0],))
            obj_pos = self.rot_mat[0:3,3]
            rot_mat = self.rot_mat[0:3,0:3]
            ITaSCObject.log.debug('%s%s %s' % (obj_pos, feature.pos, feature.drawable))
            pos = np.array([0, 0, 0]) + np.sum([np.dot(rot_mat.transpose(), v) for v in [[feature.pos[0], 0, 0], [0,feature.pos[1], 0], [0, 0, feature.pos[2]]]])
#             if feature.type == ITaSCFeature.LINE or feature.type == ITaSCFeature.PLANE:
#                 ori = np.sum([np.dot(rot_mat, v) for v in [[feature.ori[0], 0, 0], [0,feature.ori[1], 0], [0, 0, feature.ori[2]]]])
#             else:
#                 ori = None
            ori = feature.ori
            ITaSCObject.log.debug('%s' % str(feature.drawable))
            yield ITaSCFeature(feature.name, feature.type, pos, ori, drawable=feature.drawable)
            
    def initDrawing(self, axes):
        drawables = []
        ITaSCObject.log.debug('init drawing')
        for feat in self.iterGlobalFeaturePoses(self.rot_mat):
            ITaSCObject.log.debug('got feature pose: %s' % str(feat))
            drawable = feat.initDrawing(axes)
            self.features[feat.name].drawable = drawable
            if drawable is not None:
                drawables.append(drawable)
        return drawables
    
    def animateDrawing(self, axes):
        drawables = []
        for feat in self.iterGlobalFeaturePoses(self.rot_mat):
            drawable = feat.animateDrawing(axes)
            if drawable is not None:
                drawables.append(drawable)
        return drawables
            
            
    def __repr__(self):
        return 'iTaSC-Obj: %s' % self.name
    
    
class ITaSCFeature():
    '''
    Represents an iTaSC feature and provides methods for transforming
    its coordinates relative to the parent object.
    '''
    logger = logging.getLogger('iTaSC')
    
    LINE = 0
    PLANE = 1
    POINT = 2    
    ID2NAMES = {0: 'line', 1: 'plane', 2: 'point'}
    NAMES2ID = {'line': 0, 'plane': 1, 'point': 2}
    ARROW_SCALE = 50.
    
    def __init__(self, name, feat_type, pos, ori, drawable=None):
        '''
        Initializes an iTaSC feature:
        - name:    abstract identifier for the feature
        - type:    the type of the feature (i.e. LINE, PLANE, POINT)
        - pos:     the position vector of the feature (relative to its object's origin)
        - ori:     the orientation vector of the feature (relative to its object's origin)
        - drawable: (optional) reference to a drawable matplotlib thing.
        '''
        self.name = name
        if type(feat_type) == str:
            feat_type = ITaSCFeature.NAMES2ID[feat_type]
        self.type = feat_type
        self.pos = pos
        self.ori = ori
        self.drawable = drawable
    
    def __str__(self):
        return self.name
    
    def initDrawing(self, axis):
        '''
        Draws the feature. Returns the drawable object.
        '''
        ITaSCFeature.logger.debug('type: %s' % self.type)
        if self.type == ITaSCFeature.LINE:
            start_point = self.pos 
            end_point = np.array(start_point) + self.ori * ITaSCFeature.ARROW_SCALE
            a = Arrow3D([start_point[0], end_point[0]], [start_point[1], end_point[1]], 
                        [start_point[2], end_point[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="k")
            self.drawable = a
            ITaSCFeature.logger.debug('Drawing %s' % a)
            axis.add_artist(a)
            return self.drawable
    
    def animateDrawing(self, axes):
        '''
        Updates the feature animation.
        ''' 
        ITaSCFeature.logger.debug('type: %s, drawable: %s' % (ITaSCFeature.ID2NAMES[self.type], str(self.drawable)))
        if self.type == ITaSCFeature.LINE:
            start_point = self.pos 
            end_point = np.array(start_point) + self.ori * ITaSCFeature.ARROW_SCALE
            self.drawable._verts3d = [start_point[0], end_point[0]], [start_point[1], end_point[1]], \
                [start_point[2], end_point[2]]
        return self.drawable

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
        drawables = []
        for obj in self.scene.objects:
            rot_mat = obj2trj[obj]
            obj.rot_mat = rot_mat
#             self.log.debug(rot_mat)
#             self.log.debug(np.array([1., 1., 1.]).transpose())
#             start_position = rot_mat[0:3,3]
#             mat = rot_mat[0:3, 0:3]
#             orientation = sum(np.dot(mat, v) for v in [[1,0,0], [0,1,0], [0,0,1]])
#             end_position = start_position + orientation * ITaSCAnimation.ARROW_SCALE
#             end_position = np.add(start_position, orientation)
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
#             coord = self.getSphereCoords(rot_mat)
#             coll = self.ax.plot_wireframe(coord[0], coord[1], coord[2], color="r")
            drawables_tmp = obj.initDrawing(self.ax)
            drawables.extend(drawables_tmp)
#             self.obj2line[obj] = 
        self.ax.set_xbound(-500, 500)
        self.ax.set_ybound(-500, 500)
        self.ax.set_zbound(-500, 500)
        return drawables

    
    def animate(self, i):
        log = logging.getLogger('iTaSC')
        obj2trj = next(self.data)
        drawables = []
        for obj in obj2trj:
            rot_mat = obj2trj[obj]
            obj.rot_mat = rot_mat
            drawables.extend(obj.animateDrawing(self.ax))
        return drawables
#             start_position = rot_mat[0:3,3]
#             mat = rot_mat[0:3, 0:3]
#             orientation = sum(np.dot(mat, v) for v in [[1.,.0,.0], [.0,1.,.0], [.0,.0,1.]])
#             end_position = start_position + orientation * ITaSCAnimation.ARROW_SCALE
#             line = self.obj2line[obj]
#             line.set_data([start_position[0], end_position[0]], 
#                           [start_position[1], end_position[1]])
#             line.set_3d_properties([start_position[2], end_position[2]])
#             drawable = self.obj2line[obj]
            
#             coll.set_verts([self.getCircleCoords(rot_mat)])
            
#             c = self.getSphereCoords(rot_mat)
#             self.log.debug(len(c))
#             coll.set_segments(c)
#         return map(lambda x: self.obj2line[x], self.scene.objects)

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

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
    
if __name__ == '__main__':
#     logging.getLogger('traj').info('running.')
    from reader import TrajReader

    reader = TrajReader('../data/scenario3')
    objs = reader.obj2traj.keys()
    scene = ITaSCScene(objs, map(lambda x: reader.obj2traj[x], objs))
    anim = ITaSCAnimation(scene)
    anim.run()
     