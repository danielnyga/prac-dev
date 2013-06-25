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

class ITaSCObject():
    '''
    Class representing a tracked object, which has a certain amount
    of iTaSC features (i.e., line, plane or point)
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

class ITaSCFeature():
    '''
    Represents an iTaSC feature and provides methods for transforming
    its coordinates relative to the parent object.
    '''
    LINE = 0
    PLANE = 1
    POINT = 2    
    NAMES = {0: 'line', 1: 'plane', 2: 'point'}
    
    def __init__(self, name, type, pos, dir):
        self.name = name
        self.type = type
        self.pos = pos
        self.dir = dir
        
class ITaSCConverter():
    pass
    
    
    