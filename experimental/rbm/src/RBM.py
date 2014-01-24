'''
RESTRICTED BOLTZMANN MACHINES

(C) 2011-2012 by Daniel Nyga, Intelligent Autonomous Systems

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import random
import math
import sys
#from multiprocessing import Value
import operator
#from pydot import *

dotTempl = '''graph G {
{rank = same; %hidden_nodes%}
{rank = same; %visible_nodes%}
%nodes%
%edges%
}
'''

RANDOM = 1

class RBMWeight(object):
    '''
    This class represents the weight of an edge or the bias of a node 
    in a Restricted Boltzmann Machine. It is initialized by a random number in [-1,1]
    ''' 
    def __init__(self, w=None, delta_w=0):
        if w is None:
            self.w = (RBM.getRandomNumber() - .5) * 2
        else:
            self.w = w
        self.delta_w = delta_w#Value('d', delta_w)
        
    def __str__(self):
        return '%.3f' % self.w
        
class RBMEdge(object):
    '''
    Represents an edge between one hidden and one visible node.
    '''
    def __init__(self, weight=None, hiddenNode=None, visibleNode=None):
        self.hidden = hiddenNode
        self.visible = visibleNode
        if weight is None:
            self.weight = RBMWeight()
        else:
            self.weight = weight
            
    def __str__(self):
        return '%s <= %.3f => %s' % (str(self.visible), self.weight.w, str(self.hidden))

class RBMNode(object):
    '''
    Represents a binary visible or hidden node in an RBM.
    '''
    def __init__(self, idx, edges=None, bias=None, label=None):
        '''
        If value is not specified, the node is initialized randomly in {0,1}
        '''
        if edges is None:
            self.edges = []
        else:
            self.edges = edges
        if bias is None:
            self.bias = RBMWeight(w=0)
        else:
            self.bias = bias
        self.idx = idx
        if label is None:
            self.label = str(self.idx)
        else:
            self.label = label
        
    def __str__(self):
        return str(self.idx)
    
    def toDOTString(self, id, sample=None):
        if sample is None or sample.value is 0:
            color = 'white'
        else:
            color = 'lightgrey'
        dotStr = ('%s [label="'+self.label+'",fillcolor=%s,style=filled];\n')%(id,color)
        return dotStr
        
        
class RBMSample(object):
    '''
    Represents a sample of a hidden or visible RBMNode
    '''
    def __init__(self,value=0,prob=0):
        self.value = value
        self.prob = prob

class RBM(object):
    '''
    Represents a two-layered Restricted Boltzmann Machine
    '''
    def __init__(self, learningRate=1.0):
        self.visibleNodes = []
        self.hiddenNodes = []
        self.edges = []
        self.eta = learningRate
    
    @staticmethod
    def createRBM(learningRate, noOfVisibleNodes, noOfHiddenNodes):
        '''
        Returns an RBM object with the respective number of fully connected visible and hidden nodes.
        '''
        rbm = RBM(learningRate)
        rbm.visibleNodes = [RBMNode(i) for i in range(noOfVisibleNodes)]
        rbm.hiddenNodes = [RBMNode(i) for i in range(noOfHiddenNodes)]
        # create a fully connected RBM
        for v in rbm.visibleNodes:
            for h in rbm.hiddenNodes:
                e = RBMEdge(hiddenNode=h, visibleNode=v)
                h.edges.append(e)
                v.edges.append(e)
                rbm.edges.append(e)
        return rbm
    
    @staticmethod
    def createEdge(visibleNode,hiddenNode,weight=None):
        '''
        Returns an RBMEdge between visibleNode and hiddenNode. The respective RBMNodes
        are updated accordingly.
        '''
        e = RBMEdge(weight, hiddenNode, visibleNode)
        hiddenNode.edges.append(e)
        visibleNode.edges.append(e)
        return e
    
    def processExample(self, v1, n):
        '''
        Takes a vector of observations v1 and performs n contrastive divergence steps
        for updating the gradient of the weights.
        '''
        # Perform CD(n) on the nodes
        v = v1
        h1 = self.sampleHiddenNodes(v)
        h = h1
        for i in range(n):
            v = self.sampleVisibleNodes(h)
            if i < n-1: h = self.sampleHiddenNodes(v)
        self.h = h 
        self.v = v
        # Update the gradients
        for edge in self.edges:
            vID = edge.visible.idx
            hID = edge.hidden.idx
            edge.weight.delta_w += self.eta * (v1[vID].value * h1[hID].value - v[vID].value * h[hID].value) 
        for vNode in self.visibleNodes:
            vNode.bias.delta_w += self.eta * (v1[vNode.idx].value - v[vNode.idx].value)
        for hNode in self.hiddenNodes:
            hNode.bias.delta_w += self.eta * (h1[hNode.idx].value - h[hNode.idx].value)

#    def computeHiddenNodes(self, v):
#        '''
#        Takes a vector of RBMSamples of the visible nodes and returns
#        a vector of RBMSamples of the hidden nodes.
#        '''
#        hiddenNodes = []
#        for hiddenNode in self.hiddenNodes:
#            truthValue = True
#            newNode = None
#            for edge in hiddenNode.edges:
#                if truthValue == None:
#                    truthValue = v[edge.visible.idx].value
#                else:
#                    if not truthValue == v[edge.visible.idx]:
#                        newNode = RBMSample(0,0)
#            if newNode == None:
#                newNode = RBMSample(1,1)
#            hiddenNodes.append(newNode)
#        return hiddenNodes
    
    def equals(self,x,y):
        if x == y:
            return 1.
        else:
            return 0.

    def updateGradient(self):
        for edge in self.edges:
            edge.weight.w += edge.weight.delta_w
#            print edge.weight.delta_w
        for vNode in self.visibleNodes:
            vNode.bias.w += vNode.bias.delta_w
        for hNode in self.hiddenNodes:
            hNode.bias.w += hNode.bias.delta_w

    def eraseGradientUpdate(self):
        for edge in self.edges:
            edge.weight.delta_w = 0
        for vNode in self.visibleNodes:
            vNode.bias.delta_w = 0
        for hNode in self.hiddenNodes:
            hNode.bias.delta_w = 0

    @staticmethod
    def sigmoid(x):
        if abs(x) > 700:
            x = 700. * RBM.sgn(x)
        return 1.0 / (1.0 + math.exp(-x))
    
    @staticmethod
    def sgn(x):
        if x < 0:
            return -1
        elif x == 0:
            return 0
        else:
            return 1

    @staticmethod
    def getRandomNumber():
        return random.random()
        
    def sampleHiddenNodes(self, visibleSamples):
        '''
        Takes a vector of RBMSamples of the visible nodes and returns
        a vector of RBMSamples of the hidden nodes.
        '''
        hiddenSamples = []
        for hiddenNode in self.hiddenNodes:
            hiddenSamples.append(self.__sampleHiddenNode(hiddenNode, visibleSamples))
        return hiddenSamples

    def __sampleHiddenNode(self, hiddenNode, visibleSamples):
        '''
        Draws a random sample of one single hiddenNode.
        '''
        activation = hiddenNode.bias.w
        for edge in hiddenNode.edges:
            activation += visibleSamples[edge.visible.idx].value * edge.weight.w
        thr = RBM.sigmoid(activation)#1.0/len(hiddenNode.edges) * 
        if RBM.getRandomNumber() < thr:
            return RBMSample(1, thr)
        else:
            return RBMSample(0, thr)
        

    def sampleVisibleNodes(self, hiddenSamples):
        '''
        Takes a vector of RBMSamples of hidden nodes and returns
        a vector of RBMSamples of the visible nodes.
        '''
        visibleSamples = []
        for visibleNode in self.visibleNodes:
            visibleSamples.append(self.__sampleVisibleNode(visibleNode, hiddenSamples))
        return visibleSamples
    
    def __sampleVisibleNode(self, visibleNode, hiddenSamples):
        '''
        Draws a random sample of one single visibleNode.
        '''
        activation = visibleNode.bias.w
        for edge in visibleNode.edges:
            activation += hiddenSamples[edge.hidden.idx].value * edge.weight.w
        thr = RBM.sigmoid(activation)#1.0/len(visibleNode.edges) *
        if RBM.getRandomNumber() < thr:
            return RBMSample(1, thr)
        else:
            return RBMSample(0, thr)

    def reconstruct(self, x, steps, evidence=None):
        '''
        Takes a vector of RBMSamples representing a vector of observations
        and returns the RBMSample vector of the reconstructed vector.
        '''
        v = x
#        print [str(s.prob) for s in x]
        atomCounts = [0.] * len(self.visibleNodes)
        numSamples = 0.
        h1 = self.sampleHiddenNodes(v)
        h = h1
        for i in range(steps):
            v = self.sampleVisibleNodes(h)
            if i < steps-1: h = self.sampleHiddenNodes(v)
        return v
    
    def getRBMSamples(self, x):
        '''
        Takes a binary vector and converts it into a vector of RBMSamples
        '''
        return [RBMSample(v, v) for v in x]
    
#    def sampleBernoulli(self,p):
#        r = random.random()
#        if r < p: return 1
#        else: return 0
    
    def toDOT(self, filename, vsamples=None, hsamples=None):
        '''
        Writes a DOT image into the file filename.
        '''
        # node ranks
        dotString = dotTempl.replace('%hidden_nodes%', ';'.join(['H'+str(x.idx) for x in self.hiddenNodes]))
        dotString = dotString.replace('%visible_nodes%', ';'.join(['V'+str(x.idx) for x in self.visibleNodes]))
        # nodes
        if hsamples is not None:
            nodelist = ';\n'.join([n.toDOTString('H'+str(n.idx),hsamples[i]) for i,n in enumerate(self.hiddenNodes)])
        else:
            nodelist = ';\n'.join([n.toDOTString('H'+str(n.idx)) for n in self.hiddenNodes])
        nodelist += ';\n'
        if vsamples is not None:
            nodelist += ';\n'.join([n.toDOTString('V'+str(n.idx),vsamples[i]) for i,n in enumerate(self.visibleNodes)])
        else:
            nodelist += ';\n'.join([n.toDOTString('V'+str(n.idx)) for n in self.visibleNodes])
        dotString = dotString.replace('%nodes%', nodelist)
        # edges
        colors = ['red','green']
        dotString = dotString.replace('%edges%', ';\n'.join(['H'+str(e.hidden.idx)+'--'+'V'+str(e.visible.idx)+' [penwidth=%f,color=%s]'%(max(1,abs(e.weight.w)),colors[RBM.sgn(e.weight.w)/2+1]) for e in self.edges]))
        f = open(filename,'w')
        f.write(dotString)
        f.close()
       # g = graph_from_dot_file(filename)
       # svgData = g.create(format='svg')
       # f = open(filename, 'w')
       # f.write(svgData)
       # f.close()

def read(filename):
    f = open(filename)
    lines = f.readlines()
    samples = []
    example = []
    for l in lines:
        l = l.replace('\n', '')
        if len(l) == 0:
            samples.append(example)
            example = []
        s = l.replace('-','0')
        s = s.replace('$','1')
        s = map(lambda x: int(x), s)
        example.extend(s)
    f.close()
    return samples

def readableOut(samples):
    for s in samples:
        s = [str(x.value) for x in s]
        s = reduce(lambda x,y: x + y, s)
        s = s.replace('0','-')
        s = s.replace('1','$')
        for i, l in enumerate(s):
            if i % 8 == 0:
                print ''
            sys.stdout.write(l)
        print ''
    
if __name__ == '__main__':
    samples = read('data/ascii_training.txt')
    rbm = RBM.createRBM(.9, 64, 64)
    for i in range(100):
        for s in samples:
            rbm.eraseGradientUpdate()
            rbm.processExample([RBMSample(v,v) for v in s], 1)
            rbm.updateGradient()
#    print [x.weight.w for x in rbm.edges]
#    rbm.toDOT('../viz/rbm.svg')
    evidence = read('../data/ascii_evidence.txt')
    result = []
    for e in evidence:
        res = rbm.reconstruct([RBMSample(v,v) for v in e], 1)
        for r in res:
            print r.value
        result.append(res)
#        print result
    readableOut(result)
    