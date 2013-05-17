'''
Created on Apr 22, 2013

@author: nyga
'''
import math
from random import random
from quadtree import QuadTree
from pyjamas.Timer import Timer
from datetime import datetime

def sign(x):
    if x < 0: return -1
    if x > 0: return 1
    return 0

class ForceLayout(object):
    '''
    Implementation of a gravity force layouting algorithm.
    '''
    
    def __init__(self, cog, graph, redraw, friction=.8, gravity=0.5, theta=.5):
        self.cog = cog # the center of gravity
        self.alpha = 0
        self.friction = friction
        self.gravity = gravity
        self.theta = theta
        self.graph = graph
        self.nodes = graph.nodes
        self.links = graph.links
        self.distances = None
        self.running = False
        self.timer = Timer(notify=self.tick)
        self.timestamp = None
        self.redraw = redraw
        self.width = 100
        self.height = 100
        
    def repulse(self, node, quad, x1, y1, x2, y2):
#         def repulse_(quad, x1, y1, x2, y2): 
        if quad.point != None and quad.point != node:
            # compute distance
            dx = quad.x - node.x
            dy = quad.y - node.y
            if dx == 0 and dy == 0:
                return True
            dn = 1 / math.sqrt(dx * dx + dy * dy)
            dn = min(100,dn)
            # Barnes-Hut criterion:
            if (x2-x1) * dn < self.theta:
                k = quad.charge * dn * dn
                node.px -= dx * k
                node.py -= dy * k
                return True
            if quad.point != None:
                k = quad.pointCharge * dn * dn
                node.px -= dx * k
                node.py -= dy * k
        return not quad.charge
#         return repulse_
    
    def tick(self):
        # simulated annealing, basically
        now = datetime.now()
        self.alpha *= .99
        if self.alpha < .005:
            # end animation here
            return
        # gauss-seidel relaxation for links
        for i, l in enumerate(self.links):
            s = l.source
            t = l.target
            x = t.x - s.x
            y = t.y - s.y
            length = x * x + y * y
            if length != 0:
                l_ = math.sqrt(length)
                l = self.alpha * l.strength * (l_ - l.distance) / length
                x *= l
                y *= l
                k = s.weight / (t.weight + s.weight)
                if not t.fixed:
                    t.x -= x * k
                    t.y -= y * k
                k = 1 - k
                if not s.fixed:
                    s.x += x * k
                    s.y += y * k
#         print 'link computation took', (datetime.now() - now).microseconds
#         now = datetime.now()
        # compute the quadtree center of mass and apply mass forces
        if len(self.nodes) != 0:
            qt = QuadTree(self.nodes)
#             print 'quad tree computation took', (datetime.now() - now).microseconds
#             now = datetime.now()
            self.accumulate_forces(qt.root)
#             print 'force accumulation computation took', (datetime.now() - now).microseconds
#             now = datetime.now()
            for n in self.nodes:
                if not n.fixed:
                    qt.repulse(self, n, qt.root, qt.x1_, qt.y1_, qt.x2_, qt.y2_)
#             print 'repulse computation took', (datetime.now() - now).microseconds
#         now = datetime.now()
        # apply gravity forces
        x = self.cog[0]
        y = self.cog[1]
        k = self.alpha * self.gravity
        for n in self.nodes:
            if n.fixed:
                continue
            dx = (x - n.x)
            dy = (y - n.y)
            n.x += dx * k
            n.y += dy * k
            # apply boundary forces
            x_ = n.x
            y_ = n.y
            scale = 0.01
            scale2 = 3
            const = 50
            n.x += scale2 * math.exp(scale*(-x_+const))
            n.x -= scale2 * math.exp(scale*(-(self.width-x_-const))) 
            n.y += scale2 * math.exp(scale*(-y_+const)) 
            n.y -= scale2 * math.exp(scale*(-(self.height-y_-const))) 
#         print 'gravity computation took', (datetime.now() - now).microseconds
#         now = datetime.now()
        # position verlet integration
        for n in self.nodes:
            if not hasattr(n, 'px'): n.px = n.x
            if not hasattr(n, 'py'): n.py = n.y
            if n.fixed:
                continue
            x_ = n.x
            y_ = n.y
            n.x -= (n.px - x_) * self.friction
            n.y -= (n.py - y_) * self.friction
            n.px = x_
            n.py = y_
#         print 'application computation took', (datetime.now() - now).microseconds
#         now = datetime.now()
        self.redraw(self.graph)
#         print 'drawing took', (datetime.now() - now).microseconds
        self.timer.schedule(0.01)
                    
    def start(self):
        # start the simulation
        for i, n in enumerate(self.nodes):
            n.index = i
            n.weight = 0.
        # count number of links for each node
        for i, l in enumerate(self.links):
            l.source.weight += 1
            l.target.weight += 1         

        for n in self.nodes:
            if n.x is None: n.x = self.cog[0]#position('x', 900)
            if n.y is None: n.y = self.cog[1]#position('y', 900)
#             print n.x, n.y
            if not hasattr(n, 'px'): n.px = n.x
            if not hasattr(n, 'py'): n.py = n.y
        self.resume()
        
#     def initialPos(self, n):
#         return self.cog
        
    def setAlpha(self, val):
#         if val == 0:
#             self.timer.cancel()
        if self.alpha > 0:
            if val > 0: 
                self.alpha = val
            else: 
                self.alpha = 0
                self.timer.cancel()
        else:
            self.alpha = val
            self.tick()
            

    def resume(self):
        return self.setAlpha(.1)
    
    def stop(self):
        return self.setAlpha(0)
    
    def drag(self):
        pass
    
    def accumulate_forces(self, quad):
        cx = 0
        cy = 0
        quad.charge = 0
        if not quad.leaf:
            for n in quad.nodes.values():
                self.accumulate_forces(n)
                quad.charge += n.charge
                cx += n.charge * n.cx
                cy += n.charge * n.cy
        if quad.point != None:
            # jitter internal nodes that are coincident
            if not quad.leaf:
                quad.point.x += (random() - .5)
                quad.point.y += (random() - .5)
            k = self.alpha * quad.point.charge
            quad.pointCharge = k
            quad.charge += k
            cx += k * quad.point.x
            cy += k * quad.point.y
        quad.cx = cx / quad.charge
        quad.cy = cy / quad.charge
    