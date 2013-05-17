'''
Created on Apr 23, 2013

@author: nyga
'''
from random import random
from graph import Node

class QuadTreeNode(object):
   
    def __init__(self):
        '''
        x- and y-range being a tuple of mix-max values
        '''
        self.x = None
        self.y = None
        self.leaf = True
        self.point = None
        self.nodes = {}
    
class QuadTree(object):
    
    def __init__(self, points):
        self.x1_ = None
        self.y1_ = None
        self.x2_ = None
        self.y2_ = None
        xs = []
        ys = []
        for p in points:
            if self.x1_ is None or p.x < self.x1_: self.x1_ = p.x
            if self.y1_ is None or p.y < self.y1_: self.y1_ = p.y
            if self.x2_ is None or p.x > self.x2_: self.x2_ = p.x
            if self.y2_ is None or p.y > self.y2_: self.y2_ = p.y
            xs.append(p.x)
            ys.append(p.y)
        # squarify the bounds.
        dx = self.x2_ - self.x1_
        dy = self.y2_ - self.y1_
        if dx > dy: 
            y2_ = self.y1_ + dx
        else:
            x2_ = self.x1_ + dy
            
        self.root = QuadTreeNode()
        for i, p in enumerate(points):
            self.insert(self.root, p, xs[i], ys[i], self.x1_, self.y1_, self.x2_, self.y2_)
        
    def insert(self, n, d, x, y, x1, y1, x2, y2):
        
        if n.leaf:
            nx = n.x
            ny = n.y
            if nx != None:
                if (abs(nx - x) + abs(ny - y)) < .01:
                    self.insertChild(n, d, x, y, x1, y1, x2, y2)
                else:
                    nPoint = n.point
                    n.x = None
                    n.y = None
                    n.point = None
                    self.insertChild(n, nPoint, nx, ny, x1, y1, x2, y2)
                    self.insertChild(n, d, x, y, x1, y1, x2, y2)
            else:
                n.x = x
                n.y = y 
                n.point = d
        else:
            self.insertChild(n, d, x, y, x1, y1, x2, y2)
            
    def insertChild(self, n, d, x, y, x1, y1, x2, y2):
        sx = (x1 + x2) * .5
        sy = (y1 + y2) * .5
        right = (x >= sx)
        bottom = (y >= sy)
        i = (bottom << 1) + right
        
        n.leaf = False
        if i not in n.nodes.keys(): 
            new_node = QuadTreeNode()
            n.nodes[i] = new_node
            n = new_node
        else:
            n = n.nodes[i]
#         print self
        
        if right: x1 = sx
        else: x2 = sx
        if bottom: y1 = sy
        else: y2 = sy
        self.insert(n, d, x, y, x1, y1, x2, y2)

    def repulse(self, layout, node, quad, x1, y1, x2, y2):
        
        if not layout.repulse(node, quad, x1, y1, x2, y2):
            sx = (x1 + x2) * .5
            sy = (y1 + y2) * .5
            children = quad.nodes
            if 0 in children.keys(): 
                self.repulse(layout, node, children[0], x1, y1, sx, sy)
            if 1 in children.keys(): 
                self.repulse(layout, node, children[1], sx, y1, x2, sy)
            if 2 in children.keys(): 
                self.repulse(layout, node, children[2], x1, sy, sx, y2)
            if 3 in children.keys(): 
                self.repulse(layout, node, children[3], sx, sy, x2, y2)
        
if __name__ == '__main__':
    
    points = []
    for _ in range(10):
        (x, y) = (random(), random())
        o = Node(pos=(x,y))
        print (x,y)
        points.append(o)
        
    quad = QuadTree(points)
#     quad.printTree()
    
    
        
#         for i, p in enumerate(points):
#             self.insert(self.root, self.data[i], xs[i], ys[i], x1_, y1_, x2_, y2_);
#         # Discard captured fields.
#         xs = ys = data = d = None;
        
#     def insert(self, self, d, x, y, x1, y1, x2, y2):
#         '''
#         Recursively inserts the specified point p at the node self or one of its
#         descendants. The bounds are defined by [x1, x2] and [y1, y2].
#         '''
#         if math.isnan(x) or math.isnan(y):
#             return   
#         if self.leaf:
#             nx = self.x
#             ny = self.y
#             if nx is not None:
#                 # If the point at this leaf node is at the same position as the new
#                 # point we are adding, we leave the point associated with the
#                 # internal node while adding the new point to a child node. This
#                 # avoids infinite recursion.
#                 if abs(nx - x) + abs(ny - y) < .01:
#                     self.insertChild(self, d, x, y, x1, y1, x2, y2)
#                 else:
#                     nPoint = self.point
#                     self.x = self.y = self.point = None
#                     self.insertChild(self, nPoint, nx, ny, x1, y1, x2, y2)
#                     self.insertChild(self, d, x, y, x1, y1, x2, y2)
#             else:
#                 self.x = x, self.y = y, self.point = d;
#         else:
#             self.insertChild(self, d, x, y, x1, y1, x2, y2)
#             
#     def insertChild(self, self, d, x, y, x1, y1, x2, y2):
#         '''
#         Recursively inserts the specified point [x, y] into a descendant of node
#         self. The bounds are defined by [x1, x2] and [y1, y2].
#         '''
#         sx = (x1 + x2) * .5
#         sy = (y1 + y2) * .5
#         right = x >= sx
#         bottom = y >= sy
#         i = (bottom << 1) + right
#         
#         # Recursively insert into the child node.
#         self.leaf = False;
#         if not self.nodes.get(i, None):
#             self.nodes[i] = QuadTreeNode()
#         else:
#             self = self.nodes[i]
#         # Update the bounds as we recurse.
#         if right: x1 = sx
#         else: x2 = sx
#         if bottom: y1 = sy
#         else: y2 = sy
#         self.insert(self, d, x, y, x1, y1, x2, y2)
#             
            
            
            
            