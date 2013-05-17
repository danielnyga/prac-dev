'''
Created on Apr 23, 2013

@author: nyga
'''
import math


class Node(object):
    
    def __init__(self, pos=None, charge=1, fixed=False, id=None):
        self.charge = charge
        self.fixed = fixed
        if pos is None:
            self.x = None
            self.y = None
        else:
            self.x = pos[0]
            self.y = pos[1]
        self.text = ''
        self.paddingx = 10
        self.paddingy = 10
        self.id = id
        self.textheight = 10
        
    def draw(self, raph):
        if not hasattr(self, 'raph_node'):
            self.raph_node = raph.rect(self.x, self.y, 0, 0)
            self.raph_node.text = raph.text(self.x, self.y, '')
            self.raph_node.text.setAttr('font-size', 20)
            self.raph_node.setAttr('fill', '#BFBFBF') 
            self.raph_node.setAttr('stroke', '#7F7F7F')
            self.raph_node.setAttr('stroke-width', 2)
        self.raph_node.text.setAttr('text', self.text)
        self.raph_node.text.setAttr('x', self.x)
        self.raph_node.text.setAttr('y', self.y)
        textwidth = self.raph_node.text.getBBox()['width']
        textheight = self.raph_node.text.getBBox()['height']
        self.raph_node.setAttr('width', textwidth + 2 * self.paddingx)
        self.raph_node.setAttr('height', textheight + 2 * self.paddingy)
        self.raph_node.setAttr('x', round(self.x-textwidth / 2.) - self.paddingx)
        self.raph_node.setAttr('y', round(self.y-textheight / 2.) - self.paddingy)
        
    def drawProc(self, p):
        p.setStrokeWeight(1)
        if not hasattr(self, 'fill'):
            p.setFillHex('#DDDDDD')
        else:
            p.setFillHex(self.fill)
        if not hasattr(self, 'stroke'):
            p.setStrokeHex('#777777')
        else:
            p.setStrokeHex(self.stroke)
        p.setTextSize(self.textheight)
        bbox = self.getBBox(p)
        p.rect(bbox['x'], bbox['y'], bbox['width'], bbox['height'])
        p.setFill(0, 0, 0)
        p.text(self.text, self.x, self.y + round(5 * .6))
        
    def getBBox(self, p):
        textwidth = p.getTextWidth(self.text)
        textheight = self.textheight
        width = textwidth + 2 * self.paddingx
        height = textheight + 2 * self.paddingy
        x = round(self.x - textwidth / 2.) - self.paddingx
        y = round(self.y - textheight / 2.) - self.paddingy
        return {'x': x, 'y':y, 'width': width, 'height': height}
    
    def distance(self, node):
        dx = node.x - self.x
        dy = node.y - self.y
        return math.sqrt(dx * dx + dy * dy)
    
    def __str__(self):
        return '(%f,%f)' % (self.x, self.y)
    
    def __eq__(self, other):
        return self.id == other.id

def sign(x):
    if x < 0: return -1
    if x > 0: return 1
    else: return 0

class Link(object):
    
    def __init__(self, source, target, strength=1, directed=False, label='', dist=100):
        self.source = source
        self.target = target
        self.strength = strength
        self.directed = directed
        self.label = label
        self.distance = float(dist)
        self.textheight = 8
        
    def __eq__(self, other):
        return self.source == other.source and self.target == other.target
    
    def drawProc(self, p):
        # compute the actual start and end points of the line and the position of the label
        bbox_source = self.source.getBBox(p)
        bbox_target = self.target.getBBox(p)
        dx = (self.target.x - self.source.x) 
        dy = (self.target.y - self.source.y)
        # for source
        x_ = bbox_source['width'] / (2. * abs(dx))
        y_ = bbox_source['height'] / (2. * abs(dy))
        if abs(dx * y_) > (bbox_source['width'] / 2.):
            scale = x_
        else:
            scale = y_
        x_source = self.source.x + dx * scale
        y_source = self.source.y + dy * scale
        # for target
        dx = -dx#(self.source.x - self.target.x) 
        dy = -dy#(self.source.y - self.target.y)
        x_ = bbox_target['width'] / (2. * abs(dx))
        y_ = bbox_target['height'] / (2. * abs(dy))
        if abs(dx * y_) > (bbox_target['width'] / 2.):
            scale = x_
        else:
            scale = y_
        x_target = self.target.x + dx * scale
        y_target = self.target.y + dy * scale
         
        text_pos = (round((x_source + x_target) / 2.), round((y_source + y_target) / 2.) + self.textheight * 0.6)
#         size = 5
#         arrowpath = 'M%d %d L%d %d L%d %d L%d %d' % (x_target,y_target,
#                                                        (x_target - size * 2),
#                                                        (y_target - size),
#                                                        (x_target - size * 2),
#                                                        (y_target + size),
#                                                        x_target,y_target)
#         angle = math.atan2(dx, -dy);
#         angle = (angle / (2 * math.pi)) * 360;
#         if not hasattr(self, 'line'):
#             self.line = canvas.path('M%d,%dL%d,%d' % 
#                                      (x_source, y_source, x_target, y_target), 
#                                      {'stroke': '#000000', 'stroke-width': 2})
#             self.line.label = canvas.text(text_pos[0], text_pos[1], self.label)
#             bbox = self.line.label.getBBox()
#             self.line.label.background = canvas.rect(bbox['x'], bbox['y'], bbox['width'], bbox['height'])
#             self.line.label.background.setAttr('fill', '#FFFFFF')
#             self.line.label.background.setAttr('stroke', '#FFFFFF')
#             self.line.toBack()
#             self.line.label.toFront()
#             if self.directed:
#                 self.line.arrow = canvas.path(arrowpath)
#                 self.line.arrow.setAttr('fill', '#000000')
#         else:
        p.setStroke(0,0,0)
        p.line(x_source, y_source, x_target, y_target)
        p.setFill(255,255,255)
        p.noStroke()
        textwidth=p.getTextWidth(self.label)
        textheight = self.textheight
        p.rect(text_pos[0]-round(textwidth/2)-2, text_pos[1]-round(textheight/2)-2, textwidth+4, textheight+4)
        p.setFill(0,0,0)
        p.text(self.label, text_pos[0], text_pos[1])
        size=10
        if self.directed:
            angle = math.atan2(dx, -dy) + math.pi;
            p.pushMatrix()
            p.translate(x_target, y_target)
            p.rotate(angle)
            p.triangle(0, 0, -round(size/2), round(size), round(size/2), size)
            p.popMatrix()
#             p.rotate(-angle)
#     
    
#     def drawProc(self, p):
#         p.setStroke(0,0,0)
#         p.line(self.source.x, self.source.y, self.target.x, self.target.y)
    
    def __str__(self):
        return str(self.source) + '->' + str(self.target)
        
class Graph(object):
    
    def __init__(self, nodes=None, links=None):
        if nodes is None:
            self.nodes = []
        else:
            self.nodes = nodes
        if links is None:
            self.links = []
        else:
            self.links = links
        self.id2node = {}
        
    def containsLink(self, l):
        for link in self.links:
            if link == l: return True
        return False
    
    def addNode(self, node):
        self.nodes.append(node)
        self.id2node[node.id] = node
        
    def removeNodeById(self, id):
        self.nodes.remove(self.getNodeById(id))
        del self.id2node[self.getNodeById(id)]
        
    def getNodeById(self, id):
        return self.id2node.get(id, None)
    
#     def clearGraph(self):
#         for n in self.
            
        