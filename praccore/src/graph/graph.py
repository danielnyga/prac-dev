# PROBABILISTIC ROBOT ACTION CORES - DRAWABLE GRAPH
#
# (C) 2012-2013 by Daniel Nyga (nyga@cs.tum.edu)
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

import math

# define some aesthetic colors for filling and stroking nodes
colors_fill = ['#ff9a8d', # red 
               '#66c0ff', # blue
               '#9cf4a1', # green
               '#fff48d', # yellow
               '#ecb5f2', # magenta
               '#ffd28f'  # orange
               ] 
colors_stroke = ['#c55454', # red
                 '#304e7c', # blue
                 '#209e1f', # green
                 '#b9b15b', # yellow
                 '#ae32ac', # magenta
                 '#b38520'  # orange
                 ]
node_gray_fill = '#dedede'
node_gray_stroke = '#777777'

node_gray_dark_fill = '#bfbfbf'
node_gray_dark_stroke = '#5d5d5d'

class Node(object):
    
    json_node_attributes = ['id', 'label', 'color', 'fixed']
    
    def __init__(self, pos=None, charge=1, fixed=False, id=None, label=''):
        self.charge = charge
        self.fixed = fixed
        if pos is None:
            self.x = None
            self.y = None
        else:
            self.x = pos[0]
            self.y = pos[1]
        self.label = label
        self.paddingx = 10
        self.paddingy = 10
        self.id = id
        self.textheight = 12
        
#     def draw(self, raph):
#         if not hasattr(self, 'raph_node'):
#             self.raph_node = raph.rect(self.x, self.y, 0, 0)
#             self.raph_node.text = raph.text(self.x, self.y, '')
#             self.raph_node.text.setAttr('font-size', 20)
#             self.raph_node.setAttr('fill', '#BFBFBF') 
#             self.raph_node.setAttr('stroke', '#7F7F7F')
#             self.raph_node.setAttr('stroke-width', 2)
#         self.raph_node.text.setAttr('text', self.text)
#         self.raph_node.text.setAttr('x', self.x)
#         self.raph_node.text.setAttr('y', self.y)
#         textwidth = self.raph_node.text.getBBox()['width']
#         textheight = self.raph_node.text.getBBox()['height']
#         self.raph_node.setAttr('width', textwidth + 2 * self.paddingx)
#         self.raph_node.setAttr('height', textheight + 2 * self.paddingy)
#         self.raph_node.setAttr('x', round(self.x-textwidth / 2.) - self.paddingx)
#         self.raph_node.setAttr('y', round(self.y-textheight / 2.) - self.paddingy)
        
    def draw(self, p):
        p.setStrokeWeight(1)
        if not hasattr(self, 'color'):
            p.setFillHex(node_gray_fill if not self.new else node_gray_dark_fill)
        else:
            p.setFillHex(colors_fill[self.color % len(colors_fill)])
        if not hasattr(self, 'stroke'):
            p.setStrokeHex(node_gray_stroke if not self.new else node_gray_dark_stroke)
        else:
            p.setStrokeHex(colors_stroke[self.color % len(colors_fill)])
        p.setTextSize(self.textheight)
        bbox = self.getBBox(p)
        p.rect(bbox['x'], bbox['y'], bbox['width'], bbox['height'])
        p.setFill(0, 0, 0)
        p.text(self.label, self.x, self.y + round(5 * .6))
        
    def getBBox(self, p):
        textwidth = p.getTextWidth(self.label)
        textheight = self.textheight
        width = textwidth + 2 * self.paddingx
        height = textheight + 2 * self.paddingy
        x = round(self.x - textwidth / 2.) - self.paddingx
        y = round(self.y - textheight / 2.) - self.paddingy
        self.bbox = {'x': x, 'y':y, 'width': width, 'height': height}
        return self.bbox
    
    def distance(self, node):
        if not hasattr(self, 'bbox') or not hasattr(node, 'bbox'):
            dx = self.x - node.x
            dy = self.y - node.y
        else:
            bbox_source = node.bbox
            bbox_target = self.bbox
            dx = (self.x - node.x) 
            dy = (self.y - node.y)
            # for source
            x_ = bbox_source['width'] / (2. * abs(dx))
            y_ = bbox_source['height'] / (2. * abs(dy))
            if abs(dx * y_) > (bbox_source['width'] / 2.):
                scale = x_
            else:
                scale = y_
            x_source = node.x + dx * scale
            y_source = node.y + dy * scale
            # for target
            dx = -dx#(self.source.x - self.target.x) 
            dy = -dy#(self.source.y - self.target.y)
            x_ = bbox_target['width'] / (2. * abs(dx))
            y_ = bbox_target['height'] / (2. * abs(dy))
            if abs(dx * y_) > (bbox_target['width'] / 2.):
                scale = x_
            else:
                scale = y_
            x_target = self.x + dx * scale
            y_target = self.y + dy * scale
            dx = x_source - x_target
            dy = y_source - y_target
        return math.sqrt(dx * dx + dy * dy)
    
    def adaptAttributes(self, other):
        '''
        Set all attributes of this node to the values of the
        corresponding attributes of the other node.
        '''
        for attr in Node.json_node_attributes:
            if not hasattr(other, attr): continue
            setattr(self, attr, getattr(other, attr))
        if self.label == '': self.label = self.id
    
    def equalAttrs(self, other):
        '''
        Checks if the attributes of two nodes are equal.
        '''
        for attr in Node.json_node_attributes:
            if (hasattr(other, attr) and not hasattr(self, attr)) or (not hasattr(other, attr) and hasattr(self, attr)):
                return False
            elif hasattr(other, attr) and hasattr(self, attr):
                if getattr(other, attr) != getattr(self, attr):
                    return False
        return True
    
    def __str__(self):
        return self.id
    
    def __eq__(self, other):
        return self.id == other.id
    
    def __repr__(self):
        return str(self)

def sign(x):
    if x < 0: return -1
    if x > 0: return 1
    else: return 0

class Link(object):
    
    json_link_attributes = ['source', 'target', 'label']
    
    def __init__(self, source, target, strength=1, directed=False, label='', dist=100):
        self.source = source
        self.target = target
        self.strength = strength
        self.directed = directed
        self.label = label
        self.distance = float(dist)
        self.textheight = 8
        
    def __eq__(self, other):
        return self.source.id == other.source.id and self.target.id == other.target.id
    
    def draw(self, p):
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
    
    def __str__(self):
        return str(self.source) + '->' + str(self.target)
    
    def __repr__(self):
        return str(self)
        
class Graph(object):
    '''
    Class representing a graph of vertices and directed or undirected edges.
    CAUTION: The class holds internal data structures for efficiently 
    process graph operations. DO NOT ACCESS THE FIELDS (e.g. nodes and 
    edges) OF THIS CLASS DIRECTLY, BUT USE THE RESPECTIVE METHODS FOR 
    MANIPULATING IT.
    '''
    def __init__(self, nodes=None, links=None):
        self.id2node = {}
        self.nodes2links = {}
        self.nodes = []
        self.links = []
        if nodes is not None:
            for n in nodes: self.addNode(n)
        if links is not None:
            for l in links: self.addLink(l)
        
    def containsLink(self, l):
        '''
        Checks for a given link if it exists in the graph.
        '''
        for link in self.links:
            if link == l: return True
        return False
    
    def addNode(self, node):
        '''
        Adds a new node to the graph. Nodes must have a unique id.
        If a node with the same id as node already exists, it's being overwritten.
        In this case, all links of the old node are lost.
        '''
        if self.getNodeById(node.id) is not None:
            self.removeNodeById(node.id)
        self.nodes.append(node)
        self.id2node[node.id] = node
        
    def addLink(self, l):
        '''
        Adds a link between two nodes to the graph.
        '''
#         if self.containsLink(l): return
        for node in [l.source, l.target]:
            links = self.nodes2links.get(node, None)
            if links is None:
                links = []
                self.nodes2links[node] = links
            links.append(l)
        self.links.append(l)
    
    def removeLink(self, l):
        '''
        Removes a link from the graph.
        '''
        for node in [l.source, l.target]:
            self.nodes2links[node].remove(l)
        self.links.remove(l)
    
    def removeNodeById(self, id):
        '''
        Removes the node with the given id from the graph.
        All links the node is connected with will also
        be deleted.
        '''
        node = self.getNodeById(id)
        if node is None:
            return
        for l in list(self.links):
            if l.source.id == node.id or l.target.id == node.id:
                self.removeLink(l)
        del self.nodes2links[node]
        del self.id2node[id]
        self.nodes.remove(node)
        
    def getNodeById(self, id):
        '''
        Returns the node with the given id or None if it doesn't exist.
        '''
        return self.id2node.get(id, None)
    
    def toJSON(self):
        '''
        Converts the graph into a JSON compatible data structure.
        NB: NOT a JSON string.
        '''
        graph = {}
        nodes = []
        for node in self.nodes:
            attrs = {}
            for attr in Node.json_node_attributes:
                if not hasattr(node, attr): continue
                attrs[attr] = getattr(node, attr)
            nodes.append(attrs)
        graph['nodes'] = nodes
        links = []
        for link in self.links:
            attrs = {}
            attrs['source'] = link.source.id
            attrs['target'] = link.target.id
            for attr in Link.json_link_attributes:
                if attr == 'source' or attr == 'target': continue
                if not hasattr(link, attr): continue
                attrs[attr] = getattr(link, attr)
            links.append(attrs)
        graph['links'] = links
        return graph
        
    
    @staticmethod
    def fromJSON(json):
        '''
        Deserializes a JSON data structure into a Graph object.
        '''
        graph = Graph()
        json_nodes = json.get('nodes', None)
        if json_nodes is not None:
            for json_node in json_nodes: # this is a dict
                node = Node()
                for attr in json_node.keys():
                    setattr(node, attr, json_node[attr])
                if node.label == '': node.label = node.id
                graph.addNode(node)
        json_links = json.get('links', None)
        if json_links is not None:
            for json_link in json_links:
                source = json_link['source']
                target = json_link['target']
                link = Link(graph.getNodeById(source), graph.getNodeById(target))
                for attr in json_link.keys():
                    if attr == 'source' or attr == 'target': continue
                    setattr(link, attr, json_link[attr])
                graph.addLink(link)
        return graph
    
                
        