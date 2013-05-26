# PROBABILISTIC ROBOT ACTION CORES - JSONRPC-HANDLER
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

from pyjamas.Timer import Timer
from pyjamas.JSONService import JSONProxy
from pyjamas import Window
from graph.graph import *
from random import random
from about import AboutDialog



class PRACRemoteHandler(JSONProxy):
    '''
    Handler for JSONRPC responses from the pracserver.
    '''
    
    REMOVE_NODES = 0
    ADD_NODES = 1
    
    def __init__(self, graph):
        JSONProxy.__init__(self, 'prac', ['action_cores', 
                                          'get_syntactic_atoms', 
                                          'get_wordsenses', 
                                          'get_possible_roles', 
                                          'get_senses_and_roles',
                                          'get_removed_nodes',
                                          'get_missing_roles',
                                          'get_possible_missing_roles',
                                          'get_missing_role_senses',
                                          'get_no_variables',
                                          'get_no_gnd_formulas',
                                          'get_symbolic_query'])
        self.handler = self
        self.color_idx = 0
        self.timer = Timer(notify=self.processNode)
        self.graph = graph
        self.processedNodes = []
        self.mode = None
        self.newNodes = []
        
    def processNode(self):
        graph = self.graph
        new_graph = self.new_graph
        while len(self.nodes) > 0:
            # nothing to be done
            if len(self.nodes) == 0:
                return
#         self.pracviz.layout.stop()
            # remove the node from the processing queue
            processedNode = self.nodes[0]
#             print 'processing', processedNode 
            self.nodes = self.nodes[1:]
            skip = False
            if self.mode == PRACRemoteHandler.ADD_NODES:
                node = graph.getNodeById(processedNode.id)
                # if the node doesn't exist in the graph so far,
                # we add it and all links to existing nodes
                if node is None:
                    node = self.addNodeAndLinks(processedNode, new_graph)
                    self.newNodes.append(node)
                    node.new = True
                # if does exist, adapt all attributes from the new node
                else:
                    skip = True
                    node.adaptAttributes(processedNode)
                    
                self.processedNodes.append(node)
#                 print 'processed:', self.processedNodes
#                 print 'nodes:', self.nodes
                # if all nodes have been updated, start removing the 
                # ones not occurring in the new graph
                if len(self.nodes) == 0:
                    self.mode = PRACRemoteHandler.REMOVE_NODES
                    self.nodes = [n for n in self.graph.nodes if not n in self.processedNodes]
                    self.processedNodes = []
            elif self.mode == PRACRemoteHandler.REMOVE_NODES:
#                 print 'removing', processedNode
                self.graph.removeNodeById(processedNode.id)
            if not skip: 
                break
        if len(self.nodes) > 0:
            self.timer.schedule(300)
        self.pracviz.layout.start()
        
        
    def addNodeAndLinks(self, node, new_graph):
        if len(self.graph.nodes) == 0:
            pos = self.pracviz.layout.cog
            node.x = pos[0]
            node.y = pos[1]
        else:
            pos = None                
        # set up the node's attributes
        node.charge = -2000
        self.graph.addNode(node)
        if len(self.graph.nodes) == 1:
            node.fixed = True
        # add all links of this node
        x_sum = 0.
        y_sum = 0.
        total = 0.
        links = new_graph.nodes2links.get(node, [])
        for link in links:
            source = self.graph.getNodeById(link.source.id)
            target = self.graph.getNodeById(link.target.id)
            if source is None or target is None: continue
            conn = source if source.id != node.id else target
            
            x_sum += conn.x
            y_sum += conn.y
            total += 1
            link.source = source
            link.target = target
            link.strength = 1000
            link.directed = True
            link.distance = 50
            if self.graph.containsLink(link): continue
            else:
                self.graph.addLink(link)
        if node.x is None or node.y is None:
            if total == 0: total = 1
#             print 'x:', (x_sum / total)
#             print 'y:', (y_sum / total)
            x = (x_sum / total) + 1 * ((x_sum / total)-self.pracviz.layout.cog[0])-100# + (random() - .5) * 100)
            y = (y_sum / total) + 1 * ((y_sum / total)-self.pracviz.layout.cog[1])-100# + (random() - .5) * 100)
            quadTree = self.pracviz.layout.quadTree
            quad = quadTree.getEmptyQuadForPoint(x, y)
            if quad is None:
#                 print 'no free space found'
                node.x = (x_sum / total + (random() - .5) * 20)
                node.y = (y_sum / total + (random() - .5) * 20)
                return node
            sq = quad[0]
            x1 = quad[1]
            y1 = quad[2]
            x2 = quad[3]
            y2 = quad[4]
            if sq is None: 
                node.x = round((x1 + x2) * .5)
                node.y = round((y1 + y2) * .5)
#                 'init node %s with x=%d, y=%d' % (node, node.x, node.y)
            else:
                sx = (x1 + x2) * .5
                sy = (y1 + y2) * .5
                   
                right = (sq.point.x >= sx)
                bottom = (sq.point.y >= sy)
                i = (bottom << 1) + right
                sel_i = round(random() * 2)
#                 print filter(lambda x: x != i, range(4)), sel_i
                sel = filter(lambda x: x != i, range(4))[sel_i]
                if sel == 0: 
                    sq = (x1, y1, sx, sy)
                if sel == 1:
                    sq = (sx, y1, x2, sy)
                if sel == 2: 
                    sq = (x1, sy, sx, y2)
                if sel == 3: 
                    sq = (sx, sy, x2, y2)                
                   
                node.x = round((sq[0] + sq[2]) * .5)
                node.y = round((sq[1] + sq[3]) * .5)
#                 print 'init node %s with x=%d, y=%d' % (node, node.x, node.y)
        
        return node
#         return node
            
    def rankNodes(self, graph):
        '''
        Computes the order in which the nodes should be
        processed.
        '''
        print 'rankNodes'
        # compute a spanning tree starting at the node with most links
        #first_node = max(graph.nodes2links.keys(), key=lambda x: len(graph.nodes2links[x])) # key is not implemented in pyjs
        first_node = None
        max_links = -1
        for n in graph.nodes:
            if n.fixed:
                first_node = n
                break
            links = graph.nodes2links.get(n, [])
            l_count = len(links)
            if l_count > max_links:
                first_node = n
                max_links = l_count
#         print first_node, max_links
        q = [first_node]
#         print q
        visited = []
        self.nodes = []
        while len(q) > 0:
            n = q.pop()
            if n in visited: continue
            self.nodes.append(n)
            visited.append(n)
            links = graph.nodes2links.get(n, [])
            for link in links:
                for node in [link.source, link.target]:
                    if node in visited: continue
                    q.append(node)
#         print 'node ranking', self.nodes
    
    def onRemoteResponse(self, response, request_info):
        print response
        print request_info
        
        query = response.get('query', None)
        if query is not None:
            box = AboutDialog()
            left = (Window.getClientWidth() - 512) / 2
            top = (Window.getClientHeight() - 256) / 2
            box.setPopupPosition(left, top)
            box.prob.setQuery(query)
            box.prob.setEvidence(response['evidence'])
            box.show()
            
        vars = response.get('no_variables', None)
        if vars is not None:
            self.pracviz.infoPanel.setText(0, 1, '%s' % vars)
            return
        formulas = response.get('no_gnd_formulas', None)
        if formulas is not None:
            self.pracviz.infoPanel.setText(1, 1, '%s' % formulas)
            return
        self.mode = PRACRemoteHandler.ADD_NODES
        self.new_graph = Graph.fromJSON(response)
        for n in self.graph.nodes:
            n.new = False
        self.rankNodes(self.new_graph)
        self.timer.schedule(200)
                    
#         self.pracviz.layout.start()
        
    def onRemoteError(self, code, errobj, request_info):
        Window.alert(code)