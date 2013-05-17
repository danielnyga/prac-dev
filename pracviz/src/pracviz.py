# PROBABILISTIC ROBOT ACTION CORES - WEBAPP
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

import pyjd # this is dummy in pyjs.

from pyjamas.ui.RootPanel import RootPanel
from pyjamas.ui.DockPanel import DockPanel
from pyjamas.ui.HorizontalPanel import HorizontalPanel
from pyjamas.ui import HasAlignment
from pyjamas.ui.Widget import Widget
from pyjamas.ui.Button import Button
from pyjamas.ui.Image import Image
from pyjamas.ui.Composite import Composite
from pyjamas.ui.TextBox import TextBox
from pyjamas.ui.Label import Label
from pyjamas.ui.HTML import HTML
from pyjamas.raphael.raphael import Raphael,DOCK_CONNECTION
from pyjamas import DOM
from pyjamas.Timer import Timer
from pyjamas.JSONService import JSONProxy

from pyjamas import Window
from pyjamas import DeferredCommand
from about import AboutDialog
from force import ForceLayout
from graph import Node, Link, Graph
import random
import time
from PopupDialog import PopupDialog
from processingcanvas import ProcessingCanvas


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


def about(fred):
    box = AboutDialog()
    left = (Window.getClientWidth() - 512) / 2
    top = (Window.getClientHeight() - 256) / 2
    box.setPopupPosition(left, top)
    box.show()
    
class PRACRemote(JSONProxy):
    def __init__(self):
        JSONProxy.__init__(self, "prac", ["action_cores", 
                                          "get_syntactic_atoms", 
                                          "get_wordsenses", 
                                          "get_possible_roles", 
                                          "get_senses_and_roles"])
        self.handler = self
#         self.timer = Timer(notify=self.addNode)
#     
#     def addNode(self):
#         print 'timer'
#         node = self.nodes.pop()
#         id = node['id']
#         if self.pracviz.g.getNodeById(id) != None:
#             continue
#         rand = 500 if len(self.pracviz.g.nodes) else 0
#         new_node = Node(pos=(self.pracviz.layout.cog[0]+(random.random()-.5)*rand, 
#                              self.pracviz.layout.cog[1]+(random.random()-.5)*rand), charge=-1000, id=id)
#         new_node.text = id
#         self.pracviz.g.addNode(new_node)
#         if len(self.pracviz.g.nodes) == 1:
#             new_node.fixed = True
#         if len(self.nodes) > 0:
#             self.timer.schedule(200)
    
    def onRemoteResponse(self, response, request_info):
        print response
        type = response.get('type', None)
        if type == 'colors':
            clusters = response.get('clusters')
            if clusters is None: return
            for color, nodes in enumerate(clusters):
                for node in nodes:
                    n = self.pracviz.g.getNodeById(node)
                    if n is None: continue
                    n.fill = colors_fill[color % len(colors_fill)]
                    n.stroke = colors_stroke[color % len(colors_stroke)]
            self.pracviz.layout.start()
        elif type == 'graph':
            nodes = response.get('nodes', None)
    #         for n in self.pracviz.g.nodes:
    #             n.fixed = True
            if nodes is not None:
    #             self.nodes = nodes
    #             self.timer.schedule(200)
                for node in nodes:
                    id = node['id']
                    if self.pracviz.g.getNodeById(id) != None:
                        continue
                    rand = 500 if len(self.pracviz.g.nodes) else 0
                    new_node = Node(pos=(self.pracviz.layout.cog[0]+(random.random()-.5)*rand, 
                                         self.pracviz.layout.cog[1]+(random.random()-.5)*rand), charge=-1000, id=id)
                    if node.get('label', None) is None:
                        new_node.text = id
                    else:
                        new_node.text = node['label']
                    self.pracviz.g.addNode(new_node)
                    if len(self.pracviz.g.nodes) == 1:
                        new_node.fixed = True
                    
            links = response.get('links', None)
            if links is not None:
                for link in links:
                    if self.pracviz.g.containsLink(link): continue
                    sourceId = link['source']
                    targetId = link['target']
                    label = link.get('label', '')
                    source = self.pracviz.g.getNodeById(sourceId, None)
                    target = self.pracviz.g.getNodeById(targetId, None)
                    if source is None or target is None:
                        continue
                    new_link = Link(source=source, target=target, strength=1000, label=label, directed=True, dist=120)
                    self.pracviz.g.links.append(new_link)
                     
            self.pracviz.layout.start()
        
    def onRemoteError(self, code, errobj, request_info):
        Window.alert(code)

    
class PRACViz():
    
    def __init__(self):
        self.circles = []
        self.dragging = False
    
    def onModuleLoad(self):
        outer = DockPanel()
        outer.setWidth('100%')
#        outer.setHeight('100%')
        self.pracRemote = PRACRemote()
        self.pracRemote.pracviz = self
        
        self.popupDialogs = [PopupDialog('Obtaining syntactic features from Stanford Parser...', 2000),
                             PopupDialog('Obtaining possible word senses from WordNet...', 2000),
                             PopupDialog('Obtaining possible action roles from FrameNet...', 2000),
                             PopupDialog('Simultaneous word sense disambiguation & action role labeling...', 2000),
                             PopupDialog('Obtaining missing action roles from PRAC...', 2000),
                             PopupDialog('Inferring object types of missing roles...', 2000)]
        self.step_i = 0
        self.steps = [self.addSyntacticFeatures,
                      self.addWordSenses,
                      self.addPossibleRoles,
                      self.addSensesAndRoles,
                      self.addMissingRoles,
                      self.addMissingRoleConcepts]
        
        welcomePanel = Composite()
        l = Image('prac_logo.png')
        l.setStyleAttribute('margin', '10px')
        welcomePanel.initWidget(l)
        welcomePanel.setHeight('50px')
        
        queryPanel = HorizontalPanel()
        queryTxt = TextBox()
        queryTxt.setStyleName('gwt-TextBox-custom')
        queryTxt.setVisibleLength('50')
        queryTxt.setText('Flip the pancake around.')
        queryTxt.defaultTxt = True
        queryTxt.addClickListener(getattr(self, "eraseTxt"))
        queryPanel.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
        queryPanel.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
        queryPanel.add(Label('Natural-language Instruction:'))
        queryPanel.add(queryTxt)
        nextBtn = Button('Next >')
        nextBtn.addClickListener(self.nextStep)
        queryPanel.add(nextBtn)
        b = Button('Query PRAC!')
        b.addClickListener(self.sendQuery)
        queryPanel.add(b)
        resetBtn = Button('Reset')
        resetBtn.addClickListener(self.reset)
        queryPanel.add(resetBtn)
        
#         center.initWidget(self.canvas)
        self.canvas = ProcessingCanvas(self.redraw)
#         self.canvas.setBackground(255,255,255)
#         self.canvas.setSize('100', '100')
        self.canvas.setFrameRate(15)
        self.canvas.textAlign('CENTER')
        self.canvas.noLoop()
        self.canvas.setMouseMove(self.onMouseMove)
        self.canvas.setMouseDown(self.onMouseDown)
        self.canvas.setMouseUp(self.onMouseUp)
        self.canvas.setMouseDrag(self.onMouseDrag)
#         self.canvas.noLoop()
#         self.canvas.text('Hello, PRAC!', 300, 300)
        outer.setBorderWidth(1)
        outer.add(welcomePanel, DockPanel.NORTH)
        outer.add(self.canvas, DockPanel.CENTER)
        outer.add(queryPanel, DockPanel.SOUTH)
        
        RootPanel().add(outer)
        # some settings
        Window.enableScrolling(False)
        Window.setMargin("0px")
        
        self.queryTxt = queryTxt
#         self.center = center
        self.queryPanel = queryPanel
        self.welcomePanel = welcomePanel
#         self.viz = viz
#         self.viz.addListener('mousewheel', self.onMouseWheel)
#         self.canvas.addListener('mousemove', self.onMouseMove)
#         self.viz.addListener('mousedown', self.onMouseDown)
        Window.addWindowResizeListener(self)
        DeferredCommand.add(self)
        self.g = Graph()
        self.layout = ForceLayout((800,400), self.g, self.redraw, gravity=-0.0001)
        self.translation = (0, 0)
        self.mouse_x = self.mouse_y = 0
        self.scrolling = False
        self.i = 0
    
    def onMouseDown(self, sender, event):
        self.mouse_x = self.canvas._p.mouseX
        self.mouse_y = self.canvas._p.mouseY
        self.canvas.loop()
    
    def onMouseMove(self, sender, event):pass
    
    def onMouseWheel(self, sender, velocity):
        viewbox = self.viz.svg.getAttribute('viewBox')
        if viewbox is None:
            width = self.viz.getOffsetWidth()
            height = self.viz.getOffsetHeight()
            x = y = 0
        else:
            scale = viewbox.split()
            x = int(scale[0])
            y = int(scale[1])
            width = int(scale[2])
            height = int(scale[3])
        x += self.mouse_x * 0.01 * velocity
        y += self.mouse_y * 0.01 * velocity
        self.viz.setViewBox(x, y, width - 20 * velocity, height - 20 * velocity)
    
    def onMouseUp(self, sender, even):
        self.canvas.noLoop()
        
    def onMouseDrag(self, sender, event):
        x_new = self.canvas._p.mouseX
        y_new = self.canvas._p.mouseY
        x = -(self.mouse_x - x_new)
        y = -(self.mouse_y - y_new)
        self.canvas.translate(x, y)
        self.mouse_x = x_new
        self.mouse_y = y_new
            
    def redraw(self): 
        self.canvas.setBackground(255, 255, 255)
        self.canvas.text('redraw: %d, x=%d, y=%d' % (self.i, self.mouse_x, self.mouse_y), 20, 20)
        self.i+=1
        print self.translation
#         self.canvas.translate(self.translation[0], self.translation[1])
        for n in self.g.nodes:
            n.drawProc(self.canvas)
        for l in self.g.links:
            l.drawProc(self.canvas)

    def execute(self):
        self.onWindowResized(Window.getClientWidth(), Window.getClientHeight())
        
    def onWindowResized(self, width, height):
        new_height = height - self.canvas.getAbsoluteTop() - 80
        if new_height < 1:
            new_height = 1
#         width = self.canvas.getOffsetWidth()
        self.canvas.setSize(width, new_height)
        self.layout.width = width
        self.layout.height = new_height
        self.redraw()
#         self.canvas.text('Hello, PRAC!', 30, 30)
#         self.center.setHeight(new_height)
    
    def eraseTxt(self):
        if self.queryTxt.defaultTxt:
            self.queryTxt.setText('')
            self.queryTxt.defaultTxt = False
            
    def sendQuery(self):
        self.reset()
        self.nextStep()
    
    def nextStep(self):
        self.layout.stop()
        step = self.steps[self.step_i]
        d = self.popupDialogs[self.step_i]
        d.addCloseListener(step)
        d.fadeIn()
        self.step_i += 1

    def addSyntacticFeatures(self):
        self.pracRemote.get_syntactic_atoms(self.pracRemote, self.queryTxt.getText())
    
    def addWordSenses(self):
        self.pracRemote.get_wordsenses(self.pracRemote, None)
        
    def addPossibleRoles(self):
        self.pracRemote.get_possible_roles(self.pracRemote, None)
        
    def addSensesAndRoles(self):
        self.pracRemote.get_senses_and_roles(self.pracRemote, None)
    
    def addMissingRoles(self): pass
    
    def addMissingRoleConcepts(self): pass
        
    def reset(self):
        self.step_i = 0
        self.g.nodes = []
        self.g.links = []
        self.layout.nodes = self.g.nodes
        self.layout.links = self.g.links
#         self.viz.clear()
        self.layout.start()
    
if __name__ == '__main__':
    pyjd.setup("http://127.0.0.1:8080/prac/pracviz.html")
    m = PRACViz()
    m.onModuleLoad()
    
    pyjd.run()
