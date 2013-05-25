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
from pyjamas.ui.FlexTable import FlexTable
from pyjamas.ui.FlowPanel import FlowPanel
from pyjamas.ui import HasAlignment
from pyjamas.ui.Widget import Widget
from pyjamas.ui.Button import Button
from pyjamas.ui.Image import Image
from pyjamas.ui.Composite import Composite
from pyjamas.ui.TextBox import TextBox
from pyjamas.ui.Label import Label
from pyjamas.ui.HTML import HTML
from pyjamas.ui.AbsolutePanel import AbsolutePanel
from pyjamas.raphael.raphael import Raphael,DOCK_CONNECTION
from pyjamas import DOM
from pyjamas.Timer import Timer
from pyjamas.JSONService import JSONProxy

from pyjamas import Window
from pyjamas import DeferredCommand
from about import AboutDialog
from force import ForceLayout
import random
import time
from PopupDialog import PopupDialog
from processingcanvas import ProcessingCanvas
from SVGWindow import SVGWidget
from LogoPanel import LogoPanel
from pracremote import PRACRemoteHandler
from graph.graph import Graph
from widgets import CondProbWidget


def about():
    box = AboutDialog()
    left = (Window.getClientWidth() - 512) / 2
    top = (Window.getClientHeight() - 256) / 2
    box.setPopupPosition(left, top)
    box.show()
    
class PRACViz():
    
    def __init__(self):
        self.circles = []
        self.dragging = False
    
    def onModuleLoad(self):
        outer = DockPanel()
        outer.setWidth('100%')
#        outer.setHeight('100%')

        self.logo_panel = LogoPanel()
        
        self.popupDialogs = [PopupDialog('Obtaining Syntactic Features from Stanford Parser...', 10),
                             PopupDialog('Obtaining Possible Word Senses from WordNet...', 2000),
                             PopupDialog('Obtaining Possible Action Roles from FrameNet...', 2000),
                             PopupDialog('Simultaneous Word Disambiguation & Action Role Labeling...', 2000),
                             PopupDialog('Removing Inapplicable Senses and Roles...', 2000),
                             PopupDialog('Obtaining Missing Action Roles from PRAC...', 2000),
                             PopupDialog('Obtaining Possible Word Senses from WordNet...', 2000),
                             PopupDialog('Inferring Object Types of Missing Roles...', 2000)]
        self.step_i = 0
        self.steps = [self.addSyntacticFeatures,
                      self.addWordSenses,
                      self.addPossibleRoles,
                      self.addSensesAndRoles,
                      self.removeInapplicableNodes,
                      self.addMissingRoles,
                      self.addMissingRoleConcepts,
                      self.addMissingRoleSenses]

        # graph maintaining all the nodes
        self.g = Graph()
        # create gravity force layout        
        self.layout = ForceLayout((800,400), self.g, self.redraw, gravity=-0.0001)
        
        self.controlPanel = AbsolutePanel()
        self.controlPanel.setStyleName('prac-panel')
        self.controlPanel.add(SVGWidget('prac_panel.svg'))
        
        queryPanel = HorizontalPanel()
        queryPanel.setStyleAttribute('background', '#dedede')
        queryTxt = TextBox()
        queryTxt.setStyleName('gwt-TextBox-custom')
        queryTxt.setVisibleLength('30')
        queryTxt.setText('Flip the pancake around.')
        queryTxt.defaultTxt = True
        queryTxt.addClickListener(getattr(self, "eraseTxt"))
#         queryPanel.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
#         queryPanel.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
        queryPanel.add(Label('Natural-language Instruction:'))
        queryPanel.add(queryTxt)
        
        #
        # Button Panel
        #
        btnPanel = FlexTable()
        btnPanel.setWidth('100px')
        queryPanel.add(btnPanel)
        
        b = Button('Query PRAC!')
        b.addClickListener(self.sendQuery)
        btnPanel.setWidget(0, 0, b)

        nextBtn = Button('Next >')
        nextBtn.addClickListener(self.nextStep)
        btnPanel.setWidget(0, 1, nextBtn)

        resetBtn = Button('Reset')
        resetBtn.addClickListener(self.reset)
        btnPanel.setWidget(1, 0, resetBtn)
        
        aboutBtn = Button('About...')
        aboutBtn.addClickListener(about)
        btnPanel.setWidget(1, 1, aboutBtn)

        layoutBtn = Button('Layout!')
        layoutBtn.addClickListener(self.layout.start)
        btnPanel.setWidget(1, 2, layoutBtn)

        queryPanel.setWidth('100%')
        
        
        #
        # Info Panel
        #
        infoPanel = FlexTable()
        infoPanel.setWidth('200px')
        queryPanel.add(infoPanel)
        
        infoPanel.setText(0, 0, '# Variables:')
        infoPanel.setText(0, 1, 'N/A')
        infoPanel.setText(1, 0, '# Ground Formulas:')
        infoPanel.setText(1, 1, 'N/A')

        infoPanel.prob = CondProbWidget()
        infoPanel.setWidget(2, 0, infoPanel.prob)
        infoPanel.getFlexCellFormatter().setColSpan(2, 0, 2)
        self.infoPanel = infoPanel
        
#         center.initWidget(self.canvas)
        self.canvas = ProcessingCanvas(self.redraw)
#         self.canvas.setBackground(255,255,255)
#         self.canvas.setSize('100', '100')
#         self.canvas.setFrameRate(15)
        self.canvas.textAlign('CENTER')
        self.canvas.noLoop()
        self.canvas.setMouseMove(self.onMouseMove)
        self.canvas.setMouseDown(self.onMouseDown)
        self.canvas.setMouseUp(self.onMouseUp)
        self.canvas.setMouseDrag(self.onMouseDrag)
#         self.canvas.noLoop()
#         self.canvas.text('Hello, PRAC!', 300, 300)
        outer.setBorderWidth(0)
#         outer.add(welcomePanel, DockPanel.NORTH)
        outer.add(self.canvas, DockPanel.CENTER)
        outer.add(queryPanel, DockPanel.SOUTH)
        
        RootPanel().add(outer)
        # some settings
        Window.enableScrolling(False)
        Window.setMargin("0px")
        
        self.queryTxt = queryTxt
#         self.center = center
        self.queryPanel = queryPanel
#         self.welcomePanel = welcomePanel
#         self.viz = viz
#         self.viz.addListener('mousewheel', self.onMouseWheel)
#         self.canvas.addListener('mousemove', self.onMouseMove)
#         self.viz.addListener('mousedown', self.onMouseDown)
        Window.addWindowResizeListener(self)
        DeferredCommand.add(self)
        self.pracRemote = PRACRemoteHandler(self.g)
        self.pracRemote.pracviz = self
        self.translation = (0, 0)
        self.mouse_x = self.mouse_y = 0
        self.scrolling = False
        self.i = 0
        self.logo_panel.moveIn()
    
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
#         self.canvas.text('redraw: %d, x=%d, y=%d' % (self.i, self.mouse_x, self.mouse_y), 20, 20)
        self.i+=1
#         print self.translation
#         self.canvas.translate(self.translation[0], self.translation[1])
        for l in self.g.links:
            l.draw(self.canvas)
        for n in self.g.nodes:
            n.draw(self.canvas)
        
    def execute(self):
        self.onWindowResized(Window.getClientWidth(), Window.getClientHeight())
        
    def onWindowResized(self, width, height):
        new_height = height - self.canvas.getAbsoluteTop() - 90
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
        self.infoPanel.setText(0, 1, 'N/A')
        self.infoPanel.setText(1, 1, 'N/A')
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
        self.pracRemote.get_no_variables(self.pracRemote, 'pracinit')
        self.pracRemote.get_no_gnd_formulas(self.pracRemote, 'pracinit')
        self.pracRemote.get_senses_and_roles(self.pracRemote, None)
    
    def removeInapplicableNodes(self):
        self.pracRemote.get_removed_nodes(self.pracRemote, None)
    
    def addMissingRoles(self):
        self.pracRemote.get_missing_roles(self.pracRemote, None)
    
    def addMissingRoleConcepts(self):
        self.pracRemote.get_possible_missing_roles(self.pracRemote, None)
        
    def addMissingRoleSenses(self):
        self.pracRemote.get_no_variables(self.pracRemote, 'missingroles')
        self.pracRemote.get_no_gnd_formulas(self.pracRemote, 'missingroles')
        self.pracRemote.get_missing_role_senses(self.pracRemote, None)
        
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
