'''
Created on May 21, 2013

@author: nyga
'''
from pyjamas.ui.DialogWindow import DialogWindow
from pyjamas.ui.AbsolutePanel import AbsolutePanel
from pyjamas.ui.HTML import HTML
from pyjamas.ui.Button import Button
from pyjamas.ui.HorizontalPanel import HorizontalPanel
from pyjamas.ui import HasAlignment
from pyjamas.ui.Label import Label
from pyjamas import Window
from pyjamas.Timer import Timer
from SVGWindow import SVGWidget

class LogoPanel(DialogWindow):
    
    def __init__(self):
        DialogWindow.__init__(self)
        outer = AbsolutePanel()
        self.setStyleName('prac-panel')
        self.logo = SVGWidget('prac_logo_2.svg')
        self.height = '90px'
        self.width = '400px'
        self.modal = False
        self.logo.setHeight(self.height)
        self.logo.setWidth(self.width)
        outer.add(self.logo)
        outer.setHeight(self.height)
        outer.setWidth(self.width)
#         outer.setSpacing(0)
        self.setWidget(outer)
        
    def moveIn(self):
        '''
        Smoothly moves in the pop up window.
        '''
        if self.showing:
            self.hide
        self.show()
        self.setStyleAttribute('top', -self.getOffsetHeight())
        self.setStyleAttribute('left', 0)
        if not hasattr(self, 'timer'):
            self.timer = Timer()
        self.timer.__onTimer = self._moveIn
        self.timer.scheduleRepeating(20)
    
    def _moveIn(self):
        yPos = self.getAbsoluteTop()
        if yPos == 0:
            self.timer.cancel()
            return
        self.setStyleAttribute('top', yPos+5)
