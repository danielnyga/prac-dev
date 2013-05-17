'''
Created on May 13, 2013

@author: nyga
'''
from pyjamas.ui.DialogWindow import DialogWindow
from pyjamas.ui.DockPanel import DockPanel
from pyjamas.ui.HTML import HTML
from pyjamas.ui.Button import Button
from pyjamas.ui.HorizontalPanel import HorizontalPanel
from pyjamas.ui import HasAlignment
from pyjamas.ui.Label import Label
from pyjamas import Window
from pyjamas.Timer import Timer

class PopupDialog(DialogWindow):
    
    def __init__(self, text, duration=None):
        '''
        If duration is set, the popup window will automatically disappear 
        after the given period of time.
        '''
        DialogWindow.__init__(self)
        
        self.duration = duration
        self.text = text
        self.closed = False
        
        # Use this opportunity to set the dialog's caption.
        self.setText("Probabilistic Action Cores (PRAC)")
        
        # Create a DockPanel to contain the 'about' label and the 'OK' button.
        outer = DockPanel()
        
        # Create the 'OK' button, along with a listener that hides the dialog
        # when the button is clicked. Adding it to the 'south' position within
        # the dock causes it to be placed at the bottom.
        buttonPanel = HorizontalPanel()
        buttonPanel.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
        if self.duration is None:
            closeButton = Button("Next", self)
            buttonPanel.add(closeButton)
            
        outer.add(buttonPanel, DockPanel.SOUTH)
        
        text = Label(self.text)
        text.setStyleName('prac-PopupText')
        
        text.setHorizontalAlignment(HasAlignment.ALIGN_CENTER)
        outer.add(text, DockPanel.CENTER)
        
        outer.setSpacing(20)
        self.setWidget(outer)
        
        width = (Window.getClientWidth() * .5)
        height = (Window.getClientHeight() * .5)
        self.setPopupPosition(round(width / 2), round(height / 2))
        self.setWidth(width)
        self.setHeight(height)
        self.closeListeners = []
        self.addDialogListener(self)
        
    def addCloseListener(self, callable):
        self.closeListeners.append(callable)
  
    def fadeIn(self):
        '''
        Smoothly fades in the pop up window. If a duration is set, the 
        respective timer will be started after the popup has reached full
        opacity.
        '''
        self.setStyleAttribute('opacity', '0')
        self.show()
        if not hasattr(self, 'timer'):
            self.timer = Timer()
        self.timer.__onTimer = self.__fadeIn
        self.timer.scheduleRepeating(0.2)
        
    def __fadeIn(self):
        op = float(self.getStyleAttribute('opacity'))
        if op >= 1:
            self.timer.cancel()
            if self.duration is not None:
                self.timer.__onTimer = self.fadeOut
                self.timer.schedule(self.duration)
        else:
            self.setStyleAttribute('opacity', '%.2f' % (min(op+0.01, 1)))
    
    def fadeOut(self):
        '''
        Smoothly fades out the popup window and closes it after it has
        been faded out completely.
        '''
        if not hasattr(self, 'timer'):
            self.timer = Timer()
        self.timer.__onTimer = self.__fadeOut
        self.timer.scheduleRepeating(0.2)
    
    def __fadeOut(self):
        op = float(self.getStyleAttribute('opacity'))
        if op <= 0:
            self.timer.cancel()
            self.onClose()
        else:
            self.setStyleAttribute('opacity', '%.2f' % (max(0, op-0.01)))
    
    def onClose(self, sender):
        for c in self.closeListeners: c()
        DialogWindow.onClose(self, sender)
    
    def onClick(self, sender):
        self.fadeOut()
