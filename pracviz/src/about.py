'''
Created on Apr 21, 2013

@author: nyga
'''
from pyjamas.ui.RootPanel import RootPanel
from pyjamas.ui.Button import Button
from pyjamas.ui.Button import Button
from pyjamas.ui.DialogBox import DialogBox
from pyjamas.ui.DockPanel import DockPanel
from pyjamas.ui.HorizontalPanel import HorizontalPanel
from pyjamas.ui.HTML import HTML
from pyjamas.ui.Image import Image
from pyjamas.ui.Widget import Widget
from pyjamas.ui import HasAlignment
from pyjamas.ui.Label import Label

class AboutDialog(DialogBox):
    
    def __init__(self):
        DialogBox.__init__(self)
        
        # Use this opportunity to set the dialog's caption.
        self.setText("Probabilistic Robot Action Cores (PRAC)")
#        self.setWidth(500)
        
        # Create a DockPanel to contain the 'about' label and the 'OK' button.
        outer = DockPanel()
        outer.setSpacing(4)
        
        outer.add(Image('logo.svg'), DockPanel.WEST)
        
        # Create the 'OK' button, along with a listener that hides the dialog
        # when the button is clicked. Adding it to the 'south' position within
        # the dock causes it to be placed at the bottom.
        buttonPanel = HorizontalPanel()
        buttonPanel.setHorizontalAlignment(HasAlignment.ALIGN_RIGHT)
        closeButton = Button("Close", self)
#        closeButton.setStyleName('button')
        buttonPanel.add(closeButton)
        outer.add(buttonPanel, DockPanel.SOUTH)
        outer.setStyleAttribute('background-color', 'blue')
        buttonPanel.setStyleAttribute('background-color', 'blue')
        
        # Create the 'about' label. Placing it in the 'rest' position within the
        # dock causes it to take up any remaining space after the 'OK' button
        # has been laid out.
        
        textplain = '''<b>Probabilistic Robot Action Cores</b><br><br>
        (C) 2013 by Daniel Nyga (<a href="mailto:nyga@cs.tum.edu">nyga@cs.tum.edu</a>)'''
        text = HTML(textplain)
        text.setStyleName("mail-AboutText")
        outer.add(text, DockPanel.CENTER)
        
        # Add a bit of spacing and margin to the dock to keep the components from
        # being placed too closely together.
        outer.setSpacing(20)
        self.setWidget(outer)

    def onClick(self, sender):
        self.hide()