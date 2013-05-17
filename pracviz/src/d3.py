'''
Created on May 14, 2013

@author: nyga
'''
from pyjamas import DOM
from pyjamas import Window
from pyjamas.ui.Widget import Widget
from __pyjamas__ import JS

class D3(Widget):
    
    def __init__(self, width, height):
        Widget.__init__(self)
        element = DOM.createDiv()
        self.setElement(element)
        self.setPixelSize(width, height)
        JS('''
        this['_d3'] = $wnd['d3'];
        this['_svg'] = $wnd['d3']['select'](@{{element}})['append']('svg');
        this['_svg']['attr']("width", width);
        this['_svg']['attr']("height", height);
        this['_svg']['attr']("background", "black");
        ''')
        print self._d3
        print self._svg
        
    def select(self, el):
        JS('''
        return this['_d3']['select'](@{{el}});
        ''')
        
    def par(self, element):
        JS('''
        this['_d3']['select'](this['element'])['append']("circle")
        ''')
        