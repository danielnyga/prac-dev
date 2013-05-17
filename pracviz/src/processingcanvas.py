'''
Created on May 15, 2013

@author: nyga
'''

from pyjamas.Canvas.GWTCanvas import GWTCanvas
from pyjamas import DOM
from pyjamas import Window
from __pyjamas__ import JS
import math

# p = None
# radius = 50.0
# delay = 16
#  
# def setup():
#     global p,radius,delay,X,Y,nX,nY
#     p.size(200,200)
#     p.strokeWeight( 10 )
#     p.frameRate( 15 )
#     X = p.width / 2
#     Y = p.width / 2
#     nX = X
#     nY = Y
# # 
# def draw():
#     global p,radius,delay,X,Y,nX,nY
#     radius = radius + math.sin( p.frameCount / 4 )
#     X+=(nX-X)/delay
#     Y+=(nY-Y)/delay
#     p.background( 100 )
#     p.fill( 0, 121, 184 )
#     p.stroke(255)
#     p.ellipse(X, Y, radius, radius )
# # 
# def mouseMoved():
#     global p,nX,nY
#     nX = p.mouseX
#     nY = p.mouseY

class ProcessingCanvas(GWTCanvas):
    
    def __init__(self, drawCallback):
        GWTCanvas.__init__(self)
        canvas = self.getCanvasElement()
        self.c = canvas
        self.drawCallback = drawCallback
        JS('''
            this['_p'] = new $wnd['Processing'](@{{canvas}});
            //this['_p']['setup'] = @{{setup}};
            this['_p']['draw'] = @{{drawCallback}};
        ''')
    
    def setSize(self, width, height):
        GWTCanvas.setSize(self, width, height)
        width = self.getOffsetWidth()
        height = self.getOffsetHeight()
        JS('''
        this['_p']['size'](@{{width}}, @{{height}});
        ''')
    
    def ellipse(self, x, y, width, height):
        JS('''
        this['_p']['ellipse'](@{{x}}, @{{y}}, @{{width}}, @{{height}});
        ''')
        
    def setStrokeWeight(self, weight):
        JS('''
        this['_p']['strokeWeight'](@{{weight}});
        ''')
        
    def setFill(self, r, g, b):
        JS('''
        this['_p']['fill'](@{{r}}, @{{g}}, @{{b}});
        ''')
        
    def setFillHex(self, c):
        c = int(c[1:], 16)
        r = (c >> 16) & 0xFF
        g = (c >> 8) & 0xFF
        b = c & 0xFF
        self.setFill(r, g, b)
        
    def setStrokeHex(self, c):
        c = int(c[1:], 16)
        r = (c >> 16) & 0xFF
        g = (c >> 8) & 0xFF
        b = c & 0xFF
        self.setStroke(r, g, b)
        
    def rect(self, x, y, width, height):
        JS('''
        this['_p']['rect'](@{{x}}, @{{y}}, @{{width}}, @{{height}});
        ''')
        
    def setBackground(self, r, g, b):
        JS('''
        this['_p']['background'](@{{r}}, @{{g}}, @{{b}});
        ''')
        
    def line(self, x1, y1, x2, y2):
        JS('''
        this['_p']['line'](@{{x1}}, @{{y1}}, @{{x2}}, @{{y2}});
        ''')
    
    def getTextWidth(self, text):
        JS('''
        return this['_p']['textWidth'](@{{text}});
        ''')
    
    def text(self, text, x, y):
        JS('''
        this['_p']['text'](@{{text}}, @{{x}}, @{{y}});
        ''')
        
    def setTextSize(self, size):
        JS('''
        this['_p']['textSize'](@{{size}});
        ''')
        
    def setFrameRate(self, rate):
        JS('''
        this['_p']['frameRate'](@{{rate}});
        ''')
        
    def loop(self):
        JS('''
        this['_p']['loop']();
        ''')
        
    def noLoop(self):
        JS('''
        this['_p']['noLoop']();
        ''')
        
    def textAlign(self, align):
        if align == 'CENTER':
            JS('''this['_p']['textAlign'](this['_p']['CENTER']);''')
        elif align == 'RIGHT':
            JS('''this['_p']['textAlign'](this['_p']['RIGHT']);''')
        elif align == 'LEFT':
            JS('''this['_p']['textAlign'](this['_p']['LEFT']);''')
            
    def setStroke(self, r, g, b):
        JS('''
        this['_p']['stroke'](@{{r}}, @{{g}}, @{{b}});
        ''')
        
    def setMouseMove(self, mouseCallback):
        JS('''
        this['_p']['mouseMoved'] = @{{mouseCallback}};
        ''')
        
    def setMouseDown(self, mouseCallback):
        JS('''
        this['_p']['mousePressed'] = @{{mouseCallback}};
        ''')
        
    def setMouseUp(self, mouseCallback):
        JS('''
        this['_p']['mouseReleased'] = @{{mouseCallback}};
        ''')
        
    def setMouseDrag(self, mouseCallback):
        JS('''
        this['_p']['mouseDragged'] = @{{mouseCallback}};
        ''')
        
    def noStroke(self):
        JS('''
        this['_p']['noStroke']();
        ''')

    def triangle(self, x1, y1, x2, y2, x3, y3):
        JS('''
        this['_p']['triangle'](@{{x1}}, @{{y1}}, @{{x2}}, @{{y2}}, @{{x3}}, @{{y3}});
        ''')
        
    def pushMatrix(self):
        JS('''
        this['_p']['pushMatrix']();
        ''')
    
    def popMatrix(self):
        JS('''
        this['_p']['popMatrix']();
        ''')
        
    def translate(self, x, y):
        JS('''
        this['_p']['translate'](@{{x}}, @{{y}});
        ''')
        
        

        
        
        
    
    
    
    
    
    
    
    
    
    
    
        
