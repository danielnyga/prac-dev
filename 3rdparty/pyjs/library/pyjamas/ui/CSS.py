""" CSS Stylesheet messing

    Copyright (C) 2010, Stolati <mickael.kerbrat@gmail.com>
    Copyright (C) 2010, Luke Kenneth Casson Leighton <lkcl@lkcl.net>
"""

from __pyjamas__ import doc
from pyjamas import DOM

class StyleSheetCssFile:

    def __init__(self, cssFile=''):
        self._e = DOM.createElement('link')
        self._e.setAttribute('rel', 'stylesheet')
        self._e.setAttribute('type', 'text/css')
        self._e.setAttribute('href', cssFile);

        print "css file", cssFile
        doc().getElementsByTagName("head").item(0).appendChild(self._e) 

    def remove(self):
        parent = DOM.getParent(self._e)
        DOM.removeChild(parent, self._e) 
        
def setStyleElementText(el, text):
    DOM.appendChild(el, doc().createTextNode(text))
    
class StyleSheetCssText:

    def __init__(self, text=''):
        self._e = DOM.createElement('style')
        self._e.setAttribute('type', 'text/css')
        setStyleElementText(self._e, text)

        doc().getElementsByTagName("head").item(0).appendChild(self._e) 

    def remove(self):
        parent = DOM.getParent(self._e)
        DOM.removeChild(parent, self._e) 
        
