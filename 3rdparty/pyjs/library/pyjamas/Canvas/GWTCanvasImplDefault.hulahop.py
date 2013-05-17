def cvt(s):
    if isinstance(s, str): 
        return unicode(s)
    return s

class GWTCanvasImplDefault:

    def createElement(self):
        e = DOM.createElement("CANVAS")
        self.setCanvasContext(e.MozGetIPCContext(u'2d'))
        return e


