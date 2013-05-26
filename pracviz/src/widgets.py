'''
Created on May 24, 2013

@author: nyga
'''

from pyjamas.ui.FlexTable import FlexTable
from graph.graph import colors_fill
from graph.graph import colors_stroke

class CondProbWidget(FlexTable):
    
    def __init__(self, **kwargs):
        FlexTable.__init__(self,  **kwargs)
        
#         self.setWidth('100%')
        self.setHeight('20px')
        self.setText(0, 0, 'arg max')
        self.setText(0, 1, 'P(')
        self.setHTML(0, 2, '')
        self.setText(0, 3, '|')
        self.setText(0, 4, '')
        self.setText(0, 5, ')')
        
        
    def setEvidence(self, evidence):
        outer = '<div style="float: none">%s</div>'
        evidenceStringColored = '<div align="left" style="background: %s; border: 1px solid %s; float: none;">%s</div>'
        evidenceString = '<div align="left" style="float: none;">%s</div>'
        content = ''
        for i, e in enumerate(evidence):
            color = e.get('color', None)
            if color is not None:   
                content += evidenceStringColored % (colors_fill[color], colors_stroke[color], e['label'])
            else:
                content += evidenceString % e['label']
#             if i < len(evidence)-1:
#                 content += '<div style="float: left">,</div>'
        print outer % content
        self.setHTML(0, 4, outer % content)
        
    def setQuery(self, query):
        outer = '<div style="float: none">%s</div>'
        evidenceStringColored = '<div align="left" style="background: %s; border: 1px solid %s; float: none;">%s</div>'
        evidenceString = '<div align="left" style="float: none;">%s</div>'
        content = ''
        for i, e in enumerate(query):
            color = e.get('color', None)
            if color is not None:
                content += evidenceStringColored % (colors_fill[color], colors_stroke[color], e['label'])
            else:
                content += evidenceString % e['label']
#             if i < len(query)-1:
#                 content += '<div style="float: left">,</div>'
        print outer % content
        self.setHTML(0, 2, outer % content)