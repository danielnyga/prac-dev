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
        
        self.setWidth('100%')
        self.setText(0, 0, 'arg max')
        self.setText(0, 1, 'P(')
        self.setHTML(0, 2, '')
        self.setText(0, 3, '|')
        self.setText(0, 4, '')
        self.setText(0, 5, ')')
        
        
    def setEvidence(self, evidence):
        evidenceString = '<div style="background: %s; border: 1px solid %s;">%s</div>'
        content = ''
        for i, e in enumerate(evidence):
            color = evidence[e]
            content += evidenceString % (colors_fill[color], colors_stroke[color], e)
            if i < len(evidence)-1:
                content += ','
        self.setText(0, 4, content)
        
    def setQuery(self, query):
        self.setText(0, 3, ','.join(query))