'''
Created on Sep 6, 2015

@author: seba
'''
from ies_models.ResultBase import ResultBase 

class SenseResult(ResultBase):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        
        super(SenseResult, self).__init__()
        self.sense_list = []
        
        