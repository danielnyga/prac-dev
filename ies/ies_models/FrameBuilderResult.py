'''
Created on Sep 6, 2015

@author: seba
'''
from ies_models.ResultBase import ResultBase

class FrameBuilderResult(ResultBase):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        super(FrameBuilderResult, self).__init__()
        self.frame_list = []
    
    def add_sense_query_result(self,query_result):
        self.num_nouns += query_result.num_nouns 
        self.num_verbs += query_result.num_verbs
        self.num_adjs += query_result.num_adjs
        self.num_unk += query_result.num_unk
        self.num_assert_senses += query_result.num_assert_senses 