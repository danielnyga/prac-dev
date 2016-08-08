'''
Created on Sep 5, 2015

@author: seba
'''
from prac.db.ies.ies_models.ResultBase import ResultBase 

class ProcessResultBase(ResultBase):
    '''
    classdocs
    '''


    def __init__(self):
        super(ProcessResultBase, self).__init__()
        self.num_slot_values = {}
        self.num_sentences = 0
        self.num_extracted_frames = 0
        self.num_errors = 0
        self.num_parsing_errors_sentences = 0
        self.num_no_predicate_sentences = 0
        self.num_no_frame_sentences = 0
        