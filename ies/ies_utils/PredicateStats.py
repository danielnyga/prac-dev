'''
Created on Jan 25, 2016

@author: seba
'''
from ies_models import Constants
from ies_utils import MongoDatabaseHandler

class PredicateStats(object):
    '''
    classdocs
    '''


    def __init__(self,frame):
        '''
        Constructor
        '''
        
        self.predicate_lemma = ""
        self.num_predicate = 0
        self.dobj_dict = {}
        self.iobj_dict = {}
        self.prepobj_dict = {}
        self.nsubj_dict = {}
        
        self.update(frame)
        
    def update(self,frame):
        slot_values = frame.slot_values
        predicate_sense = slot_values[Constants.SLOT_VALUE_PREDICATE]
        
        if not self.predicate_lemma or (predicate_sense.lemma == self.predicate_lemma):
            self.num_predicate += 1
            dobj_sense = slot_values[Constants.SLOT_VALUE_DOBJ]
            iobj_sense = slot_values[Constants.SLOT_VALUE_IOBJ]
            prepobj_sense = slot_values[Constants.SLOT_VALUE_PREPOBJ]
            nsubj_sense = slot_values[Constants.SLOT_VALUE_NSUBJ]
            
            if dobj_sense:
                if dobj_sense.lemma in self.dobj_dict.keys():
                    self.dobj_dict[dobj_sense.lemma] += 1
                else:
                    self.dobj_dict[dobj_sense.lemma] = 1
            
            if iobj_sense:
                if iobj_sense.lemma in self.iobj_dict.keys():
                    self.iobj_dict[iobj_sense.lemma] += 1
                else:
                    self.iobj_dict[iobj_sense.lemma] = 1
            
            if prepobj_sense:
                if prepobj_sense.lemma in self.prepobj_dict.keys():
                    self.prepobj_dict[prepobj_sense.lemma] += 1
                else:
                    self.prepobj_dict[prepobj_sense.lemma] = 1
            
            if nsubj_sense:
                if nsubj_sense.lemma in self.nsubj_dict.keys():
                    self.nsubj_dict[nsubj_sense.lemma] += 1
                else:
                    self.nsubj_dict[nsubj_sense.lemma] = 1

if __name__ == '__main__':
    frame_list = MongoDatabaseHandler.get_frames_based_on_query({})
    predicate_stats_dict = {}
    
    for frame in frame_list:
        if frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma in predicate_stats_dict.keys():
            predicate_stats_dict[frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma].update(frame)
        else:
            predicate_stats_dict[frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma] = PredicateStats(frame)
            
    
    print predicate_stats_dict