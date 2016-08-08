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

def slot_dict_to_sorted_str_list(slot_dict):
    key_list_sorted = sorted(slot_dict, key=lambda k: slot_dict[k], reverse=True)
    
    result = []
    for key in key_list_sorted:
        result.append("{} : {}".format(key,str(slot_dict[key])))
    
    return result

if __name__ == '__main__':
    frame_list = MongoDatabaseHandler.get_frames_based_on_query({})
    predicate_stats_dict = {}
    
    for frame in frame_list:
        if frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma in predicate_stats_dict.keys():
            predicate_stats_dict[frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma].update(frame)
        else:
            predicate_stats_dict[frame.slot_values[Constants.SLOT_VALUE_PREDICATE].lemma] = PredicateStats(frame)
            
    
    key_list_sorted = sorted(predicate_stats_dict, key=lambda k: predicate_stats_dict[k].num_predicate, reverse=True)
    
    target = open("Stats", 'w')

    for key in key_list_sorted:
        pred_stats_obj = predicate_stats_dict[key]
        target.write("Verb: {}\n".format(key))
        target.write("Number: {}\n".format(str(pred_stats_obj.num_predicate)))
        target.write("Dobjs: {}\n".format(slot_dict_to_sorted_str_list(pred_stats_obj.dobj_dict)))
        target.write("Iobjs: {}\n".format(slot_dict_to_sorted_str_list(pred_stats_obj.iobj_dict)))
        target.write("Prepobjs: {}\n".format(slot_dict_to_sorted_str_list(pred_stats_obj.prepobj_dict)))
        target.write("Nsubjs: {}\n".format(slot_dict_to_sorted_str_list(pred_stats_obj.nsubj_dict)))
        target.write("---------------------------\n")
        target.write("\n")

    target.close()
