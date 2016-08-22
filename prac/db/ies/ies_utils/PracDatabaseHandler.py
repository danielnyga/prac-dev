'''
Created on Sep 3, 2015

@author: seba
'''
from prac.db.ies.models import constants
from prac.db.ies.models.sense import Sense
from prac.db.ies.models.frame import Frame

import re

regex_pobj_str = 'nmod_(\w+)\(\s*{}\s*,\s*([^\)\s]+)\s*'
   
def get_all_objs_as_sense(db, obj_mln_predicate, predicate=None):
    predicate_word = ""
    result = []
    
    if type(predicate) is str:
        predicate_word = str(predicate)
    elif type(predicate) is Sense:
        predicate_word = predicate.word
    elif type(predicate) is Frame:
        predicate_word = predicate.predicate.word
    
    #If no predicate_word is defined, that means that all objects will be extracted
    if not predicate_word:
        predicate_word = '?w1'
    
    for q in db.query(obj_mln_predicate.format(predicate_word, '?w')):
        obj_word = q['?w']
        result.extend(add_obj_word_to_result_list(obj_word, db))
        
        for conj_query in ['conj_and','conj_or']:
            for q1 in db.query("{}({},{})".format(conj_query,obj_word, "?w")):
                conj_obj_word = q1['?w']
                result.extend(add_obj_word_to_result_list(conj_obj_word, db))
    return result


def add_obj_word_to_result_list(word, db, misc=""):
    result = []
    if not db.is_pronoun(word) and not db.is_wh(word):
        #check if dobj+prep combi (we use just nmod_of)
        for q in db.query("nmod_of({},?w1)".format(word)):
            word = q['?w1']
            break
        for obj_pos in db.postag(word=word): break
        obj_sense = Sense(word,obj_pos,misc=misc)
        result.append(obj_sense)
            
    return result


def get_all_dobjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, constants.DOBJ_MLN_PREDICATE, predicate)


def get_all_nsubjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, constants.NSUBJ_MLN_PREDICATE, predicate)


def get_all_prepobjs_as_sense(db,predicate=None):
    predicate_word = "[^,]"    

    if type(predicate) is str:
        predicate_word = str(predicate)
    elif type(predicate) is Sense:
        predicate_word = predicate.word
    elif type(predicate) is Frame:
        predicate_word = predicate.predicate.word

    regex_pobj = re.compile(regex_pobj_str.format(predicate_word))    
    result = []
    
    
    for atom, truth in db.evidence.iteritems():
        regex_pobj_result = regex_pobj.search(atom)
        if regex_pobj_result and (int(truth) == 1):
            prep_word = regex_pobj_result.group(1) 
            obj_word = regex_pobj_result.group(2)
            result.extend(add_obj_word_to_result_list(obj_word, db,misc=prep_word))
            
            for conj_query in ['conj_and','conj_or']:
                for q1 in db.query("{}({},{})".format(conj_query,obj_word,"?w")):
                    conj_obj_word = q1['?w']
                    result.extend(add_obj_word_to_result_list(conj_obj_word, db,misc=prep_word))
                    
    return result

def get_all_iobjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, constants.IOBJ_MLN_PREDICATE, predicate)

def get_all_predicates_as_senses(db):
    valid_predicate_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
    predicate_list=[]
    for word, pos in db.postags():
        if pos in valid_predicate_tags and not db.is_aux_verb(word):
            predicate_sense = Sense(word, pos)
            predicate_list.append(predicate_sense)
    
    return predicate_list





