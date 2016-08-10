'''
Created on Sep 3, 2015

@author: seba
'''
from prac.db.ies.ies_models import Constants
from prac.db.ies.ies_models.Sense import Sense
from prac.db.ies.ies_models.Frame import Frame
from pracmln.mln.database import Database

import re

regex_pobj_str = 'nmod_(\w+)\(\s*{}\s*,\s*([^\)\s]+)\s*'
   
def get_all_objs_as_sense(db,obj_mln_predicate,predicate=None):
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
    
    for q in db.query(obj_mln_predicate.format(predicate_word,'?w')):
        obj_word = q['?w']
        result.extend(add_obj_word_to_result_list(obj_word, db))
        
        for conj_query in ['conj_and','conj_or']:
            for q1 in db.query("{}({},{})".format(conj_query,obj_word,"?w")):
                conj_obj_word = q1['?w']
                result.extend(add_obj_word_to_result_list(conj_obj_word, db))
    return result

def add_obj_word_to_result_list(obj_word_,db,misc=""):
    obj_word =  obj_word_
    result = []
    if not is_pronoun(obj_word, db) and not is_wh(obj_word, db):
            #check if dobj+prep combi (we use just nmod_of)
            for q in db.query("nmod_of({},?w1)".format(obj_word)):
                obj_word = q['?w1']
                break
                    
            obj_pos = get_penn_treebank_pos_of_word(db,obj_word)
            obj_sense = Sense(obj_word,obj_pos,misc=misc)
            result.append(obj_sense)
            
    return result
            
def get_all_dobjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, Constants.DOBJ_MLN_PREDICATE, predicate)

def get_all_nsubjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, Constants.NSUBJ_MLN_PREDICATE, predicate)

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
    return get_all_objs_as_sense(db, Constants.IOBJ_MLN_PREDICATE, predicate)

def get_all_predicates_as_senses(db):
    valid_predicate_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
    predicate_list=[]
    
    for q in db.query(Constants.HAS_POS_MLN_PREDICATE.format('?w','?p')):
        word_pos = q['?p']
        word = q['?w']
        
        if word_pos in valid_predicate_tags and not is_aux_verb(word, db):
            predicate_sense = Sense(word,word_pos)
            predicate_list.append(predicate_sense)
    
    return predicate_list

def get_penn_treebank_pos_of_word(db,word):
    pos = ""
    #Assuming there is only one pos for every word
    for q in db.query(Constants.HAS_POS_MLN_PREDICATE.format(word,'?p')):
            pos = q['?p']
    
    return pos

def is_aux_verb(word,db):

        for _ in db.query('aux(?w, {})'.format(word)):
            return True;
        
        for _ in db.query('auxpass(?w, {})'.format(word)):
            return True;
        
        return False;

def is_pronoun(word,db):
        for _ in db.query('has_pos({}, PRP)'.format(word)):
            return True;
        
        for _ in db.query('has_pos({}, PRP$)'.format(word)):
            return True;
        
        return False;

def is_wh(word,db):
        
        for _ in db.query('has_pos({}, WDT)'.format(word)):
            return True;
        
        for _ in db.query('has_pos({}, WP)'.format(word)):
            return True;
        
        return False;

    
