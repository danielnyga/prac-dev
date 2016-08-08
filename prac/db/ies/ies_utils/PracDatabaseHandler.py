'''
Created on Sep 3, 2015

@author: seba
'''
from prac.db.ies.ies_models import Constants
from prac.db.ies.ies_models.Sense import Sense
from prac.db.ies.ies_models.Frame import Frame
from prac.db.ies.ies_models.SenseResult import SenseResult
from pracmln.mln.database import Database
import cStringIO
import re
from prac.db.ies.ies_utils import MongoDatabaseHandler
import traceback


regex_pobj_str = 'nmod_(\w+)\(\s*{}\s*,\s*([^\)\s]+)\s*'
def generate_ac_train_optimazed_prac_db(query,db_path,merging=True):
    dobj_bucket = []
    prepobj_bucket = []
    combi_bucket = []
    
    frames = MongoDatabaseHandler.get_frames_based_on_query(query)
    
    for frame in frames:
        frame.slot_values[Constants.SLOT_VALUE_NSUBJ] = None
        if frame.is_valid():
            if frame.slot_values[Constants.SLOT_VALUE_DOBJ] and frame.slot_values[Constants.SLOT_VALUE_PREPOBJ]:
                combi_bucket.append(frame)
            elif frame.slot_values[Constants.SLOT_VALUE_DOBJ]:
                dobj_bucket.append(frame)
            elif frame.slot_values[Constants.SLOT_VALUE_PREPOBJ]:
                prepobj_bucket.append(frame)
    
    combi_bucket = remove_duplicates_in_double_slot_value_bucket(combi_bucket, Constants.SLOT_VALUE_DOBJ, Constants.SLOT_VALUE_PREPOBJ)
    dobj_bucket = remove_duplicates_in_one_slot_value_bucket(dobj_bucket, Constants.SLOT_VALUE_DOBJ)
    prepobj_bucket = remove_duplicates_in_one_slot_value_bucket(prepobj_bucket, Constants.SLOT_VALUE_PREPOBJ)
    
    dobj_bucket = remove_duplicates_between_one_and_two_slot_value_buckets(dobj_bucket,combi_bucket,Constants.SLOT_VALUE_DOBJ)
    prepobj_bucket = remove_duplicates_between_one_and_two_slot_value_buckets(prepobj_bucket,combi_bucket,Constants.SLOT_VALUE_PREPOBJ)
    
    if merging:
        merged_container = []
        merged_container.extend(dobj_bucket)
        merged_container.extend(prepobj_bucket)
        merged_container.extend(combi_bucket)
        write_frames_as_prac_dbs(merged_container,"{}".format(db_path))
    else:
        write_frames_as_prac_dbs(combi_bucket, "{}_combi.db".format(db_path))
        write_frames_as_prac_dbs(dobj_bucket, "{}_dobj.db".format(db_path))
        write_frames_as_prac_dbs(prepobj_bucket, "{}_prepobj.db".format(db_path))

def remove_duplicates_between_one_and_two_slot_value_buckets(one_bucket,two_bucket,slot_value_name):
    new_bucket = []
    lemma_set = set()
    
    for frame in two_bucket:
        lemma_set.add(frame.slot_values[slot_value_name].lemma)
    
    for frame in one_bucket:
        if not frame.slot_values[slot_value_name].lemma in lemma_set:
            new_bucket.append(frame)
    
    return new_bucket

def remove_duplicates_in_double_slot_value_bucket(bucket,slot_value_name_1, slot_value_name_2):
    lemma_list_1 = []
    lemma_list_2 = []
    new_bucket = []
    
    for frame in bucket:
        if (not frame.slot_values[slot_value_name_1].lemma in lemma_list_1) and (not frame.slot_values[slot_value_name_2].lemma in lemma_list_2):
            new_bucket.append(frame)
            lemma_list_1.append(frame.slot_values[slot_value_name_1].lemma)
            lemma_list_2.append(frame.slot_values[slot_value_name_2].lemma)
    
    return new_bucket
   
def remove_duplicates_in_one_slot_value_bucket(bucket,slot_value_name):
    lemma_list = []
    new_bucket = []
    
    for frame in bucket:
        if not frame.slot_values[slot_value_name].lemma in lemma_list:
            new_bucket.append(frame)
            lemma_list.append(frame.slot_values[slot_value_name].lemma)
    
    return new_bucket

def write_frames_as_prac_dbs(frames,db_path):
    dbs = []
    for frame in frames:
        try:
            dbs.append(frame.slot_values_to_prac_db())
        except:
            traceback.print_exc()
            print "Cannot convert slot values to prac db"
    
    dbs_file = open(db_path,"w")
    
    for i in range(0,len(dbs)):
        db = dbs[i]
        frame = frames[i]
        dbs_file.write("//{}\n".format(frame.sentence))
        db.write(dbs_file)
        
        if i < (len(dbs) - 1):
            dbs_file.write('\n---\n')
    dbs_file.close()
    
def generate_prac_db_based_on_query(query,db_path):
    frames = MongoDatabaseHandler.get_frames_based_on_query(query)
    write_frames_as_prac_dbs(frames, db_path)
    
def get_all_objs_as_sense(db,obj_mln_predicate,predicate=None):
    result = SenseResult()
    predicate_word = ""
    
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
        add_obj_word_to_result_list(obj_word, db, result)
        
        for conj_query in ['conj_and','conj_or']:
            for q1 in db.query("{}({},{})".format(conj_query,obj_word,"?w")):
                conj_obj_word = q1['?w']
                add_obj_word_to_result_list(conj_obj_word, db, result)
    return result

def add_obj_word_to_result_list(obj_word_,db,result,misc=""):
    obj_word =  obj_word_
    
    if not is_pronoun(obj_word, db) and not is_wh(obj_word, db):
            #check if dobj+prep combi (we use just prep_of)
            for q in db.query("nmod_of({},?w1)".format(obj_word)):
                obj_word = q['?w1']
                break
                    
            obj_pos = get_penn_treebank_pos_of_word(db,obj_word)
            obj_sense = Sense(obj_word,obj_pos,misc=misc)
            result.sense_list.append(obj_sense)
            
            obj_pos = obj_sense.wordnet_pos
            
            if obj_pos == 'n':
                result.num_nouns += 1
            elif obj_pos == 'v':
                result.num_verbs += 1
            elif obj_pos == 'a':
                result.num_adjs += 1
            else:
                result.num_unk += 1
            
            if obj_sense.nltk_wordnet_sense:
                result.num_assert_senses += 1
    
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
    result = SenseResult()
    
    
    for atom, truth in db.evidence.iteritems():
        regex_pobj_result = regex_pobj.search(atom)
        if regex_pobj_result and (int(truth) == 1):
            prep_word = regex_pobj_result.group(1) 
            obj_word = regex_pobj_result.group(2)
            add_obj_word_to_result_list(obj_word, db, result,misc=prep_word)
        
            for conj_query in ['conj_and','conj_or']:
                for q1 in db.query("{}({},{})".format(conj_query,obj_word,"?w")):
                    conj_obj_word = q1['?w']
                    add_obj_word_to_result_list(conj_obj_word, db, result,misc=prep_word)
                    
   
    return result

def get_all_iobjs_as_sense(db,predicate=None):
    return get_all_objs_as_sense(db, Constants.IOBJ_MLN_PREDICATE, predicate)

def get_all_predicates_as_senses(db):
    valid_predicate_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
    predicate_list=[]
    result = SenseResult()
    
    for q in db.query(Constants.HAS_POS_MLN_PREDICATE.format('?w','?p')):
        word_pos = q['?p']
        word = q['?w']
        
        if word_pos in valid_predicate_tags and not is_aux_verb(word, db):
            predicate_sense = Sense(word,word_pos)
            predicate_list.append(predicate_sense)
            result.num_verbs += 1
            
            if predicate_sense.nltk_wordnet_sense:
                result.num_assert_senses += 1
    
    result.sense_list = predicate_list
    
    return result         
    

def get_penn_treebank_pos_of_word(db,word):
    pos = ""
    #Assuming there is only one pos for every word
    for q in db.query(Constants.HAS_POS_MLN_PREDICATE.format(word,'?p')):
            pos = q['?p']
    
    return pos

def is_aux_verb(word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for _ in db.query('aux(?w, {})'.format(word)):
            return True;
        
        for _ in db.query('auxpass(?w, {})'.format(word)):
            return True;
        
        return False;

def is_pronoun(word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for _ in db.query('has_pos({}, PRP)'.format(word)):
            return True;
        
        for _ in db.query('has_pos({}, PRP$)'.format(word)):
            return True;
        
        return False;

def is_wh(word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for _ in db.query('has_pos({}, WDT)'.format(word)):
            return True;
        
        for _ in db.query('has_pos({}, WP)'.format(word)):
            return True;
        
        return False;

    
