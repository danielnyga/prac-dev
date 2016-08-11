'''
Created on Sep 2, 2015

@author: seba
'''

import traceback
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from prac.db.ies.ies_models.Frame import Frame

from pymongo import MongoClient
import pymongo
import json
from pracmln.mln.errors import NoSuchPredicateError
from pracmln.mln.base import Predicate
from prac.db.ies.ies_utils import PracDatabaseHandler
import os
from prac.db.ies.ies_models import Constants
from prac.db.ies.ies_models.Exceptions import NoPredicateExtracted,NoValidFrame


class FrameExtractor(object):
    '''
    classdocs
    '''

    def __init__(self, howtos,verbose=1):
        '''
        Constructor
        '''
        self.howtos = howtos
        self.prac = PRAC()
        self.prac.verbose = verbose
        self.prac.wordnet = WordNet(concepts=None)
        self.parser = self.prac.module('nl_parsing')
        self.parser.initialize()
        self.parser.mln.declare_predicate(Predicate('prepobj',['word','word']))
        self.parser.mln.declare_predicate(Predicate('has_sense',['word','sense!']))
        self.parser.mln.declare_predicate(Predicate('is_a',['sense','concept']))
    
    def parse_sentence(self,sentence):
        self.prac.wordnet = WordNet(concepts=None)

        inference = PRACInference(self.prac, [sentence])
        parser = self.prac.module('nl_parsing')
        self.prac.run(inference, parser)
        
        return inference.inference_steps[-1].output_dbs
         
    def extract_frames(self):
        i = 0
        for howto in self.howtos:
            i += 1
            howto_name = howto.keys()[0]
            steps = howto.values()[0]
            if self.prac.verbose > 0:
                print "{} howto of {} howtos: {}".format(str(i),str(len(self.howtos)),howto_name)
            
            extracted_frame_list = self.process_howto(howto_name, steps)
            if extracted_frame_list:
                self.store_frames_into_database(extracted_frame_list[0].text_source_file,extracted_frame_list)
        
        if self.prac.verbose > 0:        
            print "Thread is done with processing howtos."
    
    def build_frames(self,text_source_file,sentence_number,sentence,db):
        frame_list = []
        frame_id = 0
        
        predicate_list = PracDatabaseHandler.get_all_predicates_as_senses(db)
        
        if not predicate_list:
            raise NoPredicateExtracted()
        
        for predicate in predicate_list:
            nsubj_query_result = PracDatabaseHandler.get_all_nsubjs_as_sense(db, predicate)
            dobj_query_result = PracDatabaseHandler.get_all_dobjs_as_sense(db, predicate)
            prepobj_query_result = PracDatabaseHandler.get_all_prepobjs_as_sense(db, predicate)
            iobjs_query_result = PracDatabaseHandler.get_all_iobjs_as_sense(db, predicate)
                
            if nsubj_query_result or dobj_query_result or prepobj_query_result or iobjs_query_result:
                
                slot_values_list = [{Constants.SLOT_VALUE_PREDICATE : predicate}]
                
                #To handle sentences like Bob and Alice play the violin and piano
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          nsubj_query_result,
                                                                          Constants.SLOT_VALUE_NSUBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          dobj_query_result,
                                                                          Constants.SLOT_VALUE_DOBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          prepobj_query_result,
                                                                          Constants.SLOT_VALUE_PREPOBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          iobjs_query_result,
                                                                          Constants.SLOT_VALUE_IOBJ)
                
              
                for slot_values in slot_values_list:
                    frame = Frame(text_source_file,
                         sentence_number,
                         frame_id,
                         sentence,
                         db.mln,
                         db,
                         slot_values,
                         prac=self.prac)
                    frame_list.append(frame)
                    frame_id += 1
        if not frame_list:                 
            raise NoValidFrame()        

        return frame_list
        
    def process_howto(self, howto_name,steps):
        
        sentence_number = 0
        frame_list = [] 
        
        for s in steps:
            sentence = s.strip().replace('"','')
            sentence = sentence.strip().replace("'",'')
            
            sentence = self.process_imperative_sentence(sentence)
            is_sentence_parsed = False
            sentence_number += 1
            dbs = None

            while not is_sentence_parsed:  
                try:
                    dbs = self.parse_sentence(sentence)
                    
                    for db in dbs:
                        frame_list.extend(self.build_frames(howto_name, sentence_number, sentence, db))
                    is_sentence_parsed = True
                   
                except NoSuchPredicateError:
                    _, exc_value , _ = sys.exc_info()
                    predicate_name = str(exc_value).split(' ')[1].strip()
                    self.parser.mln.declare_predicate(Predicate(predicate_name,['word','word']))
                    
                except NoPredicateExtracted:
                    _, exc_value , _ = sys.exc_info()
                    is_sentence_parsed = True
                except NoValidFrame:
                    is_sentence_parsed = True
                except Exception:
                    traceback.print_exc()
                    is_sentence_parsed = True
                    
        return frame_list
    
    def create_permutation_of_slot_values(self,list_of_current_slot_value_dicts,list_of_new_slot_values,new_slot_value_key):
        result = []
        
        #To handle empty slot values
        if not list_of_new_slot_values:
            for current_slot_values_dict in list_of_current_slot_value_dicts:
                current_slot_values_dict_copy = current_slot_values_dict.copy()
                current_slot_values_dict_copy[new_slot_value_key] = None
                result.append(current_slot_values_dict_copy)
        
        else:
            for new_slot_value in list_of_new_slot_values:
                for current_slot_values_dict in list_of_current_slot_value_dicts:
                    current_slot_values_dict_copy = current_slot_values_dict.copy() 
                    current_slot_values_dict_copy[new_slot_value_key] = new_slot_value
                    result.append(current_slot_values_dict_copy)
            
        return result
    
                
    def process_imperative_sentence(self,sentence):
        return sentence
        '''
        #Currently the fixes for imperative sentence are deactivated.
        
        #Setting sentences in quotation marks can fix the parsing process
        return '"{}"'.format(sentence)
        
        #Writing the verb in a imperative sentence with lower case can improve the parsing results        
        first_word = re.split("\s+", sentence.strip())[0]
        syns = get_synset(first_word, 'v')
        
        if len(syns) > 0:
            return sentence.replace(first_word,first_word.lower(),1)
        '''
        return sentence
    
    def store_frames_into_database(self,text_file_name,frames):
        mongo_client =  MongoClient(host=self.prac.config.get('mongodb', 'host'), 
                                    port=self.prac.config.getint('mongodb', 'port'))
        ies_mongo_db = mongo_client.prac
        frames_collection = ies_mongo_db.howtos
        plan_list = []
        
        actioncore = "UNKNOWN"
        roles_dict = {}
    
        try:
            self.prac.wordnet = WordNet(concepts=None)
        
            #Parse text file name to annotate it in the mongo db
            inference = PRACInference(self.prac, ["{}.".format(os.path.basename(text_file_name))])
            while inference.next_module() != 'role_look_up'  and inference.next_module() != 'achieved_by'  and inference.next_module() != 'plan_generation':
            
                modulename = inference.next_module()
                module = self.prac.module(modulename)
                self.prac.run(inference, module)
        
            db = inference.inference_steps[-1].output_dbs[0]
            
            for result_ac in db.actioncores():
                for result_role in db.roles(result_ac.values().pop()):
                    roles_dict[result_role.keys()[0]] = result_role.values()[0]
            
            #It will be assumed that there is only one true action_core predicate per database 
            for q in db.query("action_core(?w,?ac)"):
                actioncore = q["?ac"]
        
        except:
            actioncore = "UNKNOWN" 
    
        for frame in frames:
            plan_list.append(json.loads(frame.to_json_str()))
    
        try:
            document = {'_id' : text_file_name,
                        Constants.JSON_HOWTO_ACTIONCORE : actioncore, 
                        Constants.JSON_HOWTO_ACTIONCORE_ROLES : roles_dict,
                        Constants.JSON_HOWTO_STEPS : plan_list}
                
            frames_collection.insert_one(document)
        except pymongo.errors.DuplicateKeyError:
            frames_collection.delete_many({"_id" : document['_id']})
            frames_collection.insert_one(document)
        except:
            traceback.print_exc()
            
        mongo_client.close()
