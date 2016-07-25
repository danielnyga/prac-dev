'''
Created on Sep 2, 2015

@author: seba
'''
import re
import traceback
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from ies_models.Frame import Frame
from prac.pracutils.RolequeryHandler import RolequeryHandler

from pymongo import MongoClient
import pymongo
import json
from pracmln.mln.errors import NoSuchPredicateError
from pracmln.mln.base import Predicate
from ies_utils import PracDatabaseHandler
import os
from ies_models.FrameExtractorResult import FrameExtractorResult
from ies_models.ProcessTextFileResult import ProcessTextFileResult
from ies_models import Constants
from ies_models.LogFileSentenceRepresentation import LogFileSentenceRepresentation
from ies_models.FrameBuilderResult import FrameBuilderResult
from ies_models.Exceptions import NoPredicateExtracted,NoValidFrame
from ies_models.Sense import convert_word_to_lemma,get_synset, nounTags

def store_frames_into_database(text_file_name,frames):
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.PRAC
    frames_collection = ies_mongo_db.Frames
    plan_list = []
    
    actioncore = "UNKNOWN"
    roles_dict = {}

    try:
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)
    
        #Parse text file name to annotate it in the mongo db
        inference = PRACInference(prac, ["{}.".format(os.path.basename(text_file_name))])
        while inference.next_module() != 'role_look_up'  and inference.next_module() != 'achieved_by'  and inference.next_module() != 'plan_generation':
        
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)
    
        db = inference.inference_steps[-1].output_dbs[0]
        roles_dict = RolequeryHandler.query_roles_and_senses_based_on_action_core(db)
    
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
    
    except:
        actioncore = "UNKNOWN" 

    for frame in frames:
        plan_list.append(json.loads(frame.to_json_str()))

    try:
        document = {'_id' : text_file_name,"action_core" : actioncore, "action_roles" : roles_dict,'plan_list' : plan_list}    
        frames_collection.insert_one(document)
    except pymongo.errors.DuplicateKeyError:
        frames_collection.delete_many({"_id" : document['_id']})
        frames_collection.insert_one(document)
    except:
        traceback.print_exc()
        
    mongo_client.close()

def store_logs_into_database(logs):
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.PRAC
    logs_collection = ies_mongo_db.Logs
    
    log_file = open("logs.log","w")
    store_results_into_database(logs, logs_collection,log_file)
    log_file.close()
    mongo_client.close()

def store_results_into_database(data_list,collection,log_file):
    
    for data in data_list:
        try:
            json_dict = json.loads(data.to_json_str())
            collection.insert_one(json_dict)
        except pymongo.errors.DuplicateKeyError:
            collection.delete_many({"_id" : json_dict['_id']})
            collection.insert_one(json_dict)
        except:
            traceback.print_exc()
            log_file.write("###########")
            log_file.write(data.to_json_str())
            log_file.write("###########")

class FrameExtractor(object):
    '''
    classdocs
    '''

    #TODO Create an configuration object to handle extraction constraints
    def __init__(self, corpus):
        '''
        Constructor
        '''
        self.corpus = corpus
        self.prac = PRAC()
        self.prac.wordnet = WordNet(concepts=None)
        self.parser = self.prac.getModuleByName('nl_parsing')
        self.parser.initialize()
        self.parser.mln.declare_predicate(Predicate('prepobj',['word','word']))
        self.parser.mln.declare_predicate(Predicate('has_sense',['word','sense!']))
        self.parser.mln.declare_predicate(Predicate('is_a',['sense','concept']))
        self.result = FrameExtractorResult()
    
    def parse_sentence(self,sentence):
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)

        inference = PRACInference(prac, [sentence])
        parser = prac.getModuleByName('nl_parsing')
        prac.run(inference, parser)
        
        return inference.inference_steps[-1].output_dbs
         
    def extract_frames(self):
        i = 0
        for path_to_text_file in self.corpus:
            i += 1
            print "{} file of {} files: {}".format(str(i),str(len(self.corpus)),path_to_text_file)
            
            #self.result.frame_list.extend(self.process_text_file(path_to_text_file))
            extracted_frame_list = self.process_text_file(path_to_text_file)
            if extracted_frame_list:
                store_frames_into_database(extracted_frame_list[0].text_source_file,extracted_frame_list)
        print "DONE FRAME EXTRACTION"
        return self.result
    
    def build_frames(self,text_source_file,sentence_number,sentence,db):
        result = FrameBuilderResult()
        frame_list = []
        frame_id = 0
        
        predicate_query_result = PracDatabaseHandler.get_all_predicates_as_senses(db)
        predicate_list = predicate_query_result.sense_list
        
        
        if not predicate_list:
            raise NoPredicateExtracted()
        result.add_sense_query_result(predicate_query_result)
        
        for predicate in predicate_list:
            nsubj_query_result = PracDatabaseHandler.get_all_nsubjs_as_sense(db, predicate)
            dobj_query_result = PracDatabaseHandler.get_all_dobjs_as_sense(db, predicate)
            prepobj_query_result = PracDatabaseHandler.get_all_prepobjs_as_sense(db, predicate)
            iobjs_query_result = PracDatabaseHandler.get_all_iobjs_as_sense(db, predicate)
                
            if nsubj_query_result.sense_list or dobj_query_result.sense_list or prepobj_query_result.sense_list or iobjs_query_result.sense_list:
                
            
                result.add_sense_query_result(nsubj_query_result)
                result.add_sense_query_result(dobj_query_result)
                result.add_sense_query_result(prepobj_query_result)
                result.add_sense_query_result(iobjs_query_result)
                
                slot_values_list = [{Constants.SLOT_VALUE_PREDICATE : predicate}]
                
                #To handle sentences like Bob and Alice play the violin and piano
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          nsubj_query_result.sense_list,
                                                                          Constants.SLOT_VALUE_NSUBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          dobj_query_result.sense_list,
                                                                          Constants.SLOT_VALUE_DOBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          prepobj_query_result.sense_list,
                                                                          Constants.SLOT_VALUE_PREPOBJ)
                
                slot_values_list = self.create_permutation_of_slot_values(slot_values_list,
                                                                          iobjs_query_result.sense_list,
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

        result.frame_list = frame_list 
        
        return result
        
    def process_text_file(self, path_to_text_file):
        
        text_file = open(path_to_text_file, 'r')
        
        text_file_name = os.path.split(path_to_text_file)[1]
        text_file_parent_dir  = os.path.split(os.path.split(path_to_text_file)[0])[1]
        text_source_file = "{}/{}".format(text_file_parent_dir,text_file_name)
        
        result = ProcessTextFileResult(text_source_file)
        sentences = text_file.readlines()
        sentence_number = 0
        frame_list = [] 
        
        result.num_sentences = len(sentences)
        
        for s in sentences:
            sentence = s.strip().replace('"','')
            sentence = sentence.strip().replace("'",'')
            
            sentence = self.process_imperative_sentence(sentence)
            is_sentence_parsed = False
            sentence_number += 1
            dbs = None

            while not is_sentence_parsed:  
                try:
                    dbs = self.parse_sentence(sentence)
                    frame_builder_results = []
                    
                    for db in dbs:
                        frame_builder_results.append(self.build_frames(text_source_file, sentence_number, sentence, db))
                    
                    for frame_builder_result in frame_builder_results:    
                        result.add_frame_builder_result(frame_builder_result)
                        frame_list.extend(frame_builder_result.frame_list)
                    
                    is_sentence_parsed = True
                   
                except NoSuchPredicateError:
                    _, exc_value , _ = sys.exc_info()
                    predicate_name = str(exc_value).split(' ')[1].strip()
                    self.parser.mln.declare_predicate(Predicate(predicate_name,['word','word']))
                    
                except NoPredicateExtracted:
                    _, exc_value , _ = sys.exc_info()
                    result.no_predicate_sentences_list.append(LogFileSentenceRepresentation(sentence,db.mln,db))
                    result.num_no_predicate_sentences += 1
                    result.num_errors += 1
                    is_sentence_parsed = True
                except NoValidFrame:
                    result.no_valid_frame_sentences_list.append(LogFileSentenceRepresentation(sentence,db.mln,db))
                    result.num_no_frame_sentences += 1
                    result.num_errors += 1
                    is_sentence_parsed = True
                except Exception:
                    #TODO add logger
                    traceback.print_exc()
                    
                    result.parsing_error_sentences_list.append(LogFileSentenceRepresentation(sentence,None,None))
                    result.num_parsing_errors_sentences += 1
                    result.num_errors += 1
                    is_sentence_parsed = True
        self.result.add_process_text_file_result(result)
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
        #Currently  fixes for imperative sentence are deactivated
        
        #Setting sentences in quotation marks can fix the parsing process
        return '"{}"'.format(sentence)
        
        #Writing the verb in a imperative sentence with lower case can improve the parsing results        
        first_word = re.split("\s+", sentence.strip())[0]
        syns = get_synset(first_word, 'v')
        
        if len(syns) > 0:
            return sentence.replace(first_word,first_word.lower(),1)
        '''
        return sentence
