'''
Created on Sep 2, 2015

@author: seba
'''
import re
import traceback
from prac.core import PRAC
from prac.wordnet import WordNet
import sys
from ies_models.Frame import Frame

from pymongo import MongoClient
import pymongo
import json
from mln.errors import NoSuchPredicateError
from mln.mln import Predicate
from ies_utils import PracDatabaseHandler
import os
from ies_models.FrameExtractorResult import FrameExtractorResult
from ies_models.ProcessTextFileResult import ProcessTextFileResult
from ies_models import Constants
from ies_models.LogFileSentenceRepresentation import LogFileSentenceRepresentation
from ies_models.FrameBuilderResult import FrameBuilderResult
from ies_models.Exceptions import NoPredicateExtracted,NoValidFrame
from ies_models.Sense import convert_word_to_lemma,get_synset, nounTags

def store_frames_into_database(frames):
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.IES
    frames_collection = ies_mongo_db.Frames
    log_file = open("frames.log","w")
    store_results_into_database(frames, frames_collection,log_file)
    log_file.close()
    mongo_client.close()

def store_logs_into_database(logs):
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.IES
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
        self.parser.mln.declarePredicate(Predicate('prepobj',['word','word']))
        self.parser.mln.declarePredicate(Predicate('has_sense',['word','sense!']))
        self.parser.mln.declarePredicate(Predicate('is_a',['sense','concept']))
        self.result = FrameExtractorResult()
        
    def extract_frames(self):
        i = 0
        for path_to_text_file in self.corpus:
            i += 1
            print "{} file of {} files: {}".format(str(i),str(len(self.corpus)),path_to_text_file)
            
            #self.result.frame_list.extend(self.process_text_file(path_to_text_file))
            store_frames_into_database(self.process_text_file(path_to_text_file))
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
            #TODO ADD PREPROCESSING
            sentence = self.process_imperative_sentence(sentence)
            is_sentence_parsed = False
            sentence_number += 1
            db = []

            while not is_sentence_parsed:  
                try:
                    #TODO ADD PREPROCESSING
                    sentence = self.create_compound_nouns(sentence)
                    db = list(self.parser.parse_without_prac(sentence))[0]
                    
                    frame_builder_result = self.build_frames(text_source_file, sentence_number, sentence, db)
                    result.add_frame_builder_result(frame_builder_result)
                    frame_list.extend(frame_builder_result.frame_list)
                    
                    is_sentence_parsed = True
                   
                #TODO add except no_frame and no_predicate
                except NoSuchPredicateError:
                    _, exc_value , _ = sys.exc_info()
                    predicate_name = str(exc_value).split('No such predicate:')[1].strip()
                    self.parser.mln.declarePredicate(Predicate(predicate_name,['word','word']))
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
                    #traceback.print_exc()
                    result.parsing_error_sentences_list.append(LogFileSentenceRepresentation(sentence,None,None))
                    result.num_parsing_errors_sentences += 1
                    result.num_errors += 1
                    is_sentence_parsed = True
        #self.result.add_process_text_file_result(result)
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
    
    def check_amod_nouns(self,sentence):
        result = sentence
        db = list(self.parser.parse_without_prac(result))[0]
        #Check if recongized adj gives possibility to be part of compound nouns e.g baking sheet or washing machine.
        for q1 in db.query('amod(?w1,?w2)'):
            noun = '-'.join(q1['?w1'].split('-')[:-1])
            adj = '-'.join(q1['?w2'].split('-')[:-1])
            syns = get_synset('{}_{}'.format(adj,noun), 'n')
            #As check to handle cases like sugar bowl versus metal bowl
            if len(syns) > 0:
                result = re.sub('{}\s+{}'.format(adj,noun),'{}_{}'.format(adj,noun),result)
                
        return result
        
    def create_compound_nouns(self,sentence):
        # print "Before cn process {}".format(sentence)
        isNNPredicate = True
        result = sentence
        db = []
        
        #Connect proper nouns together to one proper compound noun
        while isNNPredicate:
            isNNPredicate = False
            db = list(self.parser.parse_without_prac(result))[0]
            
            for q1 in db.query('compound(?w1,?w2)'):
                n1 = q1['?w1']
                n2 = q1['?w2']
                n1Word = '-'.join(n1.split('-')[:-1])
                n2Word = '-'.join(n2.split('-')[:-1])
                
                #Check if all nouns are proper nouns
                for _ in db.query('has_pos({},NNP)'.format(n1)):
                    for _ in db.query('has_pos({},NNP)'.format(n2)):
                        temp_result = re.sub('{}\s+{}'.format(n2Word,n1Word),'{}_{}'.format(n2Word,n1Word),result)
                        if not result == temp_result:
                            result = temp_result 
                            isNNPredicate = True
            
        #Some compound nouns will be correct recognized by the Stanford parser e.g swimming pool or sugar bowl.
        db = list(self.parser.parse_without_prac(result))[0]
        for q1 in db.query('compound(?w1,?w2)'):
            n1 = q1['?w1']
            n2 = q1['?w2']
            n1Word = '-'.join(n1.split('-')[:-1])
            n2Word = '-'.join(n2.split('-')[:-1])
            
            temp_lemma = '{}_{}'.format(n2Word,n1Word)
            syns = get_synset(temp_lemma, 'n')
            #If synset is available add to result
            if len(syns) > 0:
                result = re.sub('{}\s+{}'.format(n2Word,n1Word),temp_lemma,result)
                
        result = self.check_amod_nouns(result)
        #print "After cn process {}".format(result)
        
        return result 
                
    def process_imperative_sentence(self,sentence):
        return '"{}"'.format(sentence)       
        first_word = re.split("\s+", sentence.strip())[0]
        syns = get_synset(first_word, 'v')
        
        if len(syns) > 0:
            return sentence.replace(first_word,first_word.lower(),1)
        
        return sentence
