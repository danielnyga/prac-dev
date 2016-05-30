'''
Created on Oct 28, 2015

@author: seba
'''
from ies_models import Constants
from mln.mln import readMLNFromString
from mln.database import readDBFromString
from ies_models.Sense import Sense
from ies_models.Frame import Frame
from pymongo import MongoClient

def get_frames_based_on_query(query):
    frame_list = []
    
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.IES
    frames_collection = ies_mongo_db.Frames
    
    cursor = frames_collection.find(query)
    num_found_results = cursor.count()    
    i = 0
    for document in cursor:
        i += 1
        print " Converting {} of {}".format(int(i),int(num_found_results))
        frame = document_to_frame(document)
        if frame.is_valid():
            frame_list.append(frame)  
          
    mongo_client.close()
    
    return frame_list

def get_all_stored_frames():
    frame_list = []
    
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.IES
    frames_collection = ies_mongo_db.Frames
    
    cursor = frames_collection.find()

    for document in cursor:
        frame_list.append(document_to_frame(document))    
    mongo_client.close()
    
    return frame_list
def document_to_frame(document):
    
    sentence = str(document[Constants.JSON_FRAME_SENTENCE])
    #prac_mln = readMLNFromString(str(document[Constants.JSON_FRAME_PRAC_MLN]))
    #prac_db = readDBFromString(prac_mln,str(document[Constants.JSON_FRAME_PRAC_DB]))[0]
    prac_mln = ""
    prac_db = ""
    text_source_file, sentence_number, frame_id = document[Constants.JSON_FRAME_ID].split("#")
    slot_values = document_slot_values_to_senses(document[Constants.JSON_FRAME_SLOT_VALUES])
    actioncore = str(document[Constants.JSON_FRAME_ACTIONCORE])
    actioncore_roles = document_action_core_roles_to_senses(document[Constants.JSON_FRAME_ACTIONCORE_ROLES])
    sentence_number = int(sentence_number)
    frame_id = int(frame_id)
    
    return Frame(str(text_source_file),
                sentence_number,
                frame_id,
                sentence,
                prac_mln,
                prac_db,
                slot_values,
                actioncore,
                actioncore_roles,
                process_frame=False)

def document_action_core_roles_to_senses(document_action_core_roles):
    action_core_roles = {}
    
    for key,value in document_action_core_roles.iteritems():
        action_core_roles[str(key)] = Sense(str(value[Constants.JSON_SENSE_WORD]), 
                                     str(value[Constants.JSON_SENSE_PENN_TREEBANK_POS]), 
                                     str(value[Constants.JSON_SENSE_NLTK_WORDNET_SENSE]),
                                     str(value[Constants.JSON_SENSE_WORDNET_POS]),
                                     str(value[Constants.JSON_SENSE_LEMMA]),
                                     str(value[Constants.JSON_SENSE_MISC]),
                                     False)
             
    return action_core_roles
    
def document_slot_values_to_senses(document_slot_values):
    slot_values = {}
    
    for key,value in document_slot_values.iteritems():
        if value:
            slot_values[str(key)] = Sense(str(value[Constants.JSON_SENSE_WORD]), 
                                     str(value[Constants.JSON_SENSE_PENN_TREEBANK_POS]), 
                                     str(value[Constants.JSON_SENSE_NLTK_WORDNET_SENSE]),
                                     str(value[Constants.JSON_SENSE_WORDNET_POS]),
                                     str(value[Constants.JSON_SENSE_LEMMA]),
                                     str(value[Constants.JSON_SENSE_MISC]),
                                     False)
             
        else:
            slot_values[str(key)] = None
    return slot_values
