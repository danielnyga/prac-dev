'''
Created on Sep 2, 2015

@author: Sebastian Koralewski (seba@informatik.uni-bremen.de)
'''
from multiprocessing import Pool, cpu_count
import json

from pymongo import MongoClient
import pymongo
import sys
from ies_utils import MultiprocessingMethods, MongoDatabaseHandler,\
    PracDatabaseHandler
from ies_controller.FrameExtractor import FrameExtractor
from os import path,listdir,fstat
from ies_models.FrameExtractorResult import FrameExtractorResult

def run_frame_extraction_process(corpus):
    frame_extractor = FrameExtractor(corpus)
    return frame_extractor.extract_frames()

def extract_frames_of_corpus(corpus,use_multicore=False):
    corpus_ = []
    
    if path.isdir(corpus):
        for filename in listdir(corpus):
            corpus_.append(path.abspath(path.join(corpus,filename)))
    
    elif path.isfile(corpus):
        corpus_.append(path.abspath(corpus))
    
    else:
        corpus_ = corpus
    
    if use_multicore:
        result = FrameExtractorResult()
        corpus_sub_lists = MultiprocessingMethods.split_list_in_n_equal_sub_lists(corpus_, cpu_count())
        workerPool = Pool(processes=cpu_count())
        temp_result_list = workerPool.map_async(run_frame_extraction_process, corpus_sub_lists).get()
        workerPool.close()
        workerPool.join()
        
        for temp_result in temp_result_list:
            #TODO update to frame_extractor_result
            
            result.merge_frame_extractor_result(temp_result)
        
        return result
    else:
        return run_frame_extraction_process(corpus_)

def store_frames_as_files(frames,path_to_file,file_base_name):
    store_files(frames,path_to_file,file_base_name)
    
def store_logs_as_files(logs,path_to_file,file_base_name):
    store_files(logs,path_to_file,file_base_name)


def store_files(objects,save_dir,file_base_name):
    #Mongo DB can only import json files to max 16mb    
    file_size_limit = 16*1000*1000    
    num_files = 1
    frame_file = open(path.join(save_dir,"{}_{}.json".format(file_base_name,str(num_files))),'w')
    
    
    frame_file.write("[")
    
    for i in range(0,len(objects)):
        frame_str = objects[i].to_json_str()
        current_file_size = fstat(frame_file.fileno()).st_size
        
        if (current_file_size + ((len(frame_str)+1)*8)) < file_size_limit:            
            frame_file.write(frame_str+"\n")
            
            if i < (len(objects)-1):
                frame_file.write(',')
            frame_file.flush()
        else:
            #If one single frame or log stat will be more than 16MB large, the user has to handle it by himself.
            #Last char will be "," or "\n". "," has to be replaced to avoid a wrong formatted json obj.
            #Close old file
            frame_file.seek(-1,2)
            frame_file.write("]")
            frame_file.close()
            
            #Create a new file            
            num_files += 1            
            frame_file = open(path.join(save_dir,"{}_{}.json".format(file_base_name,str(num_files))),'w')
            frame_file.write("[")
            frame_file.write(frame_str+"\n")
            
            if i < (len(objects)-1):
                frame_file.write(',')
            frame_file.flush()
    
    frame_file.write("]")
    frame_file.close()

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
            log_file.write("###########")
            log_file.write(data.to_json_str())
            log_file.write("###########")
    
if __name__ == '__main__':
    #FRAME EXTRACTION PROCESS
    result = extract_frames_of_corpus('chemical_corpus_v2',False) 
    #logs = result.process_text_file_result_list
    #frames = result.frame_list

    #store_frames_into_database(frames)
    #store_logs_into_database(logs)
    
    '''
    ac_list = ['bring'
               ,'barbecue'
               ,'bake'
               ,'clean'
               ,'combine'
               ,'mix'
               ,'flip'
               ,'grab'
               ,'pull'
               ,'remove'
               ,'sprinkle'
               ,'store'
               ,'extract'
               ,'cook']
        
    
    ac_list = ['flip','store']
    
    for ac in ac_list:
        #PracDatabaseHandler.generate_ac_train_optimazed_prac_db({'slot_values.predicate.lemma' : '{0}'.format(ac)}, "prac_db/{}".format(ac))
        PracDatabaseHandler.generate_prac_db_based_on_query({'slot_values.predicate.lemma' : '{0}'.format(ac)}, "prac_db/{}.db".format(ac))
    #print MongoDatabaseHandler.get_frames_based_on_query({'action_core' :'Filling'})[0].slot_values.values()[4].misc
    '''
    print 'FINISHED'
