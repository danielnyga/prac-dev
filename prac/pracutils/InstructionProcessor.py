from os import path, listdir
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from RolequeryHandler import RolequeryHandler
import json
from pymongo import MongoClient,errors

def store_instruction_in_mongo_db(text_file_name,action_core,roles_dict,plan_list):
    json_str="""
                    {{ 
                            "_id" : "{}",
                            "action_core" : "{}",
                            "action_roles" : {},
                            "plan_list" : {}
                    }}
                    """.format(text_file_name,
                               action_core,
                               str(roles_dict),
                               str(plan_list))
    
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.PRAC
    collection = ies_mongo_db.Instructions
    
    try:
        json_dict = {"_id" : text_file_name, "action_core" : action_core, "action_roles" : roles_dict, "plan_list" : plan_list}
        collection.insert_one(json_dict)
        
    except errors.DuplicateKeyError:
        collection.delete_many({"_id" : json_dict['_id']})
        collection.insert_one(json_dict)
        
    mongo_client.close()

if __name__ == '__main__':
    args = sys.argv[1:]
    
    path_to_corpus = args[0]
    corpus = []
    
    if path.isdir(path_to_corpus):
        corpus = map(lambda x: path.abspath(path.join(path_to_corpus,x)),listdir(path_to_corpus))
    
    elif path.isfile(path_to_corpus):
        corpus.append(path.abspath(path_to_corpus))
        
    else:
        print "No valid corpus path"
    
    for text_file_path in corpus:    
        text_file = open(text_file_path, 'r')
        text_file_name = path.basename(text_file_path)
        sentences = text_file.readlines()
        
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)
            
        #Parse text file name to annotate it in the mongo db
        inference = PRACInference(prac, ["{}.".format(text_file_name)])
        while inference.next_module() != 'achieved_by' and inference.next_module() != 'plan_generation':
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)
        
        #There will be only one db
        db = inference.inference_steps[-1].output_dbs[0]
        
        roles_dict = RolequeryHandler.queryRolesAndSensesBasedOnActioncore(db)
        
        #Process all sentences
        plan_list = []
        for s in sentences:
            inference = PRACInference(prac, [s])
            while inference.next_module() != None :
                modulename = inference.next_module()
                module = prac.getModuleByName(modulename)
                prac.run(inference, module)
            step = inference.inference_steps[-1]
                
            if hasattr(step, 'executable_plans'):
                plan_list.extend(step.executable_plans)
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
                    
        store_instruction_in_mongo_db(text_file_name, actioncore, roles_dict, plan_list)