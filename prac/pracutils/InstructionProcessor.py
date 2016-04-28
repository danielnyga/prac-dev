from os import path, listdir
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from RolequeryHandler import RolequeryHandler
import json
from pymongo import MongoClient,errors
import StringIO

def store_instruction_in_mongo_db(text_file_name,action_core,roles_dict,plan_list):
    
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.PRAC
    collection = ies_mongo_db.Instructions
    
    
    plan_list_json = []
    #Transform
    
    for key in plan_list.keys():
        for plan in plan_list[key]:
            db_str = StringIO.StringIO()
            mln_str = StringIO.StringIO()
            
            plan.write(db_str,None,False)
            plan.mln.write(mln_str,None)
            action_roles = RolequeryHandler.query_roles_and_senses_based_on_achieved_by(plan)
            
            if not action_roles:
                action_roles = RolequeryHandler.query_roles_and_senses_based_on_action_core(plan)
            temp_dict = {'Sentence' : key,'MLN': mln_str.getvalue(),'DB' : db_str.getvalue(),'action_roles': action_roles}
            plan_list_json.append(temp_dict)
            
            db_str.close()
            mln_str.close()
            
    try:
        json_dict = {"_id" : text_file_name, "action_core" : action_core, "action_roles" : roles_dict, "plan_list" : plan_list_json}
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
        while inference.next_module() != 'role_look_up' and inference.next_module() != 'achieved_by' and inference.next_module() != 'plan_generation':
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)
        
        #There will be only one db
        db = inference.inference_steps[-1].output_dbs[0]
        roles_dict = RolequeryHandler.query_roles_and_senses_based_on_action_core(db)
        
        #Process all sentences
        plan_list = {}
        for s in sentences:
            inference = PRACInference(prac, [s])
            while inference.next_module() != 'plan_generation' :
                modulename = inference.next_module()
                module = prac.getModuleByName(modulename)
                prac.run(inference, module)
            plan_list[s]=inference.inference_steps[-1].output_dbs
            
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
                    
        store_instruction_in_mongo_db(text_file_name, actioncore, roles_dict, plan_list)