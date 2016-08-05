import StringIO
from collections import defaultdict
from os import path, listdir

import sys
from pymongo import MongoClient,errors

from prac.core.base import PRAC
from prac.core.inference import PRACInference
from pracmln.mln.util import headline, out


def store_instruction_in_mongo_db(prac, text_file_name, action_core, roles_dict, plan_list):
    
    mongo_client = MongoClient()
    ies_mongo_db = mongo_client.prac
    collection = ies_mongo_db.prac
    
    plan_list_json = []
    #Transform
    
    for key in plan_list.keys():
        for plan in plan_list[key]:
            db_str = StringIO.StringIO()
            mln_str = StringIO.StringIO()
            
            plan.write(db_str,None,False)
            plan.mln.write(mln_str,None)

            # retrieve action roles
            action_roles = {(k, v) for (k, v) in db.roles(actioncore)}

            plan_action_core = ""
            
            for q in plan.query("action_core(?w,?ac)"):
                plan_action_core = q["?ac"]
            
            for q in plan.query("achieved_by(?w,?ac)"):
                plan_action_core = q["?ac"]
        
            temp_dict = {'Sentence' : key,
                         'MLN': mln_str.getvalue(),
                         'DB' : db_str.getvalue(), 
                         'action_core' : plan_action_core,
                         'action_roles': action_roles}
            
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


def extract_syntax(pracinfer):
    #=======================================================================
    # compose the syntactic evidences
    #=======================================================================
    syntax_db = pracinfer.inference_steps[0].output_dbs[0]
    d = defaultdict(list)
    for atom in syntax_db.evidence.keys():
        _, rel, args = syntax_db.mln.logic.parse_literal(atom)
        d[rel].append(args)
    return d

def analyze_howto(prac, howto, steps=None, verbose=1):
    '''
    Analyzes a howto and returns a dictionary holding the action cores.
    '''
    #===========================================================================
    # print a little summary of the instructions handed over
    #===========================================================================
    if verbose:
        print headline('How to {}:'.format(howto))
        if steps:
            for i, inst in enumerate(steps):
                print '{:.2f}.'.format(i+1), inst
    #===========================================================================
    # Parse the howto title and analyze it
    #===========================================================================
    pipeline = ('nl_parsing', 'ac_recognition', 'senses_and_roles', 'coref_resolution', 'prop_extraction')
    
    inference = PRACInference(prac, ["{}.".format(howto)])
    while inference.next_module() in pipeline:
        modulename = inference.next_module()
        module = prac.module(modulename)
        prac.run(inference, module)
    #===========================================================================
    # There will be only one DB, if there are more, raise an exception
    #===========================================================================
    if len(inference.inference_steps[-1].output_dbs) != 1: 
        raise Exception('title of the howto comprises more than one action: {}'.format(howto))
    db = inference.inference_steps[-1].output_dbs[0]
    howto_syntax = extract_syntax(inference)

    roles_dict = {}
    for ac in db.actioncores():
        actioncore = ac.values().pop()
        roles_dict.update({k: v for r in db.roles(actioncore) for k, v in r.items()})

    actioncore = ""
    #===========================================================================
    # It will be assumed that there is only one true action_core predicate per database 
    #===========================================================================
    for q in db.query("action_core(?w,?ac)"):
        actioncore = q["?ac"]
    #===========================================================================
    # Process all sentences
    #===========================================================================
    plan_list = []
    inference = PRACInference(prac, steps)
    while inference.next_module() in pipeline:
        modulename = inference.next_module()
        module = prac.module(modulename)
        prac.run(inference, module)
    step_dbs = inference.inference_steps[-1].output_dbs
    #===========================================================================
    # Build the data structures from the analysis result
    #===========================================================================
    for db in step_dbs:
        action_roles = {k: v for r in db.roles(actioncore) for k, v in r.items()}

        plan_action_core = ""
        for q in db.query("action_core(?w,?ac)"):
            plan_action_core = q["?ac"]
        for q in db.query("achieved_by(?w,?ac)"):
            plan_action_core = q["?ac"]
        if not plan_action_core:
            plan_action_core = 'UNKNOWN'
        temp_dict = {#'sentence' : step,
                     'syntax': extract_syntax(inference),
                     'actioncore' : plan_action_core,
                     'actionroles': action_roles}
        
        plan_list.append(temp_dict)
    result = {'howto': howto, 'actioncore': actioncore, 'actionroles' : roles_dict, 'steps' : plan_list, 'syntax': howto_syntax} 
    return result
    

def store_howto(prac, howto):
    '''
    Stores an analyzed howto in the MongoDB.
    '''
    mongodb = MongoClient(host=prac.config.get('mongodb', 'host'), port=prac.config.getint('mongodb', 'port'))
    mongodb.prac.howtos.insert_one(howto)


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
        
        modules = ('nl_parsing', 'ac_recognition', 'senses_and_roles', 'coref_resolution', 'prop_extraction')
        
        prac = PRAC()
        #Parse text file name to annotate it in the mongo db
        inference = PRACInference(prac, ["{}.".format(text_file_name)])
        while inference.next_module() in modules:
            modulename = inference.next_module()
            module = prac.module(modulename)
            prac.run(inference, module)
        exit(0)
        #There will be only one db
        db = inference.inference_steps[-1].output_dbs[0]

        #Process all sentences
        plan_list = {}
        for s in sentences:
            inference = PRACInference(prac, [s])
            while inference.next_module() != 'plan_generation' :
                modulename = inference.next_module()
                module = prac.module(modulename)
                prac.run(inference, module)
            plan_list[s]=inference.inference_steps[-1].output_dbs

        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]

        action_roles = {(k, v) for (k, v) in db.roles(actioncore)}

        store_instruction_in_mongo_db(prac, text_file_name, actioncore, roles_dict, plan_list)