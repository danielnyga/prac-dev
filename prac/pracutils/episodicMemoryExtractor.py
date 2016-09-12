'''
Created on Aug 29, 2016

@author: seba
'''

import ontospy
import os
from prac.core.base import PRAC, PRACDatabase, PRACInference
from pracmln import MLN
from pracmln.mln.database import Database
import prac
import re
from prac.db.ies.extraction import HowtoImport
from prac.db.ies.models import Howto




PRAC_ADT_URI = "http://knowrob.org/kb/acat.owl#PracAdt"
RDF_TYPE = "http://www.w3.org/1999/02/22-rdf-syntax-ns#type"
PRAC_ADT_PROPERTY_URI = "http://knowrob.org/kb/acat.owl#" 

ACTIONCORE_URI = '{}actionCore'.format(PRAC_ADT_PROPERTY_URI)
INSTRUCTION_URI = 'http://knowrob.org/kb/knowrob.owl#taskContext'

def camel_case_to_snake_case(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def snake_case_to_camel_case(snake_str):
    if not snake_str: return ""
    
    words = snake_str.split('_')
    return words[0] + "".join(x.title() for x in words[1:])

def perform_triplet_query(queryHelper,x,y="?y",z="?z"):
    
    if not y.startswith("?"):
        y = "<{}>".format(y)
    if not z.startswith("?"):
        z = "<{}>".format(z)
    
    query_str = """CONSTRUCT {{ <{0}> {1} {2} }} WHERE {{ {{ <{0}> {1} {2} }} }}""".format(x,y,z)
    
    return list(g.queryHelper.rdfgraph.query(query_str))

def extract_actioncore(adt_instance, queryHelper):
    actioncore = ""
    query_result = perform_triplet_query(queryHelper,adt_instance,ACTIONCORE_URI)
    
    if query_result:
        #Assuming the query result contains only one triplet since one adt should contain only one actioncore
        actioncore_instance = query_result[0][2]
        query_result = perform_triplet_query(queryHelper,actioncore_instance,RDF_TYPE)
        
        #At the moment the actioncore is represented with the URI 'http://knowrob.org/kb/actioncore'
        for e in query_result:
            temp = e[2].split("/")[-1]
            if not temp.startswith("owl#"): 
                actioncore = temp
    
    return actioncore

def transform_instruction_to_frame(prac_instance,instruction):
    stopmodules = ('role_look_up', 'achieved_by', 'complex_achieved')
    infer = PRACInference(prac_instance, instruction).run(stopmodules)
    [db for step in infer.steps() for db in step.outdbs] 
    result = None
    for step in infer.steps(): 
        result = step.frame
    
    return result
def extract_instruction(adt_instance, queryHelper):
    query_result = perform_triplet_query(queryHelper, adt_instance, INSTRUCTION_URI)
    if query_result:
        return str(query_result[0][2])
        
    return ""
    
def extract_actionroles(adt_instance, queryHelper,actioncore,prac_instance):
    current_sense_map = {"LysergicAcid" : "lysergic_acid.n.01",
                         "Five" : "five.n.01",
                         "Add" : "add.v.01",
                         "Drop" : "drop.n.02",
                         "Pyrimidine" :" pyrimidine.n.01"}
    
    #PRAC uses snake case to represent the actionroles
    #However the episodic memory log file uses camel case
    actionroles = map(snake_case_to_camel_case, prac_instance.actioncores[actioncore].roles)
    extraction_result = {}
    
    for actionrole in actionroles:
        actionrole_uri = PRAC_ADT_PROPERTY_URI+actionrole 
        query_result = perform_triplet_query(queryHelper, adt_instance, actionrole_uri)
        if query_result:
            #It will be assumed that one role is only represented once in the adt
            role_owl_individual_uri = query_result[0][2]
            query_result = perform_triplet_query(queryHelper,role_owl_individual_uri,RDF_TYPE)
            #At the moment the actionrole is represented with the URI 'http://knowrob.org/kb/actionrole'
            #TODO Update this procedure if the connection to WordNet is added to the OWL files
            for e in query_result:
                temp = e[2].split("/")[-1]
                if not temp.startswith("owl#"): 
                    extraction_result[camel_case_to_snake_case(actionrole)] = current_sense_map[str(temp)]
    
    return extraction_result

def create_how_to(prac_instance,instruction_sentence,db):
    howToImport = HowtoImport(prac_instance,{})
    return Howto(prac_instance, 
                 instr=transform_instruction_to_frame(prac_instance,instruction_sentence), 
                 steps=list(howToImport.buildframes(db, 1, "")))
    
def create_database(prac_instance,actioncore, actionroles):
    mln_ac = MLN(mlnfile=os.path.join(prac.locations.home, 
                                   'pracmodules', 
                                   'ac_recognition', 
                                   'mln', 'predicates.mln'),grammar='PRACGrammar', logic='FuzzyLogic')
    
    mln_roles = MLN(mlnfile=os.path.join(prac.locations.home, 
                                   'pracmodules', 
                                   'senses_and_roles', 
                                   'mln', 'predicates.mln'),grammar='PRACGrammar', logic='FuzzyLogic')
    
    mln_ac.update_predicates(mln_roles)
    
    db = PRACDatabase(prac_instance,db=Database(mln=mln_ac))
    lemma = actionroles['action_verb'].split(".")[0]
     
    db << "action_core({}-1,{})".format(lemma,actioncore)
    db << "action_verb({}-1,{})".format(lemma,actioncore)
    db << "has_sense({}-1,{})".format(lemma,actionroles['action_verb'])
    db << "has_pos({}-1,VB)".format(lemma)
    
    i = 2
    
    for role, sense in actionroles.iteritems():
        if role == "action_verb":continue
        splitted_sense = sense.split(".")
        
        lemma = splitted_sense[0]
        pos = ""
        
        if splitted_sense[1] == "n":
            pos = "NN"
        elif splitted_sense[1] == "v":
            pos = "VB"
        else:
            pos = "JJ"
             
        word = "{}-{}".format(lemma,str(i))
        db << "{}({},{})".format(role,word,actioncore)
        db << "has_sense({},{})".format(word,sense)
        db << "has_pos({},{})".format(word,pos)
        i += 1
    
    return db

def process_prac_adt(adt_instance, queryHelper):
    prac_instance = PRAC()
    actioncore = extract_actioncore(adt_instance, queryHelper)
    actionroles = {}
    
    if actioncore:
        instruction_sentence = extract_instruction(adt_instance, queryHelper)
        actionroles = extract_actionroles(adt_instance, queryHelper, actioncore, prac_instance)
        db = create_database(prac_instance,actioncore,actionroles)
        how_to = create_how_to(prac_instance,instruction_sentence,db)
        print how_to.tojson()
        prac_instance.mongodb.prac.howtos.insert_one(how_to.tojson())
        raw_input("prompt")
        
if __name__ == '__main__':
    g = ontospy.Graph("chem_pipette_coll_2.owl")
    adt_instances = g.getClass(PRAC_ADT_URI).instances() 

    map(lambda x: process_prac_adt(x,g.queryHelper),adt_instances) 
    
    print     