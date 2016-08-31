'''
Created on Aug 29, 2016

@author: seba
'''

import ontospy


PRAC_ADT_URI = "http://knowrob.org/kb/acat.owl#PracAdt"
RDF_TYPE = "http://www.w3.org/1999/02/22-rdf-syntax-ns#type"
ACTIONCORE_URI = 'http://knowrob.org/kb/acat.owl#actionCore'

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
            print e
            temp = e[2].split("/")[-1]
            if not temp.startswith("owl#"): 
                actioncore = temp
    
    return actioncore

def process_prac_adt(adt_instance, queryHelper):
    ##EXTRACT ACTION CORE
    actioncore = extract_actioncore(adt_instance, queryHelper)
    return ""

if __name__ == '__main__':
    g = ontospy.Graph("chem_pipette_coll_2.owl")
    adt_instances = g.getClass(PRAC_ADT_URI).instances() 

    result = map(lambda x: process_prac_adt(x,g.queryHelper),adt_instances) 
    
    print result    