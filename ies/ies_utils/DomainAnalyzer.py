'''
Created on Nov 3, 2015

@author: seba
'''
from __future__ import division
from nltk.corpus import wordnet as wn
from mln.database import readDBFromFile
from mln.mln import readMLNFromFile,Predicate
from prac.wordnet import WordNet
from pracutils.pracgraphviz import render_gv
from utils.graphml import to_html_hex
from os.path import basename
from ies_models import Constants

import sys  
import os

'''
Represents a hypernym.
'''

class ClassStatistics(object):
    
    def __init__(self,syn):
        self.syn = syn
        self.name = str(syn.name)
        self.factor = 0
        #Number of occurrence  
        self.counter = 1
        #Set containing all words which activated the hypernym
        self.word_set = set()
        self.word_total = 0
        #Contains all hypernym distances.
        self.distance_list = []
        
    def get_weight(self,highest_factor):
        #return len(self.word_set)/self.word_total*self.factor
        if self.get_factor() > highest_factor/2:
            return 0
        
        return (1 - self.get_factor()/highest_factor)*(len(self.word_set)/self.word_total)
    
    def get_factor(self):
        return (sum(self.distance_list)/len(self.distance_list))
    
#Need only to be run once.
#rock hind has depth length of 19
def get_max_depth_of_wordnet_taxonomy():
    max_depth = 0
    syn_name = ""
     
    for syn in wn.all_synsets():
        if syn.max_depth() > max_depth:
            max_depth = syn.max_depth()
            syn_name = syn.name
    
    print syn_name
    return max_depth

'''
1.Get synsets for d/prep obj
2.For every synset get all hypernym and their corresponding distances.
3.Create/Update for every hypernym a ClassStatistics
'''

def analyse_db_based_on_mln_predicate(db,mln_predicate,word_parameter,class_statistics_dict):
    correct_word = ""
    for q in db.query(mln_predicate):
        word = '-'.join(q[word_parameter].split('-')[:-1])
        #Assuming every dobj or prepobj has to be a noun.
        synsets = wn.synsets(word,'n')
        
        if synsets:
            correct_word = word
        for synset in synsets:
            for hypernym_tupel in synset.hypernym_distances():
                hypernym_synset =  hypernym_tupel[0]
                
                if hypernym_synset == synset: continue
                
                hypernym_distance = hypernym_tupel[1]
                if hypernym_synset.name in class_statistics_dict.keys():
                    class_statistics_dict[hypernym_synset.name].counter += 1
                else:
                    class_statistics_dict[hypernym_synset.name] = ClassStatistics(hypernym_synset)
                    
                class_statistics_dict[hypernym_synset.name].word_set.add(word)
                class_statistics_dict[hypernym_synset.name].distance_list.append(hypernym_distance)
                    
        
    return [class_statistics_dict,correct_word]

def print_class_statistics(class_statistics_dict,graph_file_name):
    highest_factor = max(c.get_factor() for c in class_statistics_dict.values())
    
    class_statistics_list = class_statistics_dict.values()
    class_statistics_list.sort(key=lambda x: x.get_weight(highest_factor), reverse=False)
    known_concepts = []
    
    for class_statistic in class_statistics_list[-10:]:
        known_concepts.append(class_statistic.name)
    
    class_statistic_file = open("{}.txt".format(graph_file_name),'w+')
    
    for class_statistics in class_statistics_list[-10:]:
        class_statistic_file.write("NAME: {}, PERCENTAGE OF ALL WORDS: {}, WEIGHT: {}, FACTOR: {}, #WORDS: {}\n".format(class_statistics.name,len(class_statistics.word_set)/class_statistics.word_total, class_statistics.get_weight(highest_factor),class_statistics.get_factor(),len(class_statistics.word_set)))
        class_statistic_file.write(",".join(class_statistics.word_set))
        class_statistic_file.write("\n\n")
        print
    
    wordnet =  WordNet(known_concepts)
    dot_graph = wordnet.to_dot()
    
    for concept in known_concepts:
        if concept in wordnet.known_concepts:
            dot_graph.node(concept,fillcolor=get_prob_color(class_statistics_dict[concept].get_weight(highest_factor)))
    
    render_gv(dot_graph,"{}.svg".format(graph_file_name))

def get_prob_color(p):
    return to_html_hex(.3, p * .75, 1., 'hsv')

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) == 2:
        mln_path = args[0]
        db_path = args[1]
        
        mln = readMLNFromFile(mln_path)
        predicate_list = [Constants.SLOT_VALUE_DOBJ]
        is_db_loaded = False
        dbs = []
        
        '''
        It is possible that the dbs contain predicates which are not defined in the start MLN.
        To avoid that the program stops, missing predicates has to be defined during the reading process.
        
        Every predicate representing a prep_obj will be added to the predicate list.
        The predicate list contains predicates which will be analyzed. 
        '''
        
        while not is_db_loaded:
            try:
                dbs = readDBFromFile(mln,db_path)
                is_db_loaded = True
            except KeyError:
                _, exc_value , _ = sys.exc_info()
                print exc_value
                predicate_name = str(exc_value).replace("'", "")
                mln.declarePredicate(Predicate(predicate_name,['word','word']))
                if predicate_name.startswith("prep_"):
                    predicate_list.append(predicate_name)
            except Exception:
                print sys.exc_info()
                is_db_loaded = True
        
        result_dir = "{}_result".format(basename(db_path))
        
        if not os.path.exists(result_dir):
            os.makedirs(result_dir)
            
        for query_predicate in predicate_list:
            result_list = [{}]
            total_word_set = set()
            for db in dbs: 
                result_list = analyse_db_based_on_mln_predicate(db, "{}(?w1,?w2)".format(query_predicate), "?w2", result_list[0])
                if result_list[1]:
                    total_word_set.add(result_list[1])
          
            for values in result_list[0].values():
                values.word_total = len(total_word_set)
            
            if result_list[0]: 
                print_class_statistics(result_list[0],os.path.join(result_dir,"{}_{}".format(basename(db_path),query_predicate)))
        
        
        
    
        
