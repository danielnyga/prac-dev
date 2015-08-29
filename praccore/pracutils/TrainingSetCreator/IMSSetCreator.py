import os
import re
import sys
from nltk.corpus import wordnet as wn
from prac.wordnet import WordNet

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from nltk.stem import WordNetLemmatizer

import xml.etree.cElementTree as ET


wnl = WordNetLemmatizer()
MLN = None

wordnet = WordNet(None)

nounTags = ['NN', 'NNS', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'

mapping_noun_dict = None
mapping_verb_dict = None
mapping_adj_dict = None

wordnet_index_sense = None
    

def get_wn30_to_wn171_dict(file_path):
    print 'Start process'
    mapping_file = open(file_path,'r')
    mapping_list = mapping_file.read().split('\n')
    mapping_dict = {}
    length_of_list = str(len(mapping_list))
    print length_of_list
    
    for mapping in mapping_list:
        mapping_splited = re.split("\s+",mapping)
        
        if len(mapping_splited) >= 2:
            wn30_offset = mapping_splited[0]
            wn17_offset = mapping_splited[1]
            mapping_dict[wn30_offset] = wn17_offset
    
    
    print "Finished calculating wn30_to_wn171_dict"
    mapping_file.close()
    print len(mapping_dict.keys())
    
    return mapping_dict


def get_offset(sense):
    mapping_dict = None
    syn = wordnet.synset(sense)
    pos = syn.pos()

    
    if pos == 'n':
        mapping_dict = mapping_noun_dict
    
    if pos == 'v':
        mapping_dict = mapping_verb_dict
    
    if pos == 'a':
        mapping_dict = mapping_adj_dict
    
    if mapping_dict is None:
        return None
    
    offset = str(syn.offset())
    
    if len(offset) < 8:
        for x in range(0,8-len(offset)):
            offset = "0"+offset
    
    if offset not in mapping_dict.keys():
        print "{} cannot be mapped".format(sense)
    else:
        offset = mapping_dict[offset]
    return offset

    
def get_wordnet_index_sense():
    index_file = open("index.sense",'r')
    sense_list = index_file.read().split('\n')
    pos_regex = re.compile('(.+)\s+(\d+)')
    
    wordnet_index_sense = {}
    for line in sense_list:
        res = pos_regex.search(line)
        if res:
            splited_res = re.split("\s+",res.group())
            sense_id = splited_res[0]
            offset = splited_res[1]
            wordnet_index_sense[offset] = sense_id
    
    return wordnet_index_sense
            
def create_ukb_data_set(file_list):
    
    for file_path in file_list:
        key_file = open(os.path.join("ims_result",os.path.basename(file_path)),"w")
        
        dbs = readDBFromFile(MLN,file_path)
        i = 0
        for db in dbs:
            i += 1
            predicate = ""
            dobj = ""
            pos = ""
            dobj_sense = ""
            
            
            for q1 in db.query('predicate(?w)'):
                predicate = wnl.lemmatize('-'.join(q1['?w'].split('-')[:-1]),'v')

                
            for q1 in db.query('dobj(?w1,?w2)'):
                
                for q2 in db.query('has_pos({},?p)'.format(q1['?w2'])):
                    pos = posMap.get(q2['?p'], None)
                    if pos is not None:
                        dobj = wnl.lemmatize('-'.join(q1['?w2'].split('-')[:-1]),pos)
                        
                    else:
                        dobj = '-'.join(q1['?w2'].split('-')[:-1])
                
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    dobj_sense = "TRUE"
            
            
            if dobj_sense:
                
                if pos is 'n':
                    sentence = predicate+ ' the '+ dobj+'.\n' 
                else:
                    sentence = predicate+ ' ' + dobj+'.\n'
                
                sentence = '//' + sentence + '\n'
                key_file.write(sentence)
                db.write(key_file)
                
                if i < len(dbs):
                    key_file.write('\n---\n')

        key_file.close()
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')

        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        create_ukb_data_set(file_list)
        
        
    