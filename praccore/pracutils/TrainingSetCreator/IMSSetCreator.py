import os
import re
import sys
from nltk.corpus import wordnet as wn
from prac.wordnet import WordNet

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from django.contrib.webdesign.lorem_ipsum import sentence
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
    
def get_offset(sense):
    syn = wordnet.synset(sense)
    offset = str(syn.offset())
    
    if len(offset) < 8:
        for x in range(0,8-len(offset)):
            offset = "0"+offset
    
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
    wordnet_index_sense = get_wordnet_index_sense()
    
    for file_path in file_list:
        key_file = open(os.path.join("ims_result",os.path.basename(file_path)+'.key'),"w")
        
        dbs = readDBFromFile(MLN,file_path)
        corpus = ET.Element("corpus",lang='english')
        predicate_lemma_list = {}
        dobj_lemma_list = {}
        i = 0
        
        for db in dbs:
            
            predicate = ""
            dobj = ""
            pos = ""
            predicate_sense = ""
            dobj_sense = ""
            
            i += 1
            
            for q1 in db.query('predicate(?w)'):
                predicate = wnl.lemmatize('-'.join(q1['?w'].split('-')[:-1]), 'v')
                
                for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                    predicate_sense = wordnet_index_sense[get_offset(q2['?s'])]
            
            for q1 in db.query('dobj(?w1,?w2)'):
                
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                        dobj_sense = wordnet_index_sense[get_offset(q2['?s'])]
                        
                for q2 in db.query('has_pos({},?p)'.format(q1['?w2'])):
                    pos = posMap.get(q2['?p'], None)
                    if pos is not None:
                        dobj = wnl.lemmatize('-'.join(q1['?w2'].split('-')[:-1]),pos)
                        
                    else:
                        dobj = '-'.join(q1['?w2'].split('-')[:-1])
            
            predicate_lemma = predicate + '.v'
            dobj_lemma = dobj + '.n'
            
            if pos is 'n':
                sentence = '<head>' + predicate+ '</head> the '+ dobj+'\n' 
            else:
                sentence = '<head>'+predicate+ '</head>' + dobj+'\n'
            
            if predicate_lemma not in predicate_lemma_list.keys():
                predicate_lemma_list[predicate_lemma] = [[sentence,predicate_sense]]
            else:
                predicate_lemma_list[predicate_lemma].append([sentence,predicate_sense])
            
            if pos is 'n':
                sentence = predicate+ ' the <head>'+ dobj+'</head>\n' 
            else:
                sentence = predicate+ '<head>'+ dobj+'</head>\n'
                
            if dobj_lemma not in dobj_lemma_list.keys():
                dobj_lemma_list[dobj_lemma] = [[sentence,dobj_sense]]
            else:
                dobj_lemma_list[dobj_lemma].append([sentence,dobj_sense])
        
                
        #Create xml and key file
        for x in range(0,len(predicate_lemma_list.keys())):
            lexelt = ET.SubElement(corpus, 'lexelt', item=predicate_lemma_list.keys()[x])
            
            for y in range(0,len(predicate_lemma_list[predicate_lemma_list.keys()[x]])):
                instance = predicate_lemma_list[predicate_lemma_list.keys()[x]][y]
                sentence = instance[0]
                sense = instance[1]
                id_str = predicate_lemma_list.keys()[x]+"."+str(y)
                key_file.write("{} {}\n".format(id_str,sense))
                
                instance = ET.SubElement(lexelt, 'instance', id=id_str,doscr="SEMCORE")
                context = ET.SubElement(instance, 'context')
                context.text = sentence
        
        for x in range(0,len(dobj_lemma_list.keys())):
            lexelt = ET.SubElement(corpus, 'lexelt', item=dobj_lemma_list.keys()[x])
            
            for y in range(0,len(dobj_lemma_list[dobj_lemma_list.keys()[x]])):
                instance = dobj_lemma_list[dobj_lemma_list.keys()[x]][y]
                sentence = instance[0]
                sense = instance[1]
                id_str = dobj_lemma_list.keys()[x]+"."+str(y)
                key_file.write("{} {}\n".format(id_str,sense))
                
                instance = ET.SubElement(lexelt, 'instance', id=id_str,doscr="SEMCORE")
                context = ET.SubElement(instance, 'context')
                context.text = sentence
                
        tree = ET.ElementTree(corpus)
        tree.write(os.path.join("ims_result",os.path.basename(file_path)+'.xml'))
        key_file.close()
        
        tree_file = open(os.path.join("ims_result",os.path.basename(file_path)+'.xml'),'r')
        tree_content = tree_file.read()
        tree_file.close()
        os.remove(os.path.join("ims_result",os.path.basename(file_path)+'.xml'))
        
        tree_file = open(os.path.join("ims_result",os.path.basename(file_path)+'.xml'),'w')
        tree_content = tree_content.replace("&lt;","<")   
        tree_content = tree_content.replace("&gt;",">")
        tree_file.write(tree_content)
        tree_file.close()
                    
            
        

    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        create_ukb_data_set(file_list)
        
        
    