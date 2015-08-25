import os
import re
import sys
from nltk.corpus import wordnet as wn

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from django.contrib.webdesign.lorem_ipsum import sentence
from nltk.stem import WordNetLemmatizer

wnl = WordNetLemmatizer()
MLN = None

nounTags = ['NN', 'NNS', 'NNP', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'
    

def create_ukb_data_set(file_list):
    
    for file_path in file_list:
        new_db_content = ""
        dbs = readDBFromFile(MLN,file_path)
        
        i = 0
        
        for db in dbs:
            predicate = ""
            dobj = ""
            pos = ""
            
            i += 1
            
            for q1 in db.query('predicate(?w)'):
                predicate = wnl.lemmatize('-'.join(q1['?w'].split('-')[:-1]), 'v')
                
            for q1 in db.query('dobj(?w1,?w2)'):
                
                for q2 in db.query('has_pos({},?p)'.format(q1['?w2'])):
                    pos = posMap.get(q2['?p'], None)
                    if pos is None:
                        print file_path
                        dobj = '-'.join(q1['?w2'].split('-')[:-1])
                        pos = 'n'
                    else:
                        dobj = wnl.lemmatize('-'.join(q1['?w2'].split('-')[:-1]), pos)
            
            if dobj:                                    
                new_db_content = new_db_content + "ctx_{}\n".format(str(i))
                new_db_content = new_db_content + '{}#v#w1#1 {}#{}#w2#1\n'.format(predicate,dobj,pos)
        
        new_db_file = open(os.path.join("ukb_result",os.path.basename(file_path)),"w")
        new_db_file.write(new_db_content)
        new_db_file.close()
                    
            
        

    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        create_ukb_data_set(file_list)
        
        
    