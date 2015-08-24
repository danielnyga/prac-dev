import os
import re
import sys
from nltk.corpus import wordnet as wn

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from django.contrib.webdesign.lorem_ipsum import sentence

MLN = None
regex_new_db = re.compile("\n\s*-+\s*\n")
regex_has_pos = re.compile('has_pos\s*\(\s*\w+-{0,1}\d*\s*,\s*\w+\s*\)')
regex_has_sense = re.compile('has_sense\s*\(\s*\w+-{0,1}\d*\s*,\s*[A-Za-z0-9._]+\s*\)')

def trim(str):
    return str.replace(" ", "")
def mapSentencesToXvalDbs(brown_dbs,xval_dbs):
    
    for xval_db in xval_dbs:
        new_db_content = ""
        file = open(xval_db,'r')
        
        dbs_as_textfiles = regex_new_db.split(file.read())
        
        for db in dbs_as_textfiles:
            has_sense_list = map(trim,regex_has_sense.findall(db))
            has_pos_list = map(trim,regex_has_pos.findall(db))
            
            for key, brown_lists in brown_dbs.iteritems():
                has_senses_brown_list = brown_lists[0]
                has_pos_brown_list = brown_lists[1]
                
                if(set(has_sense_list).issubset(set(has_senses_brown_list)) and set(has_pos_list).issubset(set(has_pos_brown_list))) :
                    if new_db_content :
                        new_db_content = new_db_content + "\n---\n"
                        
                    new_db_content = new_db_content + key+"\n"
                    new_db_content =  new_db_content + db
                    break
            
        file.close()    
        new_db_file = open(os.path.join("lesk_result",os.path.basename(xval_db)),"w")
        new_db_file.write(new_db_content)
        new_db_file.close()
        file.close()
        
def getBrownCorpusList(file_list_brown):
    brown_corpus_list = {}
    
    for file in file_list_brown:
        file = open(file,'r')
        dbs_as_textfiles = regex_new_db.split(file.read())
        
        for db in dbs_as_textfiles:
            sentence = db.strip().split("\n")[0]
            
            has_sense_key_list = map(trim,regex_has_sense.findall(db))
            has_pos_key_list = map(trim,regex_has_pos.findall(db))
            
            brown_corpus_list[sentence] = [has_sense_key_list,has_pos_key_list]
    
    return brown_corpus_list
    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir_brown = args[0]
        input_dir_xval = args[1]
        file_list_brown = []
        file_list_xval = []
        
        for filename in os.listdir(input_dir_brown):
            file_list_brown.append(os.path.join(input_dir_brown,filename))
        
        for filename in os.listdir(input_dir_xval):
            file_list_xval.append(os.path.join(input_dir_xval,filename))
            
        mapSentencesToXvalDbs(getBrownCorpusList(file_list_brown), file_list_xval)
        
        
    