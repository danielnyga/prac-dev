import os
import re
import sys
from prac.wordnet import WordNet

class ActionCoreDbCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    ACTIONCORE_DB_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','ActioncoreDbCreator',"actioncore_dbs")
    wordnet = WordNet(concepts=None)
    
    def init_synset_key_list(self):
        
        synset_key_list = []
        
        for filename in os.listdir(self.ACTIONCORE_DB_PATH):
            if filename.endswith("~"): continue
            file = open(os.path.join(self.ACTIONCORE_DB_PATH,filename),'r')
            content = file.read()
            regex_result = re.search('\s*//SYNSET_KEY\s*:\s*(\w+)',content)
            
            if regex_result:
                synset_key_list.append(self.wordnet.synsets(str(regex_result.group(1)),"v"))
        
        return synset_key_list
    
    
    def create_actioncore_dbs(self,input_dir):
        #verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
        #Ignore modal verbs
        verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
        synset_key_list = ac.init_synset_key_list()
        regex_has_pos = re.compile('has_pos\s*\(\s*(\w+)-{0,1}\d*\s*,\s*(\w+)\s*\)')
        regex_new_db = re.compile("\n\s*-+\s*\n")
         
        for filename in os.listdir(input_dir):
            if filename.endswith("~"): continue
            
            file = open(os.path.join(input_dir,filename),'r')
            dbs = regex_new_db.split(file.read())
            
            for db in dbs:
                for (word,tag) in regex_has_pos.findall(db):
                    if tag in verb_tags:
                        synset = self.wordnet.synsets(word,"v")

                        if synset:
                            print word
                            print synset 
                            print "---"

if __name__ == '__main__':
    args = sys.argv[1:]
    
    if args:
        input_dir = args[0] 
        ac = ActionCoreDbCreator()
        ac.create_actioncore_dbs(input_dir)
            
            
    
    