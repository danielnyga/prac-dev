import os
import re
import sys
from prac.wordnet import WordNet

class ActionCoreDbCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    ACTIONCORE_DB_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','ActioncoreDbCreator',"actioncore_dbs")
    wordnet = WordNet(concepts=None)
    SYNSET_KEY = "SYNSET_KEY"
    def init_synset_key_list(self):
        
        synset_key_list = []
        
        for filename in os.listdir(self.ACTIONCORE_DB_PATH):
            if filename.endswith("~"): continue
            file = open(os.path.join(self.ACTIONCORE_DB_PATH,filename),'r')
            content = file.read()
            regex_result = re.search('\s*//'+self.SYNSET_KEY+'\s*:\s*(\w+)',content)
            
            if regex_result:
                synset_key_list.append(self.wordnet.synsets(str(regex_result.group(1)),"v"))
            
            file.close()
        return synset_key_list
    
    
    def create_actioncore_dbs(self,input_dir):
        #verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
        #Ignore modal verbs
        verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
        synset_key_list = ac.init_synset_key_list()
        regex_has_pos = re.compile('has_pos\s*\(\s*(\w+)(-{0,1}\d*)\s*,\s*(\w+)\s*\)')
        regex_new_db = re.compile("\n\s*-+\s*\n")
         
        for filename in os.listdir(input_dir):
            if filename.endswith("~"): continue
            
            file = open(os.path.join(input_dir,filename),'r')
            dbs = regex_new_db.split(file.read())
            
            for db in dbs:
                sentence = db.strip().split("\n")[0]
                for (word,id,tag) in regex_has_pos.findall(db):
                    if tag in verb_tags:
                        synset = self.wordnet.synsets(word,"v")
                        
                        if synset and not self.is_aux_verb(word+id,db):
                            is_synset_added = False
                            for i in range(0,len(synset_key_list)):
                                synset_key = synset_key_list[i]
                                
                                if self.are_synsets_equal(synset,synset_key):
                                    pas_db = self.create_pas_db(db, word+id)
                                    
                                    if pas_db:
                                        file = open(os.path.join(self.ACTIONCORE_DB_PATH,str(i+1)+".db"),'a')
                                        file.write('\n---\n')
                                        file.write(self.SYNSET_KEY+":"+word+"\n")
                                        file.write(sentence+'\n\n')
                                        file.write(pas_db)
                                        file.close()
                                    is_synset_added = True
                                    break
                            
                            if not is_synset_added:
                                pas_db = self.create_pas_db(db, word+id)
                                if pas_db:
                                    file = open(os.path.join(self.ACTIONCORE_DB_PATH,str(len(synset_key_list)+1)+".db"),'a')
                                    file.write(self.SYNSET_KEY+":"+word+"\n")
                                    file.write(sentence+'\n\n')
                                    file.write(pas_db)
                                    file.close()
                                    synset_key_list.append(synset)
                                
                                
    def are_synsets_equal(self,syn1,syn2):
        
        if len(syn1) != len(syn2):
            return False
        
        for i in range(0,len(syn1)):
            if not syn1[i] == syn2[i]:
                return False
            
        return True
    
    def is_aux_verb(self,word,db):
        regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        if regex_aux.search(db):
            return True;
        
        if regex_auxpass.search(db):
            return True;
        
        return False;
    
    def create_pas_db(self,db,predicate):
        regex_dobj = re.compile('dobj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        regex_nsubj = re.compile('nsubj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        regex_iobj = re.compile('iobj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        regex_pobj = re.compile('prep\w+\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        result = ""
        #TODO add has_sense
        
        for e in regex_dobj.findall(db):
            result += '{} {}\n'.format(str(1.00),str(e))
        
        for e in regex_nsubj.findall(db):
            result += '{} {}\n'.format(str(1.00),str(e))
            
        for e in regex_iobj.findall(db):
            result += '{} {}\n'.format(str(1.00),str(e))
            
        for e in regex_pobj.findall(db):
            result += '{} {}\n'.format(str(1.00),str(e))
     
        return result
    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if args:
        input_dir = args[0] 
        ac = ActionCoreDbCreator()
        ac.create_actioncore_dbs(input_dir)
            
            
    
    