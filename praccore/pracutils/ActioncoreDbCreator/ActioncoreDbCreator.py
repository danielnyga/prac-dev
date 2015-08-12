import os
import re
import sys
from prac.wordnet import WordNet

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile

class ActionCoreDbCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    ACTIONCORE_DB_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','ActioncoreDbCreator',"actioncore_dbs")
    wordnet = WordNet(concepts=None)
    SYNSET_KEY = "SYNSET_KEY"
    pobj_type_list = []
    
    #TODO Update to prac db handling
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
    
    
    def create_actioncore_dbs(self,input_dir,mln):
        #verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
        #Ignore modal verbs
        verb_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
        synset_key_list = ac.init_synset_key_list()
        regex_has_pos = re.compile('has_pos\s*\(\s*(\w+)(-{0,1}\d*)\s*,\s*(\w+)\s*\)')
        regex_new_db = re.compile("\n\s*-+\s*\n")
        regex_pobj = re.compile('1.00\s*(prep_\w+)\s*')
        
         
        for filename in os.listdir(input_dir):
            if filename.endswith("~"): continue
            
            try:
                print filename
                
                file = open(os.path.join(input_dir,filename),'r')
                dbs_as_textfiles = regex_new_db.split(file.read())
                dbs = readDBFromFile(mln,os.path.join(input_dir,filename))
                                     
                for i in range(0,len(dbs)):
                    self.pobj_type_list = []
                    db = dbs[i]
                    db_as_textfile = dbs_as_textfiles[i]
                    
                    for line in db_as_textfile.splitlines():
                        regex_pobj_result = regex_pobj.search(line)
                        if regex_pobj_result:
                            self.pobj_type_list.append(str(regex_pobj_result.group(1)).strip())
                    
                    sentence = " "
                    sense_list = []
                    #TODO use index of dbs to get correct sentence
                    #sentence = db.strip().split("\n")[0]
                    
                    for q in db.query('has_pos(?w, ?p)'):
                        if q['?p'] in verb_tags:
                            word = '-'.join(q['?w'].split('-')[:-1])# extract everything except the number (e.g. compound words like heart-shaped from heart-shaped-4)
                            synset = self.wordnet.synsets(word,"v")
                            
                            if synset and not self.is_aux_verb(q['?w'],db):
                                pas_db = self.create_pas_db(db, q['?w'],sense_list)
                                is_synset_added = False
                                
                                if not pas_db.isEmpty():
                                    for j in range(0,len(synset_key_list)):
                                        synset_key = synset_key_list[j]
                                        
                                        if self.are_synsets_equal(synset,synset_key):
                                            path_to_dbs = os.path.join(self.ACTIONCORE_DB_PATH,str(j+1)+".db")
                                            synset_dbs = readDBFromFile(mln,path_to_dbs)
                                            dbs_file = open(path_to_dbs,'w')
                                            synset_dbs.append(pas_db)
                                            Database.writeDBs(synset_dbs,dbs_file)
                                            is_synset_added = True
                                            dbs_file.close()
                                            break
                                        
                                    if not is_synset_added:
                                        pas_db.writeToFile(os.path.join(self.ACTIONCORE_DB_PATH,str(len(synset_key_list)+1)+".db"))
                                        synset_key_list.append(synset)
                file.close()
                os.remove(os.path.join(input_dir,filename))
            except:
                print "Parsing Error"
                                        
                                
    def are_synsets_equal(self,syn1,syn2):
        
        if len(syn1) != len(syn2):
            return False
        
        for i in range(0,len(syn1)):
            if not syn1[i] == syn2[i]:
                return False
            
        return True
    
    def is_aux_verb(self,word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for q in db.query('aux(?w, {})'.format(word)):
            return True;
        
        for q in db.query('auxpass(?w, {})'.format(word)):
            return True;
        
        return False;
    
    def is_pronoun(self,word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for q in db.query('has_pos({}, PRP)'.format(word)):
            return True;
        
        for q in db.query('has_pos({}, PRP$)'.format(word)):
            return True;
        
        return False;
    
    def is_wh(self,word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for q in db.query('has_pos({}, WDT)'.format(word)):
            return True;
        
        for q in db.query('has_pos({}, WP)'.format(word)):
            return True;
        
        return False;
    
    def create_pas_db(self,db,predicate,sense_list):
        regex_dobj = re.compile('dobj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        regex_nsubj = re.compile('nsubj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        regex_iobj = re.compile('iobj\s*\(\s*'+predicate+'\s*,\s*\w+-{0,1}\d*\s*\)')
        
        result = Database(db.mln)
        
        is_obj_added = False
        for obj_type in ['dobj','nsubj','iobj']:
            for q in db.query('{}({}, ?w)'.format(obj_type,predicate)):
                if not self.is_pronoun(q['?w'], db) and not self.is_wh(q['?w'], db):
                    result.addGroundAtom('{}({}, {})'.format(obj_type,predicate,q['?w']))
                    result = self.add_senses_and_concept(q['?w'], db, result, sense_list)
                    is_obj_added = True
        
        
        for q in db.query('agent({}, ?w)'.format(predicate)):
                if not self.is_pronoun(q['?w'], db) and not self.is_wh(q['?w'], db):
                    result.addGroundAtom('nsubj({}, {})'.format(predicate,q['?w']))
                    result = self.add_senses_and_concept(q['?w'], db, result, sense_list)
                    is_obj_added = True
        
        for q in db.query('nsubjpass({}, ?w)'.format(predicate)):
                if not self.is_pronoun(q['?w'], db) and not self.is_wh(q['?w'], db):
                    result.addGroundAtom('dobj({}, {})'.format(predicate,q['?w']))
                    result = self.add_senses_and_concept(q['?w'], db, result, sense_list)
                    is_obj_added = True
        
        for pobj_type in self.pobj_type_list:
            for q in db.query('{}({}, ?w)'.format(pobj_type,predicate)):
                if not self.is_pronoun(q['?w'], db) and not self.is_wh(q['?w'], db):
                    result.addGroundAtom('prepobj({}, {})'.format(predicate,q['?w']))
                    result = self.add_senses_and_concept(q['?w'], db, result, sense_list)
                    is_obj_added = True
                
        if not is_obj_added:
            return result
        
        result.addGroundAtom("predicate({})".format(predicate))
        result = self.add_senses_and_concept(predicate, db, result, sense_list)
            
        return result
    
    def add_senses_and_concept(self,word,kb,db,sense_list):
        result = db.duplicate()
        
        for sense in kb.query('has_sense({}, ?s)'.format(word)):
            result.addGroundAtom('has_sense({}, {})'.format(word,sense['?s']))
                    
            if not sense['?s'] in sense_list:
                result.addGroundAtom('is_a({}, {})'.format(sense['?s'],sense['?s']))
                sense_list.append(sense['?s'])
        
        return result        
                

    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        mln = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FuzzyLogic') 
        ac = ActionCoreDbCreator()
        ac.create_actioncore_dbs(input_dir,mln)
            
            
    
    