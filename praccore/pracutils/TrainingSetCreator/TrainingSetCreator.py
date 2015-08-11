import os
import re
import sys
from prac.wordnet import WordNet

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile

class TrainingSetCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    TRAINING_SET_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','TrainingSetCreator',"result")
    wordnet = WordNet(concepts=None)
    
    def create_test_sets(self,input_dir,mln):
         
        for filename in os.listdir(input_dir):
            if filename.endswith("~"): continue
            
            try:
                print filename
                file = open(os.path.join(input_dir,filename),'r')
                dbs_as_textfiles = regex_new_db.split(file.read())
                dbs = readDBFromFile(mln,os.path.join(input_dir,filename))
                sorted_dbs_by_predicate_sense = {}
                
                for i in range(0,len(dbs)):
                    db = dbs[i]
                    db_as_textfile = dbs_as_textfiles[i]
                    
                    for q in db.query('predicate(?w)'):
                        predicate = q['?w']
                        #Assuming there is only one sense for each word
                        for sense_query in db,query("has_sense({},?s)".format(predicate)):
                            sense = sense_query['?s']
                            if sense in sorted_dbs_by_predicate_sense:
                                sorted_dbs_by_predicate_sense[sense].append(db)
                            else:
                                sorted_dbs_by_predicate_sense[sense] = [db]
                    
                            
                file.close()
                #os.remove(os.path.join(input_dir,filename))
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
        ts = TrainingSetCreator()
        ts.create_test_sets(input_dir,mln)
            
            
    
    