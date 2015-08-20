import os
import re
import sys
from nltk.corpus import wordnet as wn

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count

MLN = None

class Frame(object):
    def __init__(self,db):
        self._frame_projection_type = ' '
        self._dobj_dict = {}
        self._prepobj_dict = {}
        self._iobj_dict = {}
        self._nsubj_dict = {}
        self._db = None
        
        for element in ['dobj','iobj','nsubj','prepobj']:
            for q in db.query('{}(?w1,?w2)'.format(element)):
                self._frame_projection_type += element
                break
        
        for element in ['dobj','iobj','nsubj','prepobj']:
            for q1 in db.query('{}(?w1,?w2)'.format(element)):
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    eval("self._%s_dict.update({q1['?w2'] : q2['?s']})" % (element))
        
        self._db = db
    
class TrainingSetCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    TRAINING_SET_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','TrainingSetCreator',"result")
    
    def __init__(self,mln):
        self.mln = mln 
    
    def create_test_sets(self,file_list):
         
        for filepath in file_list:
            
            try:
                sorted_dbs_by_predicate_sense = self.process_db_file(filepath)
                #self.remove_dbs_containing_sister_terms(sorted_dbs_by_predicate_sense)
                self.save_dbs_to_file(sorted_dbs_by_predicate_sense, os.path.join(self.TRAINING_SET_PATH,os.path.basename(filepath)))            
                
                #os.remove(os.path.join(input_dir,filepath))
            except:
                print sys.exc_info()
    
    def process_db_file(self,filepath):
        
        sorted_dbs_by_predicate_sense = {}
        
        if filepath.endswith("~"): 
            return sorted_dbs_by_predicate_sense
        
        print filepath
        regex_new_db = re.compile("\n\s*-+\s*\n")
        file = open(filepath,'r')
        dbs_as_textfiles = regex_new_db.split(file.read())
        dbs = []
        db = readDBFromFile(self.mln,filepath)
        
        if isinstance(db, list):
            dbs = db
        else:
            dbs.append(db)
            
        for i in range(0,len(dbs)):
            db = dbs[i]
            db_as_textfile = dbs_as_textfiles[i]
            
            for q in db.query('predicate(?w)'):
                predicate = q['?w']
                
                #Assuming there is only one sense for each word
                for sense_query in db.query("has_sense({},?s)".format(predicate)):
                    sense = sense_query['?s']
                    db_trimmed_sense = self.remove_slots_without_sense(db)
                    
                    db_frame_projection =  self.determine_frame_projection(db_trimmed_sense)
                    
                    if db_frame_projection:
                        if sense in sorted_dbs_by_predicate_sense:
                            sorted_dbs_by_predicate_sense[sense].append(Frame(db_trimmed_sense))
                        else:
                            sorted_dbs_by_predicate_sense[sense] = [Frame(db_trimmed_sense)]
        
        file.close()                    
        
        return sorted_dbs_by_predicate_sense
    
    def remove_slots_without_sense(self,db):
        db_ = Database(db.mln)
        
        for element in ['dobj','iobj','nsubj','prepobj']:
            for q1 in db.query('{}(?w1,?w2)'.format(element)):
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    db_.addGroundAtom('{}({},{})'.format(element,q1['?w1'],q1['?w2']))
                    db_.addGroundAtom('has_sense({},{})'.format(q1['?w2'],q2['?s']))
                    db_.addGroundAtom('is_a({},{})'.format(q2['?s'],q2['?s']))
                
                    for q3 in db.query('has_pos({},?p)'.format(q1['?w2'])):
                        db_.addGroundAtom('has_pos({},{})'.format(q1['?w2'],q3['?p']))
        
        for q in db.query('predicate(?w)'):
            predicate = q['?w']
            db_.addGroundAtom('predicate({})'.format(q['?w']))
            for sense_query in db.query("has_sense({},?s)".format(q['?w'])):
                db_.addGroundAtom('has_sense({},{})'.format(q['?w'],sense_query['?s']))
                db_.addGroundAtom('is_a({},{})'.format(sense_query['?s'],sense_query['?s']))
            
                for q3 in db.query('has_pos({},?p)'.format(q['?w'])):
                    db_.addGroundAtom('has_pos({},{})'.format(q['?w'],q3['?p']))
        
        return db_
    
    def determine_frame_projection(self,db):
        frame_projection=""
        
        for element in ['dobj','iobj','nsubj','prepobj']:
            for q in db.query('{}(?w1,?w2)'.format(element)):
                frame_projection += element
                break
        return frame_projection

    def save_dbs_to_file(self,dbs_dict,filename):
        dbs = []
        
        if dbs_dict:
            for _, frames in dbs_dict.iteritems():
                temp_dbs = []
                
                for frame in frames:
                    if frame._dobj_dict:
                        words_to_delete = []
                        words_to_delete.extend(frame._iobj_dict.keys())
                        words_to_delete.extend(frame._prepobj_dict.keys())
                        words_to_delete.extend(frame._nsubj_dict.keys())
                        temp_dbs.append(self.remove_evidence_based_on_words(frame._db, words_to_delete))
                
                if len(temp_dbs) >= 20:
                    dbs.extend(temp_dbs)
                    
    
            if len(dbs) > 0:
                dbs_file = open(filename,'w')
                Database.writeDBs(dbs,dbs_file)
                dbs_file.close()
                
    def remove_evidence_based_on_words(self,db,word_list):
        if not word_list:
            return db
        db_ = Database(db.mln)
        
        for atom, truth in db.evidence.iteritems():
            is_word_in_atom = False
            
            for word in word_list:
                if word in atom:
                    is_word_in_atom = True
                    break
            #TODO remove is_a predicate
            if not is_word_in_atom:
                db_.addGroundAtom(atom,truth)
            
        return db_

def chunks(l, n):
    dividor = len(l)/n
    result = []
    step =  0
    for i in range(1,n):
        result.append(l[step:dividor*i])
        step = dividor*i
    result.append(l[step:])
  
    return result

def run_process(list):
    ts = TrainingSetCreator(MLN)
    ts.create_test_sets(list)
    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FuzzyLogic') 
        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        splitted_file_list = [] 
        for chunk in list(chunks(file_list,cpu_count())):
            splitted_file_list.append(chunk)
        
        workerPool = Pool(processes=cpu_count())
        workerPool.map_async(run_process, splitted_file_list)
        workerPool.close()
        workerPool.join()
    
    