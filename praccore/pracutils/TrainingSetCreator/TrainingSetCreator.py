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
                regex_new_db = re.compile("\n\s*-+\s*\n")
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
                        for sense_query in db.query("has_sense({},?s)".format(predicate)):
                            sense = sense_query['?s']
                            db_trimmed_sense = self.remove_slots_without_sense(db)
                            
                            db_frame_projection =  self.determine_frame_projection(db_trimmed_sense)
                            
                            if db_frame_projection:
                                if sense in sorted_dbs_by_predicate_sense:
                                    frame_projection_dict = sorted_dbs_by_predicate_sense[sense]
                                    if db_frame_projection in frame_projection_dict:
                                        sorted_dbs_by_predicate_sense[sense][db_frame_projection].append(db_trimmed_sense)
                                    else:
                                        sorted_dbs_by_predicate_sense[sense][db_frame_projection] = [db_trimmed_sense]
                                else:
                                    sorted_dbs_by_predicate_sense[sense] = { db_frame_projection : [db_trimmed_sense]}
                    
                
                self.save_dbs_to_file(sorted_dbs_by_predicate_sense, os.path.join(self.TRAINING_SET_PATH,filename))            
                file.close()
                #os.remove(os.path.join(input_dir,filename))
            except:
                print sys.exc_info()
    
    def remove_slots_without_sense(self,db):
        db_ = Database(db.mln)
        
        for element in ['dobj','iobj','nsubj','prepobj']:
            for q1 in db.query('{}(?w1,?w2)'.format(element)):
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    db_.addGroundAtom('{}({},{})'.format(element,q1['?w1'],q1['?w2']))
                    db_.addGroundAtom('has_sense({},{})'.format(q1['?w2'],q2['?s']))
                    db_.addGroundAtom('is_a({},{})'.format(q2['?s'],q2['?s']))
        
        for q in db.query('predicate(?w)'):
            predicate = q['?w']
            db_.addGroundAtom('predicate({})'.format(q['?w']))
            for sense_query in db.query("has_sense({},?s)".format(q['?w'])):
                db_.addGroundAtom('has_sense({},{})'.format(q['?w'],sense_query['?s']))
                db_.addGroundAtom('is_a({},{})'.format(sense_query['?s'],sense_query['?s']))
        
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
            for _, frame_projections in dbs_dict.iteritems():
                for _, db in frame_projections.iteritems():
                    dbs.extend(db)
    
            if len(dbs) > 0:
                dbs_file = open(filename,'w')
                Database.writeDBs(dbs,dbs_file)
                dbs_file.close()
            
        
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        mln = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FuzzyLogic') 
        ts = TrainingSetCreator()
        ts.create_test_sets(input_dir,mln)
            
            
    
    