import os
import re
import sys
from nltk.corpus import wordnet as wn

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from mln.methods import LearningMethods, InferenceMethods

from multiprocessing import Pool, cpu_count

MLN = None

    
class PredicateValidator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    TRAINING_SET_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','TrainingSetCreator',"result")
    
    def __init__(self,mln):
        self.mln = mln
        self.foldIdx = None
        self.foldCount = None
        self.learnDBs = None
        self.testDBs = None
        self.queryPred = None
        self.queryDom = None
        self.cwPreds = None
        self.learningMethod = LearningMethods.DCLL
        self.optimizer = 'cg'
        self.gtol = 0.001
        self.learningRate = .5
        self.maxrepeat = 1
        self.queryPreds = []
        self.partSize = 1
        self.maxiter = None
        self.verbose = False
        self.noisyStringDomains = None
        self.directory = None 
        
    def valid_sets(self,file_list):
        for filepath in file_list:
            
            try:
                self.process_db_file(filepath)
            except:
                print sys.exc_info()
                
    
    def process_db_file(self,filepath):
        
        
        if filepath.endswith("~"): 
            return 
        
        print filepath
        regex_new_db = re.compile("\n\s*-+\s*\n")
        file = open(filepath,'r')
        dbs_as_textfiles = regex_new_db.split(file.read())
        dbs =[]
        db = readDBFromFile(self.mln,filepath)
        
        if isinstance(db, list):
            dbs = db
        else:
            dbs.append(db)
        
        #train mln
        learnedMLN = self.train_mln(dbs)
        test_dbs = self.generate_test_dbs(dbs)
        
        for test_db in test_dbs:
            test_db.printEvidence()
            print "####################"
        
        file.close()                    
    
    def train_mln(self,dbs):
        return self.mln.learnWeights(dbs, method=self.learningMethod, 
                                      optimizer=self.optimizer, 
                                      gaussianPriorSigma=10.,
                                      verbose=False,
                                      maxiter=self.maxiter,
                                      learningRate=self.learningRate,
                                      partSize=self.partSize,
                                      maxrepeat=self.maxrepeat,
                                      gtol=self.gtol,
                                      evidencePreds=["is_a","dobj","nsubj"],
                                      ignoreZeroWeightFormulas=True)#200
        
    
    def generate_test_dbs(self,train_dbs):
        test_dbs = []
        for db in train_dbs:
            
            predicate_name = ""
            predicate_sense = ""
            
            #Assuming that there is only one  predicate
            for q1 in db.query('predicate(?w)'):
                    predicate_name = q1["?w"]
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                        predicate_sense = q1['?w']
                        
            for element in ['dobj']:
                for q1 in db.query('{}(?w1,?w2)'.format(element)):
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                        sense = q2['?s']
                        syn = wn.synset(sense)
                        hypernyms = syn.hypernyms()
                        for e in hypernyms:
                            for sister_term_syn in e.hyponyms():
                                sister_term_sense = sister_term_syn.name
                                sister_term = sister_term_sense.split(".")[0]
                                
                                test_db = Database(db.mln)                    
                                #Add predicate
                                test_db.addGroundAtom("predicate({})".format(predicate_name))
                                test_db.addGroundAtom('has_sense({},{})'.format(predicate_name,predicate_sense))
                                #test_db.addGroundAtom('is_a({},{})'.format(predicate_sense,predicate_sense))
                                
                                #Add object
                                test_db.addGroundAtom('{}({},{})'.format(element,predicate_name,sister_term))
                                test_db.addGroundAtom('has_sense({},{})'.format(sister_term,sister_term_sense))
                                #test_db.addGroundAtom('is_a({},{})'.format(sister_term_sense,sister_term_sense))
            
                                test_dbs.append(test_db)
        
        return test_dbs
                                
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
    ts = PredicateValidator(MLN)
    ts.valid_sets(list)
    
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
    
    