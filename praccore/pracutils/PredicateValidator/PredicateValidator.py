import os
import re
import sys
from nltk.corpus import wordnet as wn

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from mln.methods import LearningMethods, InferenceMethods

from multiprocessing import Pool, cpu_count

from prac.core import PRAC, PRACKnowledgeBase

from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep, PRACInference

from utils.eval import ConfusionMatrix

MLN = None

PRAC_HOME = os.environ['PRAC_HOME']
CM_SET_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','PredicateValidator',"result")
    
    
class PredicateValidator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    
    
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
        learned_mln = self.train_mln(dbs)
        test_dbs = self.generate_test_dbs(dbs)
        cm = self.start_validation(test_dbs, learned_mln)
        os.makedirs(os.path.join(CM_SET_PATH,os.path.basename(filepath)))
        dbs_file = open(os.path.join(CM_SET_PATH,os.path.basename(filepath),"test.db"),'w')
        Database.writeDBs(test_dbs,dbs_file)
        cm.toFile(os.path.join(CM_SET_PATH,os.path.basename(filepath),os.path.basename(filepath)))
        file.close()                    
        dbs_file.close()
        os.remove(filepath)
        
    def start_validation(self,dbs,learned_mln):
        cm = ConfusionMatrix()
        
        for db in dbs:
            truth_db = Database(db.mln)
            test_db = Database(db.mln)
            
            for atom, truth in sorted(db.evidence.iteritems()):
                if 'has_sense' in atom:
                    truth_db.addGroundAtom(atom,truth)
                    continue
                
                test_db.addGroundAtom(atom,truth)
                
            prac = PRAC()
            prac.mln = learned_mln;
            prac.wordnet = WordNet(concepts=None)
            senses = prac.getModuleByName('wn_senses')
            senses.initialize()
            infer = PRACInference(prac, 'None');
            wsd = prac.getModuleByName('wsd')
            kb = PRACKnowledgeBase(prac)
            kb.query_params = {'verbose': False, 
                               'logic': 'FirstOrderLogic', 'queries': 'has_sense',
                                'debug': 'ERROR', 'useMultiCPU': 0, 'method': 'WCSP'}

            kb.dbs.append(test_db)
            prac.run(infer,wsd,kb=kb)
            inferStep = infer.inference_steps[0]
            resultDB = inferStep.output_dbs[0]
            
            for predicate in truth_db.iterGroundLiteralStrings('has_sense'):
                group = re.split(',',re.split('has_sense\w*\(|\)',predicate[1])[1])
                word = group[0];
                truth = group[1];
                query = 'has_sense('+word+',?s)'
                for result in resultDB.query(query):
                    pred = result['?s']
                    cm.addClassificationResult(truth, pred)
                
            
        return cm
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
                    predicate_name = q1['?w']
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                        predicate_sense = q2['?s']
                        
            for element in ['dobj']:
                for q1 in db.query('{}(?w1,?w2)'.format(element)):
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                        sense = q2['?s']
                        syn = wn.synset(sense)
                        hypernyms = syn.hypernyms()
                        for e in hypernyms:
                            for sister_term_syn in e.hyponyms():
                                sister_term_sense = sister_term_syn.name
                                sister_term = sister_term_sense.split(".")[0] + "-1"
                                test_db = Database(db.mln)                    
                                #Add predicate
                                test_db.addGroundAtom("predicate({})".format(predicate_name))
                                test_db.addGroundAtom('has_sense({},{})'.format(predicate_name,predicate_sense))
                                test_db.addGroundAtom('has_pos({},{})'.format(predicate_name,'VB'))
                                #test_db.addGroundAtom('is_a({},{})'.format(predicate_sense,predicate_sense))
                                
                                #Add object
                                test_db.addGroundAtom('{}({},{})'.format(element,predicate_name,sister_term))
                                test_db.addGroundAtom('has_sense({},{})'.format(sister_term,sister_term_sense))
                                test_db.addGroundAtom('has_pos({},{})'.format(sister_term,'NN'))
                                #test_db.addGroundAtom('is_a({},{})'.format(sister_term_sense,sister_term_sense))
            
                                test_dbs.append(test_db)
        
        return test_dbs

def write_cm_results():
    cm_filename_regex = re.compile("\d+\.db")
    cm_paths = []
    for path in os.listdir(CM_SET_PATH):
        path_ = os.path.join(CM_SET_PATH,path)
        if os.path.isdir(path_) : 
            for filename in os.listdir(path_):
                if cm_filename_regex.search(filename):
                    cm_paths.append(os.path.join(path_,filename))
                    
    ConfusionMatrix.write_comparison_results_between_confusion_matrices(os.path.join(CM_SET_PATH,'OVERALL_RESULT.txt'),*cm_paths)
    
                                 
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
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic') 
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
        write_cm_results()
    
    