import traceback

import os
import re
import sys
from nltk.corpus import wordnet as wn,WordNetCorpusReader


from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from mln.methods import LearningMethods, InferenceMethods

from multiprocessing import Pool, cpu_count

from prac.core import PRAC, PRACKnowledgeBase

from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep, PRACInference

from utils.eval import ConfusionMatrix

import imp

MLN = None

PRAC_HOME = os.environ['PRAC_HOME']
CM_SET_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','PredicateValidator',"IMS","ims_result")
IMS_XVAL_MODULE = imp.load_source("xvalIMS","/home/seba/workspace/prac/pracmodules/wsd/mln/ims_0.9.2.1/xvalIMS.py")    
wn_171 = WordNetCorpusReader('/home/seba/Downloads/WordNet-1.7.1/dict', None)


offsets_noun_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('n'))])
offsets_verb_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('v'))])
offsets_adj_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('a'))])

class TestDatabase(object):
        def __init__(self,predicate_truth,dobj_truth,sentence):
            self.predicate_truth = predicate_truth
            self.dobj_truth = dobj_truth
            self.sentence = sentence
            
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
                print traceback.format_exc()
                
    
    def process_db_file(self,filepath):
        
        
        if filepath.endswith("~"): 
            return 
        
        print filepath
        dbs = IMS_XVAL_MODULE.create_IMS_dbs(MLN,filepath)
        result_path = os.path.abspath(os.path.join(CM_SET_PATH,os.path.basename(filepath)))
        os.makedirs(result_path)
        #train mln
        self.train_mln(dbs,result_path)
        
        
        test_dbs = self.generate_test_dbs(dbs,result_path)
        
        cm = self.start_validation(test_dbs, result_path)
        self.write_test_dbs(test_dbs,result_path)
        cm.toFile(os.path.join(result_path,os.path.basename(filepath)))
    
    def write_test_dbs(self,test_dbs,result_path):
        test_db_file = open(os.path.join(result_path,'test.db'),'w')
        
        for test_db in test_dbs:
            test_db_file.write("{}\n".format(test_db.predicate_truth))
            test_db_file.write("{}\n".format(test_db.dobj_truth))
            test_db_file.write("---\n".format(test_db.predicate_truth))
        
        test_db_file.close()
                    
    def start_validation(self,dbs,result_path):
        cm = ConfusionMatrix()
        
        IMS_XVAL_MODULE.test_classifier(os.path.abspath(os.path.join(result_path,'test.txt')), 
                                            os.path.abspath(os.path.join(result_path,'model')),
                                            os.path.abspath(os.path.join(result_path,'result.txt')))
        
        pred_dbs = IMS_XVAL_MODULE.get_highest_probable_senses(os.path.abspath(os.path.join(result_path,'result.txt')))
        for i in range(0,len(dbs)):
            truth_db = dbs[i]
            pred_db = pred_dbs[i]
            
            if len(pred_db) > 0:
                cm.addClassificationResult(truth_db.predicate_truth, pred_db[0])
                cm.addClassificationResult(truth_db.dobj_truth, pred_db[-1])
            else:
                cm.addClassificationResult(truth_db.predicate_truth, "NONE")
                cm.addClassificationResult(truth_db.dobj_truth, "NONE")
                
        return cm
    
    def train_mln(self,dbs,result_path):
        os.mkdir(os.path.join(result_path,'model'))
        IMS_XVAL_MODULE.create_training_set(dbs,result_path)
        IMS_XVAL_MODULE.train_classifier(os.path.join(result_path,'train.xml'), 
                             os.path.join(result_path,'train.key'), 
                             os.path.join(result_path,'model'))
    
    def generate_test_dbs(self,train_dbs,result_path):
        test_dbs = []
        for db_ in train_dbs:
            
            db = db_.db
            predicate_name = ""
            predicate_sense = ""
            
            #Assuming that there is only one  predicate
            for q1 in db.query('predicate(?w)'):
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                        predicate_name = IMS_XVAL_MODULE.transform_30_nltk_sense_to_17_nltk_sense(q2['?s']).name().split(".")[0]
                        predicate_sense = IMS_XVAL_MODULE.transform_30_nltk_sense_to_ims_sense(q2['?s']) 
                        
            for element in ['dobj']:
                for q1 in db.query('{}(?w1,?w2)'.format(element)):
                    for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                        syn = IMS_XVAL_MODULE.transform_30_nltk_sense_to_17_nltk_sense(q2['?s'])
                        if syn is not None:
                            hypernyms = syn.hypernyms()
                            for e in hypernyms:
                                for sister_term_syn in e.hyponyms():
                                    sister_term_sense = sister_term_syn.name()
                                    sister_term = sister_term_sense.split(".")[0]
                                    dobj_sense = IMS_XVAL_MODULE.transform_17_nltk_sense_to_ims_sense(sister_term_sense)
                                    sentence = "{} the {}.".format(predicate_name,sister_term)
                                    
                                    test_db = TestDatabase(predicate_sense, dobj_sense, sentence)                    
                                    test_dbs.append(test_db)
            
        IMS_XVAL_MODULE.create_test_set(test_dbs,result_path)
        
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
    
    