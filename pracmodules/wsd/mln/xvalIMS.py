import traceback

import os
import time
import re
import sys
from nltk.corpus import wordnet as wn
from prac.wordnet import WordNet
from random import shuffle

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from nltk.stem import WordNetLemmatizer
from utils.eval import ConfusionMatrix

import xml.etree.cElementTree as ET
import math


wnl = WordNetLemmatizer()
MLN = None
regex_new_db = re.compile("\n\s*-+\s*\n")
wordnet = WordNet(None)

nounTags = ['NN', 'NNS', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'

mapping_noun_dict = None
mapping_verb_dict = None
mapping_adj_dict = None

wordnet_index_sense = None

class XValFoldParams(object):
    
    def __init__(self, **params):
        self.pdict = params
        self.foldIdx = None
        self.foldCount = None
        self.learnDBs = None
        self.testDBs = None
        
class XValFold(object):
    '''
    Class representing and providing methods for a cross validation fold.
    '''
    
    def __init__(self, params):
        '''
        params being a XValFoldParams object.  
        '''
        self.params = params
        self.fold_id = 'Fold-%d' % params.foldIdx
        self.confMatrix = ConfusionMatrix()
            
    def evalMLN(self, mln, dbs, logicInfer,cm):
        '''
        Returns a confusion matrix for the given (learned) MLN evaluated on
        the databases given in dbs.
        '''
        mln.predicates.update({'has_pos' : ['word','pos']})
        queryPred = self.params.queryPred
        sig = ['?arg%d' % i for i, _ in enumerate(mln.predicates[queryPred])]
        querytempl = '%s(%s)' % (queryPred, ','.join(sig))
        i = 0
        for db in dbs:
            i = i + 1
            db_ = Database(mln)
            # save and remove the query predicates from the evidence
            trueDB = Database(mln)
            
            for bindings in db.query(querytempl):
                atom = querytempl
                for binding in bindings:
                    atom = atom.replace(binding, bindings[binding])
                trueDB.addGroundAtom(atom)
                #db_.retractGndAtom(atom)
            
            #WSD specific stuff
            #Remove all is_a predicates of the training db
            for pred in ['has_pos','is_a','predicate','dobj']:
                for atom in list(db.iterGroundLiteralStrings(pred)):
                    db_.addGroundAtom(atom[1])
            prac = PRAC()
            prac.mln = mln;
            prac.wordnet = WordNet(concepts=None)
            senses = prac.getModuleByName('wn_senses')
            senses.initialize()
            infer = PRACInference(prac, 'None');
            wsd = prac.getModuleByName('wsd')
            kb = PRACKnowledgeBase(prac)
            print "LOGIC INFER: " + logicInfer
            kb.query_params = {'verbose': False, 
                               'logic': logicInfer, 'queries': 'has_sense',
                                'debug': 'ERROR', 'useMultiCPU': 0, 'method': 'WCSP'}

            kb.dbs.append(db_)
            prac.run(infer,wsd,kb=kb)
            inferStep = infer.inference_steps[0]
            resultDB = inferStep.output_dbs[0]
            for predicate in trueDB.iterGroundLiteralStrings('has_sense'):
                group = re.split(',',re.split('has_sense\w*\(|\)',predicate[1])[1])
                word = group[0];
                truth = group[1];
                query = 'has_sense('+word+',?s)'
                for result in resultDB.query(query):
                    pred = result['?s']
                    cm.addClassificationResult(truth, pred)

    def run(self):
        '''
        Runs the respective fold of the crossvalidation.
        '''
        print 'Running fold %d of %d...' % (self.params.foldIdx + 1, self.params.foldCount)
        directory = os.path.join(self.params.directory,str(self.params.foldIdx))
        os.mkdir(directory)
        
        try:
                
            learnDBs_ = self.params.learnDBs
            testDBs_ = self.params.testDBs

            # train
            print 'Starting learning...'
            
            # store the learned MLN in a file
            create_training_set(learnDBs_, directory)
            print 'Finished training set'
            create_test_set(testDBs_,directory )
            print 'Finished test set'
            
            #cm = ConfusionMatrix()
            #self.evalMLN(learnedMLN, testDBs_, 'FirstOrderLogic',cm)
            #cm.toFile(os.path.join(directory,'FOL', 'conf_matrix_%d.cm' % self.params.foldIdx))
            
            print 'Evaluation finished.'
        except (KeyboardInterrupt, SystemExit):
            return None
    
class IMSDatabase(object):
    
    def __init__(self,sentence,db):
        self.sentence = sentence
        self.db = db
        
def get_wn30_to_wn171_dict(file_path):
    print 'Start process'
    mapping_file = open(file_path,'r')
    mapping_list = mapping_file.read().split('\n')
    mapping_dict = {}
    length_of_list = str(len(mapping_list))
    print length_of_list
    
    for mapping in mapping_list:
        mapping_splited = re.split("\s+",mapping)
        
        if len(mapping_splited) >= 2:
            wn30_offset = mapping_splited[0]
            wn17_offset = mapping_splited[1]
            mapping_dict[wn30_offset] = wn17_offset
    
    
    print "Finished calculating wn30_to_wn171_dict"
    mapping_file.close()
    print len(mapping_dict.keys())
    
    return mapping_dict


def get_offset(sense):
    mapping_dict = None
    sense_synset = wordnet.synset(sense)
    syn_pos = sense_synset.pos

    
    if syn_pos == 'n':
        mapping_dict = mapping_noun_dict
    
    if syn_pos == 'v':
        mapping_dict = mapping_verb_dict
    
    if syn_pos == 'a':
        mapping_dict = mapping_adj_dict
    
    if mapping_dict is None:
        return None
    
    offset = str(sense_synset.offset)
    
    if len(offset) < 8:
        for x in range(0,8-len(offset)):
            offset = "0"+offset
    
    if offset not in mapping_dict.keys():
        print "{} cannot be mapped".format(sense)
    else:
        offset = mapping_dict[offset]
    return offset

def runFold(fold):
    try:
        fold.run()
    except:
        print traceback.format_exc()
    return fold
    
def get_wordnet_index_sense():
    index_file = open("index.sense",'r')
    sense_list = index_file.read().split('\n')
    pos_regex = re.compile('(.+)\s+(\d+)')
    
    wordnet_index_sense = {}
    for line in sense_list:
        res = pos_regex.search(line)
        if res:
            splited_res = re.split("\s+",res.group())
            sense_id = splited_res[0]
            offset = splited_res[1]
            wordnet_index_sense[offset] = sense_id
    
    return wordnet_index_sense

def create_test_set(dbs,result_path):
    test_file = open(os.path.join(result_path,'test.txt'),"w")
    
    
    for db in dbs :
        test_file.write("{}\n".format(db.sentence))
    
    test_file.close()
                
def create_training_set(dbs,result_path):
    key_file = open(os.path.join(result_path,'train.key'),"w")
    
    corpus = ET.Element("corpus",lang='english')
    predicate_lemma_list = {}
    dobj_lemma_list = {}
    i = 0
    
    for db_ in dbs:
        db = db_.db
        predicate = ""
        dobj = ""
        pos = ""
        predicate_sense = ""
        dobj_sense = ""
        
        i += 1
        
        for q1 in db.query('predicate(?w)'):
            predicate = wnl.lemmatize('-'.join(q1['?w'].split('-')[:-1]),'v')
            
            for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                predicate_sense = wordnet_index_sense[get_offset(q2['?s'])]
        
        for q1 in db.query('dobj(?w1,?w2)'):
            
            for q2 in db.query('has_pos({},?p)'.format(q1['?w2'])):
                pos = posMap.get(q2['?p'], None)
                if pos is not None:
                    dobj = wnl.lemmatize('-'.join(q1['?w2'].split('-')[:-1]),pos)
                    
                else:
                    dobj = '-'.join(q1['?w2'].split('-')[:-1])
            
            for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    offset = get_offset(q2['?s'])
                    if offset is not None:
                        dobj_sense = wordnet_index_sense[offset]
        
        
        if dobj_sense:
            predicate_lemma = predicate + '.v'
            dobj_lemma = dobj + '.n'
            
            if pos is 'n':
                sentence = '<head>' + predicate+ '</head> the '+ dobj+'.\n' 
            else:
                sentence = '<head>'+predicate+ '</head>' + dobj+'.\n'
            
            if predicate_lemma not in predicate_lemma_list.keys():
                predicate_lemma_list[predicate_lemma] = [[sentence,predicate_sense]]
            else:
                predicate_lemma_list[predicate_lemma].append([sentence,predicate_sense])
            
            if pos is 'n':
                sentence = predicate+ ' the <head>'+ dobj+'</head>.\n' 
            else:
                sentence = predicate+ '<head>'+ dobj+'</head>.\n'
                
            if dobj_lemma not in dobj_lemma_list.keys():
                dobj_lemma_list[dobj_lemma] = [[sentence,dobj_sense]]
            else:
                dobj_lemma_list[dobj_lemma].append([sentence,dobj_sense])
        
            
    #Create xml and key file
    for x in range(0,len(predicate_lemma_list.keys())):
        lexelt = ET.SubElement(corpus, 'lexelt', item=predicate_lemma_list.keys()[x],pos="unk")
        
        for y in range(0,len(predicate_lemma_list[predicate_lemma_list.keys()[x]])):
            instance = predicate_lemma_list[predicate_lemma_list.keys()[x]][y]
            sentence = instance[0]
            sense = instance[1]
            id_str = predicate_lemma_list.keys()[x]+"."+str(y)
            key_file.write("{} {} {}\n".format(predicate_lemma_list.keys()[x],id_str,sense))
            
            instance = ET.SubElement(lexelt, 'instance', id=id_str,docscr="SEMCORE")
            #instance = ET.SubElement(lexelt, 'instance', id=id_str)
            context = ET.SubElement(instance, 'context')
            context.text = sentence
    
    for x in range(0,len(dobj_lemma_list.keys())):
        lexelt = ET.SubElement(corpus, 'lexelt', item=dobj_lemma_list.keys()[x],pos="unk")
        
        for y in range(0,len(dobj_lemma_list[dobj_lemma_list.keys()[x]])):
            instance = dobj_lemma_list[dobj_lemma_list.keys()[x]][y]
            sentence = instance[0]
            sense = instance[1]
            id_str = dobj_lemma_list.keys()[x]+"."+str(y)
            key_file.write("{} {} {}\n".format(dobj_lemma_list.keys()[x],id_str,sense))
            
            instance = ET.SubElement(lexelt, 'instance', id=id_str,docscr="SEMCORE")
            #instance = ET.SubElement(lexelt, 'instance', id=id_str)
            context = ET.SubElement(instance, 'context')
            context.text = sentence
            
    tree = ET.ElementTree(corpus)
    tree.write(os.path.join(result_path,'train.xml'))
    key_file.close()
    
    tree_file = open(os.path.join(result_path,'train.xml'),'r')
    tree_content = tree_file.read()
    tree_file.close()
    os.remove(os.path.join(result_path,'train.xml'))
    
    tree_file = open(os.path.join(result_path,'train.xml'),'w')
    tree_content = tree_content.replace("&lt;","<")   
    tree_content = tree_content.replace("&gt;",">")
    tree_file.write(tree_content)
    tree_file.close()
                
def doXVal(folds, multicore, dbfile,inverse=False,testSetCount=1):
    directory = time.strftime("%a_%d_%b_%Y_%H:%M:%S_K="+str(folds)+"_TSC="+str(testSetCount)+"_"+str(os.path.basename(dbfile))+"_", time.localtime())
    os.mkdir(directory)
    dbs = []
    dbs_mln = readDBFromFile(MLN,dbfile)
    dbs_as_textfiles = regex_new_db.split(open(dbfile,'r').read())
    
    for i in range(0,len(dbs_mln)):
        sentence = dbs_as_textfiles[i].split('\n')[0]
        dbs.append(IMSDatabase(sentence,dbs_mln[i]))
        
    if len(dbs) < folds:
        print 'Cannot do %d-fold cross validation with only %d databases.' % (folds, len(dbs))
        exit(0)
    
    shuffle(dbs)
    partSize = int(math.ceil(len(dbs)/float(folds)))
    partition = []
    for i in range(folds):
        partition.append(dbs[i*partSize:(i+1)*partSize])
    
    
    foldRunnables = []
    for foldIdx in range(folds):
        partion_ = list(partition)
        params = XValFoldParams()
        params.testDBs = []
        params.learnDBs = []
        
        for i in range(0,testSetCount):
            if (foldIdx >= len(partion_)):
                params.testDBs.extend(partion_[0])
                del partion_[0]
            else:     
                params.testDBs.extend(partion_[foldIdx])
                del partion_[foldIdx]
        
        for part in partion_:
            params.learnDBs.extend(part)
            
        print 'LEARN DBS :' + str(len(params.learnDBs))
        print 'TEST DBS :' + str(len(params.testDBs))
        
        params.foldIdx = foldIdx
        params.foldCount = folds
        params.directory = directory
        foldRunnables.append(XValFold(params))
    
    if multicore:
        # set up a pool of worker processes
        try:
            workerPool = Pool(maxtasksperchild=1)
            result = workerPool.map_async(runFold, foldRunnables).get()
            workerPool.close()
            workerPool.join()
            cm = ConfusionMatrix()
            
            for r in result:
                cm.combine(r.confMatrix)
            
        except (KeyboardInterrupt, SystemExit, SystemError):
            workerPool.terminate()
            workerPool.join()
            exit(1)
        except:
            exit(1)
#     startTime = time.time()
    else:
        
        for fold in foldRunnables:
            runFold(fold)
        
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        mapping_noun_dict = get_wn30_to_wn171_dict('wn30-171.noun')
        mapping_verb_dict = get_wn30_to_wn171_dict('wn30-171.verb')
        mapping_adj_dict = get_wn30_to_wn171_dict('wn30-171.adj')
        
        wordnet_index_sense = get_wordnet_index_sense()

        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        for db in file_list:
            doXVal(10, False, db)
        
    