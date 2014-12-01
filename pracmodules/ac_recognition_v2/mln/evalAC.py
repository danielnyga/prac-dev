# Markov Logic Networks -- Automated Cross-Validation Tool
#
# (C) 2012 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import time
import os
import sys
import traceback

from optparse import OptionParser
from mln.mln import readMLNFromFile
from mln.database import readDBFromFile, Database
from random import shuffle, sample
import math
from mln.methods import LearningMethods, InferenceMethods
from wcsp.converter import WCSPConverter
from utils.eval import ConfusionMatrix
from mln.util import strFormula, mergeDomains
from multiprocessing import Pool
from utils.clustering import SAHN, Cluster, computeClosestCluster
import logging
import praclog
from logging import FileHandler
from prac.core import PRAC, PRACKnowledgeBase
from prac.wordnet import WordNet
from prac.inference import PRACInference
import re
import pickle
from mlnQueryTool import MLNInfer

usage = '''Usage: %prog [options] <mlnfile> <dbfiles>'''

parser = OptionParser(usage=usage)
parser.add_option("-v", "--verbose", dest="verbose", action='store_true', default=False,
                  help="Verbose mode.")
parser.add_option("-m", "--multicore", dest="multicore", action='store_true', default=False,
                  help="Verbose mode.")

class XValFoldParams(object):
    
    def __init__(self, **params):
        self.pdict = params
        self.foldIdx = None
        self.foldCount = None
        self.learnDBs = None
        self.testDBs = None
        self.queryPred = None
        self.queryDom = None
        self.cwPreds = None
        self.learningMethod = LearningMethods.DCLL
        self.optimizer = 'cg'
        self.verbose = False
        self.noisyStringDomains = None
        self.directory = None
        self.mln = None
        self.logicInfer = None
        for p, val in params.iteritems():
            setattr(self, str(p), val)
            
class XValFold(object):
    '''
    Class representing and providing methods for a cross validation fold.
    '''
    
    def __init__(self, params):
        '''
        params being a XValFoldParams object.  
        '''
        self.params = params
        self.confMatrix = ConfusionMatrix()
            
    def evalMLN(self, mln, dbs, logicInfer,cm):
        '''
        Returns a confusion matrix for the given (learned) MLN evaluated on
        the databases given in dbs.
        '''
        queryPred = self.params.queryPred
        sig = ['?arg%d' % i for i, _ in enumerate(mln.predicates[queryPred])]
        querytempl = '%s(%s)' % (queryPred, ','.join(sig))
        i = 0
        
        for db in dbs:
            
            db_ = Database(mln)
            # save and remove the query predicates from the evidence
            trueDB = Database(mln)
            
            for bindings in db.query(querytempl):
                atom = querytempl
                for binding in bindings:
                    atom = atom.replace(binding, bindings[binding])
                trueDB.addGroundAtom(atom)
            
            #for atom in list(db.iterGroundLiteralStrings('has_pos')):
            #    db_.addGroundAtom(atom[1])
            
            wordnet = WordNet(concepts=None)
            
            for atom in list(db.iterGroundLiteralStrings('has_pos')):
                group = re.split(',',re.split('has_pos\w*\(|\)',atom[1])[1])
                word = group[0]
                tag = group[1];
                
                if tag == 'NN':
                    tag = 'n'
                else:
                    tag = 'v'
                
                known_concepts = mln.domains.get('concept', [])
                for concept in known_concepts:    
                    for syn in wordnet.synsets(word,tag):
                        sim = wordnet.wup_similarity(syn,concept)
                        atom =  'is_a(%s,%s)' % (syn.name, concept)
                        atomExists = False
                        #To aviod the same evidences
                        for _ in db_.query(atom):
                            atomExists = True
                        if atomExists == False and sim > 0.75:
                            db_.addGroundAtom(atom,sim)
            db.writeToFile("start.db")
            db_.writeToFile("temp.db")    
            resultDB = mln.infer(InferenceMethods.WCSP, queryPred, db_, cwPreds=["is_a"])
            
            for predicate in trueDB.iterGroundLiteralStrings('ac_word'):
                group = re.split(',',re.split('ac_word\w*\(|\)',predicate[1])[1])
                truth = group[0];
                query = 'ac_word(?s)'
                for result in resultDB.query(query):
                    pred = result['?s']
                    cm.addClassificationResult(truth, pred)
                    
    def run(self):
        '''
        Runs the respective fold of the crossvalidation.
        '''
        log = logging.getLogger('Random')
        directory = self.params.directory
        testDBs_ = self.params.testDBs

        # train the MLN
        learnedMLN = self.params.mln
        
        # store the learned MLN in a file
        learnedMLN.writeToFile(os.path.join(directory, 'run_%d.mln' % 0))
        
        # evaluate the MLN
        log.debug('Evaluating.')
        learnedMLN.setClosedWorldPred('is_a')
        
        cm = ConfusionMatrix()
        self.evalMLN(learnedMLN, testDBs_, 'FuzzyLogic',cm)
        cm.toFile(os.path.join(directory, 'conf_matrix.cm'))
        pdfName = "conf_matrix" 
        cm.toPDF(pdfName)
        os.rename('%s.pdf' % pdfName, os.path.join(directory, '%s.pdf' % pdfName))
        os.remove(pdfName+".tex" )
        os.remove(pdfName+".log" )
        os.remove(pdfName+".aux" )
        log.debug('Evaluation finished.')
        
    
def runFold(fold):
    log = logging.getLogger("test")
    try:
        fold.run()
    except:
        raise Exception(''.join(traceback.format_exception(*sys.exc_info())))
    return fold

        
def doXVal(verbose, multicore, mlnfile, dbfiles):  
    startTime = time.time()
    
    directory = time.strftime("%a_%d_%b_%Y_%H:%M:%S_MLN="+mlnfile+";_DB="+dbfiles[0], time.localtime())
    os.mkdir(directory)

    # set up the logger
    log = logging.getLogger('xval')
    fileLogger = FileHandler(os.path.join(directory, 'xval.log'))
    fileLogger.setFormatter(praclog.formatter)
    log.addHandler(fileLogger)

    log.info('Results will be written into %s' % directory)

    # preparations: Read the MLN and the databases 
    mln_ = readMLNFromFile(mlnfile, verbose=verbose, logic='FuzzyLogic', grammar='PRACGrammar')
    log.info('Read MLN %s.' % mlnfile)
    dbs = []
    for dbfile in dbfiles:
        db = readDBFromFile(mln_, dbfile,True)
        if type(db) is list:
            dbs.extend(db)
        else:
            dbs.append(db)
    log.info('Read %d databases.' % len(dbs))
    
    #cwpreds = [pred for pred in mln_.predicates if pred != predName]
    
    # create the partition of data
    params = XValFoldParams()
    params.mln = mln_.duplicate()
    params.testDBs = []
    
    for db in dbs:
        params.testDBs.append(db)
        
    params.directory = directory
    params.queryPred = 'ac_word'
    params.queryDom = 'ac'
    fold = XValFold(params)

    log.info('Starting %d-fold Cross-Validation in 1 process.' % 1)
    
    runFold(fold)
    
    elapsedTimeSP = time.time() - startTime
    log.info('%d-fold crossvalidation (SP) took %.2f min' % (1, elapsedTimeSP / 60.0))

if __name__ == '__main__':
    (options, args) = parser.parse_args()

    # set up the directory 
    

    multicore = options.multicore
    mlnfile = args[0]
    dbfiles = args[1:]
    verbose = False
    
    doXVal(verbose, multicore, mlnfile, dbfiles)
