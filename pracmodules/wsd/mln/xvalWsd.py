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

usage = '''Usage: %prog [options] <predicate> <domain> <mlnfile> <dbfiles>'''

parser = OptionParser(usage=usage)
parser.add_option("-k", "--folds", dest="folds", type='int', default=10,
                  help="Number of folds for k-fold Cross Validation")
parser.add_option("-p", "--percent", dest="percent", type='int', default=100,
                  help="Use only PERCENT% of the data. (default=100)")
parser.add_option("-v", "--verbose", dest="verbose", action='store_true', default=False,
                  help="Verbose mode.")
parser.add_option("-m", "--multicore", dest="multicore", action='store_true', default=False,
                  help="Verbose mode.")
parser.add_option('-n', '--noisy', dest='noisy', type='str', default=None,
                  help='-nDOMAIN defines DOMAIN as a noisy string.')
parser.add_option('-c', '--inCremental', dest="incremental", action='store_true', default=False,
                  help='-Run 2 to k cross validations on one run.')
parser.add_option('-a', '--auto', dest="auto", action='store_true', default=False,
                  help='-Run each cross validation with FL, FOL and the combinations of them.')
parser.add_option('-l', '--learn', dest='learn', type='str', default='FirstOrderLogic',
                  help='-Defines which logic should be used for the learning.')
parser.add_option('-i', '--infer', dest='infer', type='str', default='FirstOrderLogic',
                  help='-Defines which logic should be used for the inference.')
parser.add_option('-1', '--inverse', dest='inverse', action='store_true', default='False',
                  help='-Defines if an inverse cross validation should be done.')

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
        self.optimizer = 'bfgs'
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
        self.fold_id = 'Fold-%d' % params.foldIdx
        self.confMatrix = ConfusionMatrix()
            
    def evalMLN(self, mln, dbs):
        '''
        Returns a confusion matrix for the given (learned) MLN evaluated on
        the databases given in dbs.
        '''
        queryPred = self.params.queryPred
        sig = ['?arg%d' % i for i, _ in enumerate(mln.predicates[queryPred])]
        querytempl = '%s(%s)' % (queryPred, ','.join(sig))
        i = 0
        for db in dbs:
            i = i + 1
            db_ = db.duplicate()
            # save and remove the query predicates from the evidence
            trueDB = Database(mln)
            
            for bindings in db_.query(querytempl):
                atom = querytempl
                for binding in bindings:
                    atom = atom.replace(binding, bindings[binding])
                trueDB.addGroundAtom(atom)
                db_.retractGndAtom(atom)
            
            #WSD specific stuff
            #Remove all is_a predicates of the training db
            for atom in list(db_.iterGroundLiteralStrings('is_a')):
                db_.retractGndAtom(atom[1])
            prac = PRAC()
            prac.mln = mln;
            prac.wordnet = WordNet(concepts=None)
            senses = prac.getModuleByName('wn_senses')
            senses.initialize()
            infer = PRACInference(prac, 'None');
            wsd = prac.getModuleByName('wsd')
            kb = PRACKnowledgeBase(prac)
            kb.query_params = {'cwPreds': [], 'verbose': False, 'closedWorld': 1, 
                               'logic': self.params.logicInfer, 'queries': 'has_sense',
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
                    self.confMatrix.addClassificationResult(truth, pred)

    def run(self):
        '''
        Runs the respective fold of the crossvalidation.
        '''
        log = logging.getLogger(self.fold_id)
        log.info('Running fold %d of %d...' % (self.params.foldIdx + 1, self.params.foldCount))
        directory = self.params.directory
        try:
            # Apply noisy string clustering
            log.debug('Transforming noisy strings...')
            if self.params.noisyStringDomains is not None:
                noisyStrTrans = NoisyStringTransformer(self.params.mln, self.params.noisyStringDomains, True)
                learnDBs_ = noisyStrTrans.materializeNoisyDomains(self.params.learnDBs)
                testDBs_ = noisyStrTrans.transformDBs(self.params.testDBs)
            else:
                learnDBs_ = self.params.learnDBs
                testDBs_ = self.params.testDBs

            # train the MLN
            mln = self.params.mln
            log.debug('Starting learning...')
            learnedMLN = mln.learnWeights(learnDBs_, method=self.params.learningMethod,
                                          verbose=verbose,
                                          evidencePreds=["is_a","has_pos"],
                                          partSize=4)
            
            # store the learned MLN in a file
            learnedMLN.writeToFile(os.path.join(directory, 'run_%d.mln' % self.params.foldIdx))
            log.debug('Finished learning.')
            
            # evaluate the MLN
            log.debug('Evaluating.')
            learnedMLN.setClosedWorldPred(None)
            if self.params.cwPreds is None:
                self.params.cwPreds = [p for p in mln.predicates if p != self.params.queryPred]
            for pred in [pred for pred in self.params.cwPreds if pred in learnedMLN.predicates]:
                learnedMLN.setClosedWorldPred(pred)
            self.evalMLN(learnedMLN, testDBs_)
            self.confMatrix.toFile(os.path.join(directory, 'conf_matrix_%d.cm' % self.params.foldIdx))
            log.debug('Evaluation finished.')
        except (KeyboardInterrupt, SystemExit):
            log.critical("Exiting...")
            return None
        
    
class NoisyStringTransformer(object):
    '''
    This transformer takes a set of strings and performs a clustering
    based on the edit distance. It transforms databases wrt to the clusters.
    '''
    
    def __init__(self, mln, noisyStringDomains, verbose=True):
        self.mln = mln
        self.noisyStringDomains = noisyStringDomains
        self.verbose = verbose
        self.clusters = {} # maps domain name -> list of clusters
        self.noisyDomains = {}
        self.log = logging.getLogger('NoisyString')
    
    def materializeNoisyDomains(self, dbs):
        '''
        For each noisy domain, (1) if there is a static domain specification,
        map the values of that domain in all dbs to their closest neighbor
        in the domain.
        (2) If there is no static domain declaration, apply SAHN clustering
        to the values appearing dbs, take the cluster centroids as the values
        of the domain and map the dbs as in (1).
        '''
        fullDomains = mergeDomains(*[db.domains for db in dbs])
        if self.verbose and len(self.noisyStringDomains) > 0:
            self.log.info('materializing noisy domains...')
        for nDomain in self.noisyStringDomains:
            if fullDomains.get(nDomain, None) is None: continue
            # apply the clustering step
            values = fullDomains[nDomain]
            clusters = SAHN(values)
            self.clusters[nDomain] = clusters
            self.noisyDomains[nDomain] = [c._computeCentroid()[0] for c in clusters]
            if self.verbose:
                self.log.info('  reducing domain %s: %d -> %d values' % (nDomain, len(values), len(clusters)))
                self.log.info('   %s', str(self.noisyDomains[nDomain]))
        return self.transformDBs(dbs)
        
    def transformDBs(self, dbs):
        newDBs = []
        for db in dbs:
            #if len(db.softEvidence) > 0:
            #    raise Exception('This is not yet implemented for soft evidence.')
            commonDoms = set(db.domains.keys()).intersection(set(self.noisyStringDomains))
            if len(commonDoms) == 0:
                newDBs.append(db)
                continue
            newDB = db.duplicate()
            for domain in commonDoms:
                # map the values in the database to the static domain values
                valueMap = dict([(val, computeClosestCluster(val, self.clusters[domain])[1][0]) for val in newDB.domains[domain]])
                newDB.domains[domain] = valueMap.values()
                # replace the affected evidences
                for ev in newDB.evidence.keys():
                    truth = newDB.evidence[ev]
                    _, pred, params = db.mln.logic.parseLiteral(ev)
                    if domain in self.mln.predicates[pred]: # domain is affected by the mapping  
                        newDB.retractGndAtom(ev)
                        newArgs = [v if domain != self.mln.predicates[pred][i] else valueMap[v] for i, v in enumerate(params)]
                        atom = '%s%s(%s)' % ('' if truth else '!', pred, ','.join(newArgs))
                        newDB.addGroundAtom(atom)
            newDBs.append(newDB)
        return newDBs

def runFold(fold):
    log = logging.getLogger(fold.fold_id)
    try:
        fold.run()
    except:
        raise Exception(''.join(traceback.format_exception(*sys.exc_info())))
    return fold

def doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles,logicLearn, logicInfer,inverse=True):  
    startTime = time.time()

    # set up the directory    
    directory = time.strftime("%a_%d_%b_%Y_%H:%M:%S_K-"+str(folds)+'_'+logicLearn+'_'+logicInfer, time.localtime())
    os.mkdir(directory)
    
    # set up the logger
    log = logging.getLogger('xval')
    fileLogger = FileHandler(os.path.join(directory, 'xval.log'))
    fileLogger.setFormatter(praclog.formatter)
    log.addHandler(fileLogger)

    log.info('Results will be written into %s' % directory)

    # preparations: Read the MLN and the databases 
    #mln_ = readMLNFromFile(mlnfile, verbose=verbose, logic='FuzzyLogic', grammar='PRACGrammar')
    mln_ = readMLNFromFile(mlnfile, verbose=verbose, logic=logicLearn, grammar='PRACGrammar')
    log.info('Read MLN %s.' % mlnfile)
    dbs = []
    for dbfile in dbfiles:
        db = readDBFromFile(mln_, dbfile)
        if type(db) is list:
            dbs.extend(db)
        else:
            dbs.append(db)
    log.info('Read %d databases.' % len(dbs))
    
    cwpreds = [pred for pred in mln_.predicates if pred != predName]
    
    # create the partition of data
    subsetLen = int(math.ceil(len(dbs) * percent / 100.0))
    if subsetLen < len(dbs):
        log.info('Using only %d of %d DBs' % (subsetLen, len(dbs)))
    dbs = sample(dbs, subsetLen)

    if len(dbs) < folds:
        log.error('Cannot do %d-fold cross validation with only %d databases.' % (folds, len(dbs)))
        exit(0)
    
    shuffle(dbs)
    partSize = int(math.ceil(len(dbs)/float(folds)))
    partition = []
    for i in range(folds):
        partition.append(dbs[i*partSize:(i+1)*partSize])
    
    
    foldRunnables = []
    for foldIdx in range(folds):
        params = XValFoldParams()
        params.mln = mln_.duplicate()
        params.learnDBs = []
        for dbs in [dbs for i,dbs in enumerate(partition) if i != foldIdx]:
            params.learnDBs.extend(dbs)
        params.testDBs = partition[foldIdx]
        if inverse:
            temp = params.testDBs
            params.testDBs = params.learnDBs
            params.learnDBs = temp 
        params.foldIdx = foldIdx
        params.foldCount = folds
        params.noisyStringDomains = noisy
        params.directory = directory
        params.queryPred = predName
        params.queryDom = domain
        params.logicInfer = logicInfer
        foldRunnables.append(XValFold(params))
    
    if multicore:
        # set up a pool of worker processes
        try:
            workerPool = Pool()
            log.info('Starting %d-fold Cross-Validation in %d processes.' % (folds, workerPool._processes))
            result = workerPool.map_async(runFold, foldRunnables).get()
            workerPool.close()
            workerPool.join()
            cm = ConfusionMatrix()
            for r in result:
                cm.combine(r.confMatrix)
            elapsedTimeMP = time.time() - startTime
            cm.toFile(os.path.join(directory, 'conf_matrix.cm'))
            # create the pdf table and move it into the log directory
            # this is a dirty hack since pdflatex apparently
            # does not support arbitrary output paths
            pdfname = 'conf_matrix'
            cm.toPDF(pdfname)
            os.rename('%s.pdf' % pdfname, os.path.join(directory, '%s.pdf' % pdfname))
        except (KeyboardInterrupt, SystemExit, SystemError):
            log.critical("Caught KeyboardInterrupt, terminating workers")
            workerPool.terminate()
            workerPool.join()
            exit(1)
        except:
            log.error('\n' + ''.join(traceback.format_exception(*sys.exc_info())))
            exit(1)
#     startTime = time.time()
    else:
        log.info('Starting %d-fold Cross-Validation in 1 process.' % (folds))
        cm = ConfusionMatrix()
        for fold in foldRunnables:
            cm.combine(runFold(fold).confMatrix)
        cm.toFile(os.path.join(directory, 'conf_matrix.cm'))
        elapsedTimeSP = time.time() - startTime
    
    if multicore:
        log.info('%d-fold crossvalidation (MP) took %.2f min' % (folds, elapsedTimeMP / 60.0))
    else:
        log.info('%d-fold crossvalidation (SP) took %.2f min' % (folds, elapsedTimeSP / 60.0))

if __name__ == '__main__':
    (options, args) = parser.parse_args()
    folds = options.folds
    percent = options.percent
    verbose = options.verbose
    multicore = options.multicore
    incremental = options.incremental
    noisy = ['text']
    predName = args[0]
    domain = args[1]
    mlnfile = args[2]
    dbfiles = args[3:]
    auto = options.auto
    logicLearn = options.learn
    logicInfer = options.infer
    inverse = options.inverse
    
    if auto:
        #First Run
        logicLearn = 'FirstOrderLogic'
        logicInfer = 'FirstOrderLogic'
        dbfiles = ['fol_training.db']
        
        if incremental:
            for i in range(2,folds+1):
                if folds%i == 0:
                    doXVal(i, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
        else:
            doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer.inverse)
        
        #Second Run
        logicLearn = 'FirstOrderLogic'
        logicInfer = 'FuzzyLogic'
        dbfiles = ['fol_training.db']
        
        if incremental:
            for i in range(2,folds+1):
                if folds%i == 0:
                    doXVal(i, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
        else:
            doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
            
        #Third Run
        logicLearn = 'FuzzyLogic'
        logicInfer = 'FirstOrderLogic'
        dbfiles = ['fuzzy_training.db']
        
        if incremental:
            for i in range(2,folds+1):
                if folds%i == 0:
                    doXVal(i, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
        else:
            doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
            
        #Fourth Run
        logicLearn = 'FuzzyLogic'
        logicInfer = 'FuzzyLogic'
        dbfiles = ['fuzzy_training.db']
        
        if incremental:
            for i in range(2,folds+1):
                if folds%i == 0:
                    doXVal(i, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
        else:
            doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
    else:
        if incremental:
            for i in range(2,folds+1):
                print i
                if folds%i == 0:
                    doXVal(i, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)
        else:
            doXVal(folds, percent, verbose, multicore, noisy, predName, domain, mlnfile, dbfiles, logicLearn, logicInfer,inverse)