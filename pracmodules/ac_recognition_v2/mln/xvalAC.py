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
import shutil

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
from collections import defaultdict
from prac.wordnet import WordNet
from isACreator import createIsAEvidence

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
parser.add_option('-f', '--folder', dest='folder', type='str', default=None,
                  help='-f <folder> the folder in which the results shall be saved.')

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
        self.gtol = 0.001
        self.learningRate = .5
        self.maxrepeat = 1
        self.queryPreds = ['object']
        self.partSize = 1
        self.maxiter = None
        self.verbose = False
        self.noisyStringDomains = None
        self.directory = None
        self.mln = None
        for p, val in params.iteritems():
            setattr(self, str(p), val)
            
    def __str__(self):
        return '\n'.join(['%s: %s' % (k, v) for k, v in self.__dict__.iteritems()])
        
            
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
        # write the training and testing databases into a file
        dbfile = open(os.path.join(params.directory, 'train_dbs_%d.db' % params.foldIdx), 'w+')
        Database.writeDBs(params.learnDBs, dbfile)
        dbfile.close()
        
            
    def evalMLN(self, mln, dbs):
        '''
        Returns a confusion matrix for the given (learned) MLN evaluated on
        the databases given in dbs.
        '''
        queryPred = self.params.queryPred
        queryDom = self.params.queryDom
        
        sig = ['?arg%d' % i for i, _ in enumerate(mln.predicates[queryPred])]
        querytempl = '%s(%s)' % (queryPred, ','.join(sig))
        
        dbs = map(lambda db: db.duplicate(), dbs)
        
        for db in dbs:
            db_ = Database(mln)
            # save and remove the query predicates from the evidence
            trueDB = Database(mln)
            for bindings in db.query(querytempl):
                atom = querytempl
                for binding in bindings:
                    atom = atom.replace(binding, bindings[binding])
                trueDB.addGroundAtom(atom)
                #db.retractGndAtom(atom)
            for pred in ['dobj','is_a']:
                for atom in list(db.iterGroundLiteralStrings(pred)):
                    db_.addGroundAtom(atom[1],atom[0])
            
            try:
                db_.writeToFile(os.path.join(self.params.directory, 'test_infer_dbs_%d.db' % self.params.foldIdx))
                
                resultDB = mln.infer(InferenceMethods.WCSP, queryPred, db_, cwPreds=[p for p in mln.predicates if p != self.params.queryPred])
                
                sig2 = list(sig)
                entityIdx = mln.predicates[queryPred].index(queryDom)
                for entity in db.domains[queryDom]:
                    sig2[entityIdx] = entity
                    query = '%s(%s)' % (queryPred, ','.join(sig2))
                    for truth in trueDB.query(query):
                        truth = truth.values().pop()
                    for pred in resultDB.query(query):
                        pred = pred.values().pop()
                    self.confMatrix.addClassificationResult(pred, truth)
                for e, v in trueDB.evidence.iteritems():
                    if v is not None:
                        db.addGroundAtom('%s%s' % ('' if v is True else '!', e))
            except:
                log.critical(''.join(traceback.format_exception(*sys.exc_info())))

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
                                          optimizer=self.params.optimizer, 
                                          gaussianPriorSigma=10.,
                                          verbose=verbose,
                                          maxiter=self.params.maxiter,
                                          learningRate=self.params.learningRate,
                                          partSize=self.params.partSize,
                                          queryPreds=self.params.queryPreds,
                                          maxrepeat=self.params.maxrepeat,
                                          gtol=self.params.gtol,
                                          evidencePreds=["is_a","dobj"])#200
            # store the learned MLN in a file
            learnedMLN.writeToFile(os.path.join(directory, 'run_%d.mln' % self.params.foldIdx))
            learnedMLN = readMLNFromFile(os.path.join(directory, 'run_%d.mln' % self.params.foldIdx), verbose=verbose,logic="FuzzyLogic")
            log.debug('Finished learning.')
            # evaluate the MLN
            log.debug('Evaluating.')
#             learnedMLN.setClosedWorldPred(None)
#             if self.params.cwPreds is None:
#                 self.params.cwPreds = [p for p in mln.predicates if p != self.params.queryPred]
#             for pred in [pred for pred in self.params.cwPreds if pred in learnedMLN.predicates]:
#                 learnedMLN.setClosedWorldPred(pred)
            testDBs_ = createTestDBs(learnedMLN, testDBs_ )
            dbfile = open(os.path.join(directory, 'test_dbs_%d.db' % self.params.foldIdx), 'w+')
            Database.writeDBs(testDBs_, dbfile)
            dbfile.close()
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
#             if len(db.softEvidence) > 0:
#                 raise Exception('This is not yet implemented for soft evidence.')
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

def createLearnDBs(mln,dbs):
    return createIsAEvidence(mln, dbs, 'sense', 'has_sense(?w,?s)', "?s", False)

def createTestDBs(mln,dbs):
    # mapping from PennTreebank POS tags to NLTK POS Tags
    nounTags = ['NN', 'NNS', 'NNP', 'CD']
    verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
    posMap = {}
    for n in nounTags:
        posMap[n] = 'n'
    for v in verbTags:
        posMap[v] = 'v'
    
    dbs_ = []
    wordnet = WordNet(concepts=None)
    
    concepts = mln.domains.get('concept', [])
    word2senses = defaultdict(list)
    for db in dbs:
        
        db_ = db.duplicate()
        for res in db.query('has_pos(?word,?pos)'):
            word_const = res['?word']
            pos = posMap.get(res['?pos'], None)
            
            word = word_const.split('-')[0]
            for i, synset in enumerate(wordnet.synsets(word, pos)):
                sense_id = synset.name#'%s-%.2d' % (word_const, i+1)
                word2senses[word_const].append(sense_id)
                for concept in concepts:
#                     sim = wordnet.semilarity(synset, concept)
                    sim = wordnet.wup_similarity(synset, concept)
                    
                    atom =  'is_a(%s,%s)' % (sense_id, concept)
                    atomExists = False
                    #To aviod the same evidences
                    for _ in db_.query(atom):
                        atomExists = True
                    if atomExists == False:
                        db_.addGroundAtom('is_a(%s,%s)' % (sense_id, concept),sim)
        
        dbs_.append(db_)
    return dbs_

if __name__ == '__main__':
    (options, args) = parser.parse_args()
    folds = options.folds
    percent = options.percent
    verbose = options.verbose
    multicore = options.multicore
    dirname = options.folder
    noisy = None
    predName = args[0]
    domain = args[1]
    mlnfile = args[2]
    dbfiles = args[3:]
    
    startTime = time.time()

    #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  
    # set up the directory    
    #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #
    timestamp = time.strftime("%Y-%b-%d-%H-%M-%S", time.localtime())
    if dirname is None:  
        mlnname = mlnfile[:-4]
        idx = 1
        while True:
            dirname = '%s-%d' % (mlnname, idx)
            idx += 1
            if not os.path.exists(dirname): break
        dirname += '-' + timestamp
    
    expdir = os.getenv('PRACMLN_EXPERIMENTS', '.')
    expdir = os.path.join(expdir, dirname)
    if os.path.exists(expdir):
        print 'Directory "%s" exists. Overwrite? ([y]/n)' % expdir,
        answer = sys.stdin.read(1)
        if answer not in ('y','\n'):
            exit(0)
        else:
            shutil.rmtree(expdir)
    os.mkdir(expdir)
    # set up the logger
    logging.getLogger().setLevel(logging.INFO)
    log = logging.getLogger()
    fileLogger = FileHandler(os.path.join(expdir, 'xval.log'))
    fileLogger.setFormatter(praclog.formatter)
    log.addHandler(fileLogger)

    log.info('Log for %d-fold cross-validation of %s using %s' % (folds, mlnfile, dbfiles))
    log.info('Date: %s' % timestamp)
    log.info('Results will be written into %s' % expdir)

    # preparations: Read the MLN and the databases 
    mln_ = readMLNFromFile(mlnfile, verbose=verbose)
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
        for dbs in [d for i, d in enumerate(partition) if i != foldIdx]:
            params.learnDBs.extend(dbs)
        params.learnDBs = createLearnDBs(params.mln, params.learnDBs)
        params.testDBs = createTestDBs(params.mln, partition[foldIdx])
        params.foldIdx = foldIdx
        params.foldCount = folds
        params.noisyStringDomains = noisy
        params.directory = expdir
        params.queryPred = predName
        params.queryDom = domain
        foldRunnables.append(XValFold(params))
        log.info('Params for fold %d:\n%s' % (foldIdx, str(params)))
    
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
            cm.toFile(os.path.join(expdir, 'conf_matrix.cm'))
            # create the pdf table and move it into the log directory
            # this is a dirty hack since pdflatex apparently
            # does not support arbitrary output paths
            pdfname = 'conf_matrix'
            log.info('creating pdf if confusion matrix...')
            cm.toPDF(pdfname)
            os.rename('%s.pdf' % pdfname, os.path.join(expdir, '%s.pdf' % pdfname))
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
        cm.toFile(os.path.join(expdir, 'conf_matrix.cm'))
        pdfname = 'conf_matrix'
        log.info('creating pdf if confusion matrix...')
        cm.toPDF(pdfname)
        os.rename('%s.pdf' % pdfname, os.path.join(expdir, '%s.pdf' % pdfname))
        elapsedTimeSP = time.time() - startTime
    
    if multicore:
        log.info('%d-fold crossvalidation (MP) took %.2f min' % (folds, elapsedTimeMP / 60.0))
    else:
        log.info('%d-fold crossvalidation (SP) took %.2f min' % (folds, elapsedTimeSP / 60.0))
        
