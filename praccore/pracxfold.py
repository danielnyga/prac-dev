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
from prac.core import PRAC
from prac.wordnet import WordNet
from prac.inference import PRACInference, PRACInferenceStep
from prac.learning import PRACLearning



def parse_list(option, opt, value, parser):
    setattr(parser.values, option.dest, value.split(','))

usage = '''Usage: %prog [options]'''

parser = OptionParser(usage=usage)
parser.add_option("-k", "--folds", dest="folds", type='int', default=10,
                  help="Number of folds for k-fold Cross Validation")
parser.add_option("-p", "--percent", dest="percent", type='int', default=100,
                  help="Use only PERCENT% of the data. (default=100)")
parser.add_option("-v", "--verbose", dest="verbose", action='store_true', default=False,
                  help="Verbose mode.")
parser.add_option("-c", "--multicore", dest="multicore", action='store_true', default=False,
                  help="Verbose mode.")
parser.add_option('-n', '--noisy', dest='noisy', type='str', default=None,
                  help='-nDOMAIN defines DOMAIN as a noisy string.')
parser.add_option('-f', '--folder', dest='folder', type='str', default=None,
                  help='-f <folder> the folder in which the results shall be saved.')
parser.add_option('-m', '--module', dest='module', type='str', default=None,
                  help='-m <module> the module for the mln to xfold.')
parser.add_option('-d', '--dbs', action='callback', type='string', callback=parse_list, dest='dbs')
parser.add_option('--predicate', dest='predicate', type='str', default=None,
                  help='The predicate.')
parser.add_option('--domain', dest='domain', type='str', default=None,
                  help='The domain.')
parser.add_option('--mln', dest='mln', type='str', default=None,
                  help='The mln needed to load the database files.')
parser.add_option('--logic', dest='logic', type='str', default='FuzzyLogic',
                  help='The logic to load the mln with.')

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
        self.learningMethod = LearningMethods.CLL
        self.optimizer = 'cg'
        self.maxrepeat = 10
        self.partSize = 2
        self.maxiter = 5
        self.verbose = False
        self.noisyStringDomains = None
        self.directory = None
        self.mln = None
        self.module = None
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
        dbfile = open(os.path.join(params.directory, 'test_dbs_%d.db' % params.foldIdx), 'w+')
        Database.writeDBs(params.testDBs, dbfile)
        dbfile.close()
        
            
    def evalMLN(self, mln, dbs):
        '''
        Returns a confusion matrix for the given (learned) MLN evaluated on
        the databases given in dbs.
        '''
        queryPred = self.params.queryPred
        queryDom = self.params.queryDom
        log.info(queryPred)
        log.info(queryDom)
        
        sig = ['?arg%d' % i for i, _ in enumerate(mln.predicates[queryPred])]
        querytempl = '%s(%s)' % (queryPred, ','.join(sig))
        
        dbs = map(lambda db: db.duplicate(mln), dbs)
        module = self.params.module
        
        infer = PRACInference(module.prac, [])
        inferenceStep = PRACInferenceStep(infer, self)
        

        for db in dbs:
            # save and remove the query predicates from the evidence
            trueDB = Database(mln)
            for bindings in db.query(querytempl):
                atom = querytempl
                for binding in bindings:
                    atom = atom.replace(binding, bindings[binding])
                trueDB.addGroundAtom(atom)
                db.retractGndAtom(atom)
            
            try:
                inferenceStep.output_dbs = [db]
                infer.inference_steps.append(inferenceStep)
                module.prac.run(infer,module, mln=mln)
                resultDB = infer.inference_steps[-1].output_dbs[-1]

                sig2 = list(sig)
                log.info(sig)
                log.info(sig2)
                entityIdx = mln.predicates[queryPred].index(queryDom)
                log.info(entityIdx)
                for entity in db.domains[queryDom]:
                    sig2[entityIdx] = entity
                    log.info(sig2)
                    query = '%s(%s)' % (queryPred, ','.join(sig2))
                    log.info(query)
                    for truth in trueDB.query(query):
                        truth = truth.values().pop()
                    for pred in resultDB.query(query):
                        pred = pred.values().pop()
                        log.info(pred)
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

            praclearn = PRACLearning(prac)
            praclearn.otherParams['mln'] = self.params.mlnFileName
            praclearn.otherParams['logic'] = self.params.logic
            praclearn.training_dbs = learnDBs_

            learnedMLN = module.train(praclearn)

            # store the learned MLN in a file
            learnedMLN.writeToFile(os.path.join(directory, 'run_%d.mln' % self.params.foldIdx))
            log.debug('Finished learning.')
            
            # evaluate the MLN
            log.debug('Evaluating.')
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



if __name__ == '__main__':
    print "Running xfold..."
    (options, args) = parser.parse_args()
    folds = options.folds
    percent = options.percent
    verbose = options.verbose
    multicore = options.multicore
    dirname = options.folder
    moduleName = options.module
    noisy = ['text']
    predName = options.predicate
    domain = options.domain
    dbfiles = options.dbs
    mlnFileName = options.mln
    
    startTime = time.time()


    #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  
    # set up the directory    
    #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #  #
    timestamp = time.strftime("%Y-%b-%d-%H-%M-%S", time.localtime())
    if dirname is None:  
        idx = 1
        while True:
            dirname = '%s-%d' % (moduleName, idx)
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

    log.info('Log for %d-fold cross-validation of %s using %s' % (folds, moduleName, dbfiles))
    log.info('Date: %s' % timestamp)
    log.info('Results will be written into %s' % expdir)
    
    
    # load module
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None) # needed?

    # read MLN and dbs
    module = prac.getModuleByName(moduleName)
    mln_ = readMLNFromFile(mlnFileName, logic=options.logic)

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
        params.testDBs = partition[foldIdx]
        params.foldIdx = foldIdx
        params.foldCount = folds
        params.noisyStringDomains = noisy
        params.directory = expdir
        params.queryPred = predName
        params.queryDom = domain
        params.module = module
        params.mlnFileName = mlnFileName
        params.logic = options.logic
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
        
