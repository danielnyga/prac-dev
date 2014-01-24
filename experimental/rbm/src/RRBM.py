'''
RELATIONAL RESTRICTED BOLTZMANN MACHINES

(C) 2011-2012 by Daniel Nyga

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

from RDB import RDB, DBSchema
from RBM import RBM, RBMWeight, RBMNode
from multiprocessing import Process, Semaphore, Event, Value, cpu_count
import Tools
import time
import os

class RRBM(object):
    '''
    This class implements Relational Restricted Boltzmann Machines
    '''    
    def __init__(self, db, eta, n):
        rdb = RDB(db)
        self.predicates = rdb.predicates
        self.keys = rdb.keys
        self.formulas = rdb.formulas
        self.eta = eta
        self.weights = [[RBMWeight() for p in f] for f in self.formulas]
        self.hbias = [RBMWeight(w=0) for f in self.formulas]
        self.vbias = [RBMWeight(w=0) for i in range(self.getAtomTemplateCount())]
        self.rbms = []
        self.rdbs = []
        self.n = n
        # initialize synchronization infrastructure
#        self.proc_count = Value('i', 0)
#        self.semaphore = Semaphore()
#        self.event = Event()
        
    def __waitForAll(self, proc_count):
        '''
        Implements a "barrier" for proc_count processes, i.e. this method
        blocks until all of the worker processes calling this method have finished.
        '''
        with self.semaphore:
            self.proc_count.value += 1
            if self.proc_count.value == proc_count:
                self.event.set()
        self.event.wait()

        
    def groundRBM(self, rdb): 
        '''
        Computes the grounded RBM for the given database
        '''
        rbm = RBM(self.eta)
        # the ground atoms represent the visible nodes
        rbm.visibleNodes = [RBMNode(idx=i,label=rdb.getReadableName(a)) for i,a in enumerate(rdb.atoms)]
        for i,visibleNode in enumerate(rbm.visibleNodes):
            visibleNode.bias = self.vbias[rdb.atomTemplate(rdb.atoms[i])]
        # the ground formulas represent the hidden nodes
        print 'Generating ground RBM...'
        print '  #visible nodes:', len(rbm.visibleNodes)
        print '  #formulas:',len(self.formulas)
        hiddenNodeCounter = 0
        rbm.gfs = []
        for i_f,f in enumerate(self.formulas):
            gfs = rdb.groundFormula(f)
            for i_gf,gf in enumerate(gfs):
                rbm.gfs.append(gf)
                hiddenNode = RBMNode(idx=i_gf + hiddenNodeCounter,label=self.getReadableGroundFormula(rdb, gf).replace('^', r'^\n'))
                hiddenNode.bias = self.hbias[i_f]
                rbm.hiddenNodes.append(hiddenNode)
                for i,visibleNodeIndex in enumerate(gf):
                    edge = RBM.createEdge(rbm.visibleNodes[visibleNodeIndex], hiddenNode, self.weights[i_f][i])
                    rbm.edges.append(edge)
            hiddenNodeCounter += i_gf + 1
        print '  #hidden nodes:',hiddenNodeCounter
        return rbm
    
    def addDatabase(self,rdb):
        self.rdbs.append(rdb)
        self.rbms.append(self.groundRBM(rdb))
    
    def trainingStep(self):
        '''
        Performs a training iteration on all ground RBMs in parallel
        '''
        self.rbms[0].eraseGradientUpdate()
        for i,rbm in enumerate(self.rbms):
            rbm.processExample(rbm.getRBMSamples(self.rdbs[i].atomsToBinary(self.rdbs[i].evidence, False)), self.n)
        self.rbms[0].updateGradient()
       #     self.__waitForAll(processes)
       # else:
       #     for i in range(processes):
       #         Tools.showProgress(float(i) / float(len(self.rbms)))
       #         p = Process(target=self.trainingStep, args=([processes, self.rbms[i::processes], self.rdbs[i::processes]]))
       #         p.start()
       #         if i == processes-1:
       #             p.join()
       #     print
        
    def getAtomTemplateCount(self):
        templateNo = 0
        for p in self.predicates:
            pTemplateNo = 1
            for i,arg in enumerate(p[1:]):
                if i in self.keys[p[0]]: continue
                pTemplateNo *= arg
            templateNo += pTemplateNo
        return templateNo
    
    def getReadableGroundFormula(self, rdb, gf):
        '''
        Returns a human readable string of the given ground formula and database.
        '''
        return ' ^ '.join([rdb.getReadableName(rdb.atoms[x]) for x in gf])
    
    def reconstruct(self, rdb, n=1):
        '''
        Takes a relational database (RDB) object as an argument and
        prints out the reconstructed database.
        '''
        rbm = self.groundRBM(rdb)
        binaryInput = rdb.atomsToBinary(rdb.evidence, True)
        evidenceIndices = [i for i,e in enumerate(binaryInput) if e is 1.]
        for j in range(n):
            print '===== TEST RUN #%d =====' % (j+1)
            x = rbm.reconstruct(rbm.getRBMSamples(rdb.atomsToBinary(rdb.evidence, True)), self.n*100, evidenceIndices)
            for i,s in enumerate(x):
                print '%.3f\t%s (%s bias)'%(s.prob, rdb.getReadableName(rdb.atoms[i]), str(self.vbias[rdb.atomTemplate(rdb.atoms[i])]))
    
    def printFormulas(self):
        for f_i, f in enumerate(self.formulas):
            f_string = str(self.hbias[f_i])
            for p_i, p in enumerate(f):
                f_string += ' ~ ' + str(self.weights[f_i][p_i]) + ' ' + '%s(%s)'%(self.rdbs[0].idxPredMap[p[0]],','.join([self.rdbs[0].idxValMap[(p[0],i,j)] for i,j in enumerate(p[1:]) if j >= 0]))
            print f_string
    
if __name__ == '__main__':
    # stop watch
    start = time.time()
    cd = 15
    path = '../data/language/filling/db/'
#    path = '../data/language/positioning/'
#    path = '../data/language/multiple_databases/'
#    db = DBSchema(schemaFile='../data/language/senses.layout', formulaFile='../data/language/senses.formulas')
    db = DBSchema(schemaFile='../data/language/filling/senses.layout', formulaFile='../data/language/senses.formulas')
#    db = DBSchema(schemaFile='../data/positionings/pos.layout', formulaFile='../data/positionings/formulas_1')
#    rrbm = RRBM(db, 0.00001 / len(os.listdir(path)), cd)
    rrbm = RRBM(db, 0.0001, cd)
    
    print 'Loading training databases...'
    for f in os.listdir(path):
        if os.path.isdir(path + f): continue
        rrbm.addDatabase(db.loadDatabase(path + f))
#    rrbm.addDatabase(db.loadDatabase('../data/language/senses_training.db'))
#    rrbm.addDatabase(db.loadDatabase('../data/positionings/training_breakfast_1.db'))
#    rrbm.addDatabase(db.loadDatabase('../data/positionings/training_lunch_1.db'))
    stop = time.time()
    print 'Model generation completed after %.2f sec' % (stop - start)
#    print 'Learning model parameters using %d CPU cores' % cpu_count()
    start = time.time()
    for i in range(500):
        #if i % 10 == 0: 
        print 'Training iteration #%d' % (i)
        rrbm.printFormulas()
        rrbm.trainingStep()
    stop = time.time()
    
    print 'Parameter learning completed after %.2f sec' % (stop - start)
    
#    testDB = db.loadDatabase('../data/language/senses_test.db')
    testDB = db.loadDatabase('../data/language/filling/filling.test.db')
#    testDB = db.loadDatabase('../data/positionings/training_breakfast_2.db')
    rrbm.reconstruct(testDB, n=10)
