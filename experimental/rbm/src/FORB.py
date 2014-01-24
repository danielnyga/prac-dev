'''
Created on Nov 25, 2011

@author: nyga
'''
import random
from RDB import *
import math

class FORB(object):
    
    def __init__(self,db):
        self.eta = 0.5
        self.atoms = db.getGroundAtoms()
        self.formulas = db.formulas
        v = Formula()
        v.preds = self.atoms
        self.weights = {}
        starttime = time.time()
        self.h_bias = [[0] * len(self.formulas) for i in self.atoms]
        self.v_bias = [0] * len(self.atoms)
        for i_a,a in enumerate(self.atoms):
            for i_f, f in enumerate(db.formulas):
                unified = f.unifyWithGroundAtom(a)
                if unified == None: continue
                for i_p, p in enumerate(unified.preds):
                    matches = v.getMatchingPredicatesIndices(p)
                    self.weights[(i_a,i_f,i_p)] = dict([(m,Weight(i_a)) for m in matches]) 
                    #   print [str(e) for e in edges]
            Tools.showProgress(i_a/float(len(self.atoms)))
        print
        print len(self.atoms), ' atoms'
        print 'Model gerneration took %.2f sec'%(time.time() - starttime)

 
    
    def processExample(self, x, n):
        v1 = x
        v = v1
        h1 = self.sampleHiddenNodes(v)
#        for i in h1:
#            for j in i:
#                sys.stdout.write('\t(%d,%.2f)'%(j[0],j[1]))
#            print
        h = h1
        v_probs = v
        for i in range(n):
            samples = self.sampleVisibleNodes(h)
            v = map(lambda x: x[0], samples)
            v_probs = map(lambda x: x[1], samples)
            if i < n-1: h = self.sampleHiddenNodes(v)
        print
        vis = self.getDatabase(v)
        for p in vis:
            print p
        for i,prob in enumerate(v_probs):
            print prob, self.atoms[i]
                            
        for i in range(len(v)):
            for j in range(len(h[i])):
                for s in range(len(self.formulas[j].preds)):
                    w = Tools.get(self.weights,(i,j,s))
                    if w == None: continue
                    for i_v in w.keys():
                        w[i_v].weight += self.eta * (v1[i] * h1[i][j][1] - v_probs[i] * h[i][j][1])
                
        for i in range(len(self.v_bias)):
            self.v_bias[i] += self.eta * (v1[i] - v_probs[i])
        for i in range(len(h)):
            for j in range(len(h[i])):
                self.h_bias[i][j] += self.eta * (h1[i][j][1] - h[i][j][1])

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))
    
    def sampleHiddenNodes(self, v):
        hidden_nodes = []
        for i in range(len(self.atoms)):
            hidden_units = []
            for j in range(len(self.formulas)):
                n,p = self.sampleHiddenNode(i,j,v)
                hidden_units.append((n,p))
            hidden_nodes.append(hidden_units)
        return hidden_nodes                
        
    def sampleHiddenNode(self, i, j, v):
        activation = self.h_bias[i][j] 
        slots = len(self.formulas[j].preds)
        for s in range(slots):
            weights = Tools.get(self.weights,(i,j,s))
            if weights == None: continue
            for vnode in weights.keys():
                activation += v[vnode] * weights[vnode].weight
        thr = self.sigmoid(activation)
        if random.random() < thr: return 1, thr 
        else: return 0, thr

    def sampleVisibleNodes(self, h):
        visible_nodes = []
        for i in range(len(h)):
            n,p = self.sampleVisibleNode(i, h)
            visible_nodes.append((n,p))
        return visible_nodes
    
    def sampleVisibleNode(self, i, h):
        activation = self.v_bias[i]
        for j in range(len(h[i])):
            # find slot id -> TODO: don't do that
            notthere = False
            for s in range(len(self.formulas[j].preds)):
                w = Tools.get(self.weights,(i,j,s))
                if w == None: 
                    notthere = True
                    break
                if i in w.keys(): 
                    slot = s
                    break
            if notthere: continue
            activation += h[i][j][0] * self.weights[(i,j,slot)][i].weight
        thr = self.sigmoid(activation)
        if random.random() < thr: return 1, thr
        else: return 0, thr
    
    def getBinaryVector(self,db):
        result = []
        for atom in self.atoms:
            if atom in db.groundpreds:
                result.append(1)
            else:
                result.append(0)
        return result
    
    def getDatabase(self,x):
        result = []
        for i,xi in enumerate(x):
            if xi == 1:
                result.append(self.atoms[i])
        return result

    def reconstruct(self, x):
        return self.sampleVisibleNodes(self.sampleHiddenNodes(x))

if __name__ == '__main__':
    db = Database()
    db.readModel('../data/senses_model.txt')
    db.readDatabase('../data/senses_training.db')
#    db.readModel('../data/smoking.hln')
#    db.readDatabase('../data/smokers-training.db')
    print 'Generating model:'
    forb = FORB(db)
    binary = forb.getBinaryVector(db)
    iterations = 20
    print 'Model generation completed.\n'
    print 'Optimizing model parameters:'
    starttime = time.time()
    for i in range(iterations):
        Tools.showProgress((i+1)/float(iterations))
        forb.processExample(binary, 1)
    print
    print 'Optimization completed after %.2f sec.'%(time.time() - starttime)
    
#    e1 = db.predicates['hasSense'].duplicate()
#    e2 = e1.duplicate()
#    e1.args = ['arg0', 'arg1', 'arg2']
#    e2.args = ['arg0', 'arg1', 'arg2']
#    e1.bindings = {'arg0': 'w1', 'arg1':'Water', 'arg2': 's1'}
#    e2.bindings = {'arg0': 'w2', 'arg1':'Cup', 'arg2': 's1'}
#    db.groundpreds = [e1,e2]
#    print '********************\nINFERENCE:'
#    res = forb.reconstruct(forb.getBinaryVector(db))
#    res = forb.getDatabase(map(lambda x: x[0], res))
#    for r in res:
#        print r