from __future__ import division
from pyswip import *
from FOL import *
#from RT import RelationalTree
import re
import copy
import os
import fnmatch
import sys
from prolog import *

class Domain:
    def __init__(self, name=None, values=None):
        if name is None:
            self.name = ''
        else: self.name = name
        if values is None:
            self.values = []
        else: self.values = values
    
    def __str__(self):
        return self.name + ': ' + str(self.values)
        
class Predicate:
    def __init__(self, name=None, args=None, keys=None,domains=None):
        if name is None:
            self.name = ''
        else: self.name = name
        if args is None:
            self.args = []
        else: self.args = args
        if keys is None:
            self.keys = []
        else: self.keys = keys
        if domains is None:
            self.domains = []
        else: self.domains = domains
        
    def __str__(self):
        return '%s(%s)' % (self.name, ','.join(self.args))

    def isKey(self,variable):
        return self.args.index(variable) in self.keys
        

class RelationalBoosting:
    
    def __init__(self):
        self.domains = {}
        self.predicates = {}
        self.constraints = []
        self.db = []
        self.propositions = []
        self.prolog = Prolog()
        self.trees = []
        self.weights = []
        self.probs = []
        self.functionals = []

    def readModel(self, file):
        f = open(file, 'r')
        lines = f.readlines()
        for l in lines:
            l = l.replace('\n', '')
            l = l.split('//')[0]
            if len(l) == 0:
                continue
            tokens = re.split('\s+',l)
            try:
                weight = float(tokens[0])
                formula = parseFormula("".join(tokens[1:]))
                self.constraints.append(formula)
            except ValueError:
                tokens = re.split('[^a-zA-Z0-9:]+',l)
                tokens = filter(lambda x: len(x) > 0, tokens)
                pargs = tokens[1:]
                name = tokens[0]
                pred = Predicate(name)
                for i, arg in enumerate(pargs):
                    if arg.startswith(':'):
                        pred.keys.append(i)
                        arg = arg[1:]
                    pred.args.append(arg)
                    if arg not in self.domains.keys():
                        self.domains[arg] = Domain(arg)
                    pred.domains.append(self.domains[arg])
                if name not in self.predicates.keys():
                    self.predicates[name] = pred
                                
    def readDatabase(self, db):
        if type(db) == str:
            db = [db]
        for file in db:
            f = open(file, 'r')
            lines = f.readlines()
            for l in lines:
                l = l.split('//')[0]
                if len(l) == 0:
                    continue
                tokens = re.split('\W+',l)
                tokens = filter(lambda x: not len(x) == 0, tokens)
                if len(tokens) == 0:
                    continue
                name = tokens[0]
                values = tokens[1:]
                form = parseFormula(l)
                if form.negated:
                    continue
                self.db.append(parseFormula(l))
                # update the domains
                for i, v in enumerate(values):
                    d = self.predicates[name].domains[i]
                    if v not in d.values:
                        d.values.append(v)
                        
    def assertEvidence(self):
        print 'Asserting...'
        for f in self.db:
            ass = makePrologLiteral(f)
            self.propositions.append(ass.replace('\'',''))
            print '\t' + ass
            self.prolog.assertz(ass)
        print 'done.'
        
def getPredicateNames(preds):
    return set([x.name for x in preds])
    
def getPredicateDomain(preds, predname, i):
    return set([x.args[i] for x in filter(lambda x: x.name == predname, preds)])
    

if __name__ == '__main__':
    dbpath = '.'
    dbs = os.listdir(dbpath)
    dbs = filter(lambda x: fnmatch.fnmatch(x,'*.db'), dbs)

    rb = RelationalBoosting()
    rb.readModel('senseModel.rtb')
    rb.readDatabase(dbpath + os.sep + x for x in dbs)

    #for i,c in enumerate(rb.constraints):
        #rt = RelationalTree('tree_' + str(i), c.toNNF(), {})
        #rb.trees.append(rt.name)
        #rt.save()

    rb.assertEvidence()
    print rb.getBestSplit('', copy.copy(rb.domains))
    
    #print 'listing all ground atoms...'
    #for p in rb.predicates.values():
        #print p.name
        #args = map(lambda x: x, p.args)
        #sol = getAllSolutions(rb.prolog, '%s(%s)' % (p.name,','.join(args)))
        #print sol
        #print '%d solutions found' % len(sol) 
    
    #for t in rb.trees:
        #print getAllSolutions(rb.prolog,'consult(\'%s.pl\')' % t)
    
    #sol = getAllSolutions(rb.prolog, 'tree_1(A,B,C,D,E,F)')
    #for s in sol:
        #print s
    #print len(sol)
    #for p in rb.predicates.values():
        #print p, 'keys: ' + str(p.keys)
    #for d in rb.domains.keys():
        #print d, rb.domains[d]
        #print '================'
    #bindings1 = {'sense': 'S7623263_pie_crust', 'role': 'Goal'}
        
    #f = parseFormula('foo(a) => bar(b) ^ !bar(a)')
    #print f.toNNF()
    #p = Prolog()
    
    #print 'Predicates:', getPredicateNames(preds)
    
    #assertions = map(lambda x: makePrologPredicateString(x), preds)
    #numassert = len(assertions)
    #print 'Making %d assertions...' % numassert,
    #for a in assertions:
        #p.assertz(a)
    #print 'done.'
    
    #roleDomain = getPredicateDomain(preds,'hasRole',1)
    #senseDomain = getPredicateDomain(preds,'hasSense',1)
    #predDecl = [Predicate('hasRole',[],[1,3],[roleDomain]),
                #Predicate('hasSense',[],[1,3],[senseDomain])]
    
    #print 'Role Domain Size:', len(roleDomain)
    #print 'Sense Domain Size:', len(senseDomain)
    
    #for role in roleDomain:
        #q = fillPrologPredicateTempl('hasRole(W,%s,S)',[role])
        #sol = getAllSolutions(p, q)
        ##print len(sol)/numassert, q
    
    #for sense in senseDomain:
        #q = fillPrologPredicateTempl('hasRole(W,%s,S), hasSense(W,%s,S)',['Theme',sense])
        #sol = getAllSolutions(p, q)
        ##print len(sol)/numassert, q
    
    ##print 'found %d solutions' % len(solutions)

