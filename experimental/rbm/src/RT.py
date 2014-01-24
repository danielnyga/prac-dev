import copy
from FOL import *
from prolog import *
from pyswip import Prolog
from database import *

class TreeNode:
    def __init__(self):
        self.test = None
        self.trueChild = None
        self.falseChild = None

class RelationalTree:
    
    def __init__(self, name, constraints, data, weights, rb):
        self.name = name
        self.constraints = constraints
        self.weights = weights
        self.data = data
        self.root = None
        self.boost = rb
        self.boundvars = {}

    def learn(self, parent, testsofar):
        pass

    def getVariablesOfConstraint(self, constraint):
        vars = []
        for pred in constraint:
            vars.extend(filter(lambda x: not pred.isKey(x),pred.args))
        return set(vars)

    def getBestSplit(self, testsofar):
        predicates = self.constraints[testsofar[0]]
        bindings = testsofar[1]
        vars = self.getVariablesOfConstraint(predicates)
        for var in vars:
            if var in bindings.keys(): 
            #    print 'skipping %s' % var
                continue
            for p in predicates:
                try: domain = p.domains[p.args.index(var)] 
                except ValueError: continue
                print str(var) + ',' + str(domain)
                
    def getScoreForTest(self, test):
        atoms = self.getPositiveAtoms(test)
        return self.getScoreForAtoms(atoms)
    
    def getScoreForAtoms(self, atoms):
        for a in atoms:
            print a
        datastr = map(lambda x: str(x).replace(' ',''), self.data)
        sum = 0
        for i, a in enumerate(datastr):
            if a in atoms:
                sum += self.weights[i]
        return sum

    def getPositiveAtoms(self, test):
        # test is a tuple [int,dict], where int is the constraint index 
        # and dict is a variable assignment
        assigned = copy.deepcopy(self.constraints[test[0]])
        assign = test[1]
        for p in assigned:
            for i, a in enumerate(p.args):
                if a in assign: p.args[i] = assign[a]

        strtest = map(lambda x: str(x), assigned)

        plstring = ','.join(strtest)
        sol = getAllSolutions(self.boost.prolog, plstring)
        atoms = []
        for c in self.constraints:
            for p in c:
                q = str(p)+','+plstring
                #print q
                sol = getAllSolutions(self.boost.prolog,q)
                for s in sol:
                    args = []
                    for a in p.args:
                        args.append(s[a])
                    atoms.append('%s(%s)'%(p.name,','.join(args)))
        atoms = set(atoms)
        return atoms
    
    def getAllArgs(self, f):
        args = []
        if type(f) is Lit:
            args.extend(f.params)
        else:
            for c in f.children:
                args.extend(self.getAllArgs(c))
        return set(args)
    
    def getArgumentString(self):
        varargs = filter(lambda x: x not in self.bindings.keys(), self.args)
        return ', '.join(map(lambda x: 'VAR' + x, varargs))

    def save(self):
        f = open('%s.pl' % self.name, 'w')
        plstring = ''
        if type(self.formula) is Disjunction:
            for c in self.formula.children:
                plstring += self._getConjPLString(c) + '\n\n'
        elif type(self.formula) is Conjunction \
            or type(self.formula) is Lit:
            plstring += self._getConjPLString(self.formula) + '\n\n'
        f.write(plstring)
        print plstring
        f.close()
    
    def _embedConj(self, c):
        return '%s(%s) :- %s' % (self.name, self.getArgumentString(), c)
    
    def _getConjPLString(self, c):
        tokens = []
        if type(c) is Lit:
            tokens.append(self.getPLLit(c))
        else:
            for lit in c.children:
                if not type(lit) is Lit: raise Exception("wrong format")
                tokens.append(self.getPLLit(lit))
        #tokens.append('.')
        conj = ',\n    '.join(tokens)
        conj += '.'
        return self._embedConj(conj)
        
    def getPLLit(self, lit):
        args = map(self.bindArgument, lit.params)
        s = '%s(%s)' % (lit.predName, ', '.join(args))
        if lit.negated:
            s = 'not(%s)' % s
        return s
    
    def bindArgument(self, arg):
        if arg in self.bindings.keys():
            return '\'' + self.bindings[arg] + '\''
        else:
            return 'VAR' + arg
    
    #def getBestSplit(self, testsofar, domains):
    #besttest = None
    #bestscore = None
    #for p in self.predicates.values():
        #for i, a in enumerate(p.args):
            #if i in p.keys:
                #continue
            #for v in domains[a].values:
                #binding = dict([(a,v)])
                #args = map(lambda x: self.bindArgument(x, binding), p.args)
                #test = '%s(%s)' % (p.name, ','.join(args))
                #print 'testing', test
                #if len(testsofar)>0:
                    #test = ',' + test
                #sol = getAllSolutions(self.prolog, testsofar + test)
                #if len(sol) > bestscore:
                    #besttest = test
                    #bestscore = len(sol)
    #return besttest,domains
                    
    #def bindArgument(self, arg, bindings):
        #if arg in bindings.keys():
            #return '\'' + bindings[arg] + '\''
        #else:
            #return arg
        
if __name__ == '__main__':
        
    rb = RelationalBoosting()
    rb.readModel('senseModel.rtb')    
    rb.readDatabase('testdb.db')
    rb.assertEvidence()
    
    p1 = copy.deepcopy(rb.predicates['hasSense'])
    p2 = copy.deepcopy(rb.predicates['hasRole'])
    p3 = copy.deepcopy(rb.predicates['hasSense'])
    p4 = copy.deepcopy(rb.predicates['hasRole'])
    
    p1.args[0] = 'W1'
    p1.args[1] = 'S1'
    p1.args[2] = 'S'

    p2.args[0] = 'W1'
    p2.args[1] = "R1"
    p2.args[2] = 'S'

    p3.args[0] = 'W2'
    p3.args[1] = 'S2'
    p3.args[2] = 'S'

    p4.args[0] = 'W2'
    p4.args[1] = 'R2'
    p4.args[2] = 'S'
    
    #cons = [[p1,p2,p3,p4]]
    cons = [[p1,p3]]

    #test = [Predicate('hasRole', ['W1',"'Theme'",'S']),Predicate('hasSense', ['W1',"'Water'",'S']),\
        #Predicate('hasRole', ['W2',"'Goal'",'S']),Predicate('hasSense', ['W2',"S2","S"])]
    assignment = {'R1':"'Theme'", 'R2':"'Goal'", 'S1':"'Milk'", 'S2':"'Pot'"}
    #assignment = {}
    test = [0,assignment]
    
    #test = "hasRole(W1,'Theme',S),hasSense(W1,'Water',S)"+\
            #",hasRole(W2,'Goal',S),hasSense(W2,'Milk',S)"
    
    w = [1./len(rb.db)] * len(rb.db)
    rt = RelationalTree('tree1', cons, rb.db, w, rb)
 #   rt.getBestSplit(test)
    
    
    pos = rt.getPositiveAtoms(test)
    
    print '++++++++++++++\npositive atoms:'
    
    for p in pos:
        print p
        
    print '++++++++++++++\nnegative atoms:'
    neg = set(rb.propositions) - pos
    for n in neg:
        print n
    print '++++++++++++++'
    print rt.getScoreForTest(test)
    #print atoms
    #rt = RT("tree1", [[rb.predicates['hasSense'],rb.predicates['hasRole']]], )
        
        
        
        
        
        
        
        
        
        
        
