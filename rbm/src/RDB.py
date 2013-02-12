'''
RELATIONAL DATABASE

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
import re
import Tools
import copy
from RBMParsing import RBMParser
import random

class Domain:
    '''
    Represents a domain of a predicate argument.
    '''
    def __init__(self):
        self.name = ''
        self.values = []
        self.isKey = False

    def __str__(self):
        return 'dom(%s)' % self.name + '=' + str(self.values)
        
class Predicate(object):
    '''
    Represents a predicate with arguments and argument bindings.
    '''
    def __init__(self):
        self.name = ''
        self.domains = []
        self.args = []
        self.keys = []
        self.neg = False
        self.bindings = {}
        
    def __str__(self):
        res = '%s(%s)' % (self.name, ','.join(['%s=%s'%(a,Tools.get(self.bindings,a)) for a in self.args]))
        if self.neg: res = '!' + res
        return res

    def isKey(self, arg):
        '''
        Checks if the given argument is a database key or not.
        '''
        return self.args.index(arg) in self.keys
        
    def getDomain(self, arg):
        '''
        Returns the domain of the given argument or None if the arg doesn't exists.
        '''
        try:
            return self.domains[self.args.index(arg)]
        except:
            return None
    
    def getFreeVariables(self):
        '''
        Returns the variables that unbound for this predicate
        '''
        return [a for a in self.args if a not in self.bindings.keys()]
    
    def duplicate(self):
        '''
        Returns a copy of this predicate. Common things like the argument domains,
        keys or the like are shared among these predicates, argument bindings
        are individual copies.
        '''
        p = Predicate()
        p.name = self.name
        p.args = copy.deepcopy(self.args)
        p.bindings = copy.deepcopy(self.bindings)
        p.keys = self.keys
        p.domains = self.domains
        p.neg = self.neg
        return p
    
    def getKeyArgs(self):
        '''
        Returns the list of key arguments of this predicate.
        ''' 
        return [a for (i,a) in enumerate(self.args) if i in self.keys]

    def getNonKeyArgs(self):
        '''
        Returns the list of non-key arguments of this predicate.
        '''
        return [x for x in self.args if not self.isKey(x)]
    
    def getArgBinding(self, argName):
        '''
        Returns the value bound to the given argument or None, if the argument is free.
        '''
        return self.bindings[argName]

    def setArgBinding(self, arg, val):
        '''
        Binds the argument arg of this predicate to the given value.
        '''
        try:
            if val in self.getDomain(arg).values:
                self.bindings[arg] = val
        except: pass
        
    def __eq__(self, other):
        ''' 
        Two predicates are regarded as equal if they have the 
        same name, same negation flag and their bindings coincide.
        '''
        if not self.name == other.name or not self.neg == other.neg or not len(self.args) == len(other.args):
            return False
        for i,a in enumerate(self.args):
            if not Tools.get(self.bindings,a) == Tools.get(other.bindings,other.args[i]):
                return False
        return True

class Formula(object):
    '''
    This class implements a pseudo logical formula consisting
    of a set of grounded, ungrounded or partially grounded atoms.
    '''
    
    def __init__(self):
        self.preds = []
    
    def duplicate(self):
        f = Formula()
        for p in self.preds:
            f.preds.append(p.duplicate())
        return f
    
    def getFreeVariables(self):
        '''
        Return all variables that are not bound in this formula.
        '''
        freevars = []
        for p in self.preds:
            freevars.extend(p.getFreeVariables())
        return set(freevars)

    def bindVariable(self, arg, value):
        '''
        Binds the given variable in the formula to a particular value.
        '''
        for p in self.preds:
            p.setArgBinding(arg, value)
    
    def __str__(self):
        return ' ~ '.join([str(p) for p in self.preds])
    
    
class DBSchema:
    def __init__(self, schemaFile=None, formulaFile=None):
        '''
        Loads the database schema from the fie schemaFile and formulaFile
        '''
        self.domains = {}
        self.predicates = {}
        self.formulas = []
        self.groundpreds = []
        if schemaFile is not None:
            self.loadDBSchema(schemaFile)
        if formulaFile is not None:
            self.loadDBFormulas(formulaFile)
        
        
    def loadDBSchema(self, layoutfile):
        '''
        Read a layout file and generate the Relational Database model.
        '''
        self.schemaFile = layoutfile
        f = open(layoutfile, 'r')
        self.modelfile = file
        lines = f.readlines()
        for l in lines:
            try: l = l[:l.index('//')] # remove comments
            except: pass
            if re.match(r'\s+\w*', l): continue # ignore blank lines
            try: # to parse it as a predicate declaration
                tokens = RBMParser.parsePredicateDeclaration(l)
                pargs = tokens[1:]
                name = tokens[0]
                pred = Predicate()
                pred.name = name
                for i, arg in enumerate(pargs):
                    key = False
                    if arg.startswith(':'):
                        pred.keys.append(i)
                        arg = arg[1:]
                        key = True
                    pred.args.append('arg%d' % i)
                    if arg not in self.domains.keys():
                        self.domains[arg] = Domain()
                        self.domains[arg].name = arg
                    self.domains[arg].isKey = key
                    pred.domains.append(self.domains[arg])
                    if name not in self.predicates.keys():
                        self.predicates[name] = pred
            except: pass
            try: # to parse it as a domain specification
                tokens = RBMParser.parseDomain(l)
                try:
                    d = self.domains[tokens[0]]
                except:
                    d = Domain()
                d.values = tokens[1:]
                d.name = tokens[0]
            except: pass
            
    def loadDBFormulas(self, formulafile):
        '''
        Reads the RRBM formulas from the given file.
        '''
        self.formulaFile = formulafile
        f = open(formulafile, 'r')
        self.modelfile = file
        lines = f.readlines()
        for l in lines:
            try: l = l[:l.index('//')] # remove comments
            except: pass
            if re.match(r'\s+\w*', l): continue # ignore blank lines
            try: # to parse it as a formula
                predicateStrings = l.split('~')
                f = Formula()
                for predString in predicateStrings:
                    pred = self.__getPredicateFromString(predString)
                    f.preds.append(pred)
                self.formulas.append(f)
            except: pass
        self.formulas = self.expandFormulas()

    def __getPredicateFromString(self, s):
        '''
        Takes a string representation of a predicate and transforms it into the respective Predicate
        object with arguments and argument bindings.
        '''
        tokens = RBMParser.parsePredicate(s)
        negate = False
        if tokens[0] == '!':
            tokens = tokens[1:]
            negate = True
        name = tokens[0]
        pargs = tokens[1:]
        pred = self.predicates[name].duplicate()
        pred.args = []
        pred.neg = negate
        pred.bindings = {}
        # update the domains (only the key domains)
        for i, val in enumerate(pargs):
            d = self.predicates[name].domains[i]
            argname = val
            if val in d.values and not d.isKey:
                argname = 'arg%d' % i
                pred.args.append(argname)
                pred.bindings[argname] = val
            else:
                pred.args.append(argname)
        return pred
        
    def expandFormulas(self):
        '''
        Performs formula expansion for the predicate arguments declared with the "+" operator
        '''
        new_formulas = []
        for f in self.formulas:
            variables = []
            values = []
            for p in f.preds:
                for a in p.args:
                    if a.startswith('+') and a not in variables:
                        variables.append(a)
                        values.append(p.getDomain(a).values)
            combinations = Tools.combine(values)
            for c in combinations:
                f_ = f.duplicate()
                for i,v in enumerate(variables):
                    f_.bindVariable(v,c[i])
                new_formulas.append(f_)
        return new_formulas
        
    def loadDatabase(self, dbfile):
        '''
        Loads a database file and returns an RDB object representing the database
        '''
        db = DBSchema(schemaFile=self.schemaFile,formulaFile=self.formulaFile)
        f = open(dbfile, 'r')
        lines = f.readlines()
        for l in lines:
            try: l = l[:l.index('//')] # remove comments
            except: pass
            if re.match(r'\s+\w*', l): continue # ignore blank lines
            try: # to parse it as a predicate
                tokens = RBMParser.parsePredicate(l)
                negate = False
                if tokens[0] == '!':
                    tokens = tokens[1:]
                    negate = True
                name = tokens[0]
                pargs = tokens[1:]
                pred = db.predicates[name].duplicate()
                db.groundpreds.append(pred)
                pred.args = []
                pred.neg = negate
                pred.bindings = {}
                for i, val in enumerate(pargs):
                    d = db.predicates[name].domains[i]
                    argname = 'arg%d' % i
                    pred.args.append(argname)
                    pred.bindings[argname] = val
                    if pred.isKey(argname) and val not in d.values: # update the domains (only the key domains)
                        d.values.append(val) 
            except: pass
        return RDB(db)
        
class RDB(object):
    '''
    Implements a relational database and in its numeric representation.
    '''
    def __init__(self, db):
        self.predicates = []
        self.keys = []
        for idx,p in enumerate(db.predicates.values()):
            self.predicates.append([idx] + [len(d.values) for d in p.domains])
            self.keys.append([k for k in p.keys])
        self.formulas = []
        self.predIdxMap,self.idxPredMap = self.__makePredicateMappings(db)
        self.valIdxMap,self.idxValMap = self.__makeValueMappings(db)
        for f in db.formulas:
            self.formulas.append(self.convertFormula(f))
        self.atoms = self.groundAtoms()
        self.evidence = []
        for groundpred in db.groundpreds:
            print groundpred
            self.evidence.append(self.convertAtom(groundpred))
    
    def __makePredicateMappings(self, db):
        '''
        Takes a Database object and returns two maps between 
        Predicate objects and their numeric representations.
        '''
        predIdxMap = {}
        idxPredMap = {}
        for idx,p in enumerate(db.predicates.values()):
            predIdxMap[p.name] = idx
            idxPredMap[idx] = p.name
        return predIdxMap, idxPredMap 

    def __makeValueMappings(self, db):
        '''
        Takes a Database object and returns two maps between domain values to their numeric representation.
        '''
        valIdxMap = {}
        idxValMap = {}
        for i_pred,predicate in enumerate(db.predicates.values()):
            for i_arg, arg in enumerate(predicate.args):
                for i_val,val in enumerate(predicate.getDomain(arg).values):
                    valIdxMap[(predicate.getDomain(arg).name,val)] = i_val
                    idxValMap[(i_pred,i_arg,i_val)] = val
        return valIdxMap,idxValMap

    def groundFormula(self, f):
        '''
        Takes a formula in numeric representation and returns a list of
        all of its groundings.
        '''
        values = []
        gvars = []
        for p in f:
            for i,a in enumerate(p[1:]):
                if a >= 0 or a in gvars: continue
                values.append(range(self.predicates[p[0]][i+1]))
                gvars.append(a)
        combinations = Tools.combine(values)
        formulaGroundings = []
        for c in combinations:
            grounding = []
            for p in f:
                p_ = [p[0]] # this is the predicate "name"
                for i,a in enumerate(p[1:]):
                    if a < 0: p_.append(c[abs(a)-1])
                    else: p_.append(a)
                grounding.append(self.atomIndex(p_))
            formulaGroundings.append(grounding)
        return formulaGroundings
    
    def convertFormula(self,f):
        '''
        Takes a Formula object and returns its numeric representation.
        '''
        f_ = []
        varMap = {}
        maxVar = -1
        for p in f.preds:
            p_ = [self.predIdxMap[p.name]]
            for a in p.args:
                if Tools.get(p.bindings,a) == None:
                    if Tools.get(varMap,a) == None:
                        varMap[a] = maxVar
                        maxVar -= 1
                    p_.append(varMap[a])
                else:
                    p_.append(self.valIdxMap[(p.getDomain(a).name,p.bindings[a])])
            f_.append(p_)
        return f_
    
    def convertAtom(self, a):
        '''
        Takes an atom of type Predicate and returns the corresponding numeric representation.
        '''
        res = [self.predIdxMap[a.name]]
        for arg in a.args:
            res.append(self.valIdxMap[(a.getDomain(arg).name),a.bindings[arg]])
        return res
                
    def groundAtoms(self):
        '''
        Computes all ground atoms.
        '''
        atoms = []
        for p in self.predicates:
            combis = Tools.combine([range(s) for s in p[1:]])
            for c in combis:
                atoms.append([p[0]] + c)
        return atoms
    
    def atomIndex(self,atom):#[0,1,0,2]
        '''
        Computes the index of the given ground atom
        '''
        idx = p = 0
        while p < atom[0]:
            idx += self.__offset(self.predicates[p][1:])
            p += 1
        return idx + self.__atomIndex(atom[1:], self.predicates[atom[0]][1:])
    
    def __atomIndex(self,atom,domsizes):
        '''
        Recursively computes the index of the given ground atom.
        '''
        if len(atom) == 0:
            return 0
        if len(atom) == 1:
            return atom[0]
        return atom[0] * self.__offset(domsizes[1:]) + self.__atomIndex(atom[1:], domsizes[1:])
    
    def __offset(self,domsizes):
        '''
        Recursively computes the offset given by the number of possible
        combinations in domsizes.
        '''
        if len(domsizes) == 0:
            return 1
        return domsizes[0] * self.__offset(domsizes[1:])
    
    def __atomTemplIndex(self,atom,predicates):
        '''
        Recursively computes the index of the given ground atom's template
        '''
        idx = p = 0
        while p < atom[0]:
            idx += self.__offset(predicates[p][1:])
            p += 1
        return idx + self.__atomIndex(atom[1:], predicates[atom[0]][1:])

    def atomTemplate(self, atom):
        '''
        Computes the index of the given ground atom's template.
        '''
        templ = copy.copy(self.predicates)
        for i_t,t in enumerate(templ):
            templ[i_t] = [e for i_e,e in enumerate(t) if i_e-1 not in self.keys[t[0]]]
        atom_ = [e for i_e,e in enumerate(atom) if i_e-1 not in self.keys[atom[0]]]
        return self.__atomTemplIndex(atom_,templ)
    
    def binaryToAtoms(self, x):
        '''
        Takes as an argument a binary vector and returns the corresponding set of ground atoms.
        '''
        return [self.atoms[i] for i, v in enumerate(x) if v == 1]
    
    def atomsToBinary(self, atoms, infer):
        '''
        Takes a database and returns a binary vector corresponding to the
        facts stored in the database.
        '''
        if infer is True:
            default = 0.
        else:
            default = 0. 
        res = [default] * len(self.atoms)
        for a in atoms:
            res[self.atomIndex(a)] = 1.
        return res
    
    def getReadableName(self, atom):
        '''
        Takes an atom in numeric representation and returns a human readable string
        '''
        return '%s(%s)'%(self.idxPredMap[atom[0]],','.join([self.idxValMap[(atom[0],i,j)] for i,j in enumerate(atom[1:])]))
    
#    def getReadablePredName(self, pred):
#        return '%s(%s)'%(self.idxPredMap[pred[0]],','.join([self.idxValMap[(pred[0],i,j)] for i,j in enumerate(pred[1:])]))
        
if __name__ == '__main__':
    db = DBSchema('../data/language/senses.layout', '../data/language/senses.formulas')
    rdb1 = db.loadDatabase('../data/language/senses_training.db')
    print 
    rdb2 = db.loadDatabase('../data/language/senses_test.db')
    print
#    db.readDatabase('../data/senses_training.db')
#    rdb = RDB(db)
#    
#    testDB = Database()
#    testDB.readModel('../data/senses_model.txt')
#    testDB.readDatabase('../data/senses_test-1.db')
#    testRDB = rdb.loadTestDB(testDB)
#    
#    print testRDB.atoms
#    for a in testRDB.atoms:
#        print testRDB.getReadableName(a)
#    print 'evidence...'
#    evidenceIndices = [testRDB.atomIndex(a) for a in testRDB.evidence]
#    for i,a in enumerate(testRDB.atoms):
#        if i in evidenceIndices:
#            print testRDB.getReadableName(a)
