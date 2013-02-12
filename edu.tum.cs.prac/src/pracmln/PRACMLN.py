# MARKOV LOGIC NETWORKS FOR PROBABILISTIC ROBOT ACTION CORES
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

from copy import deepcopy
import os
from itertools import product
from MLN.MarkovLogicNetwork import MLN
from grammar import parsePracFormula, TaxLit
from pracmln.PRACDatabase import *
from MLN.methods import InferenceMethods
from MLN.util import mergeDomains, logx, strFormula
import FOL
from nltk.corpus import wordnet as wn
import sys

class PRACMLN(MLN):
    '''
    MLN implementation for PRAC with some convenience methods
    for dynamically generating and manipulating MLNs at runtime.
    '''
    def __init__(self):
        MLN.__init__(self)
        self.materializedTemplates = False
        self.templateIdx2GroupIdx = {}
        self.fixedWeightTemplateIndices = []
        self.graph = None
        self.hard_formulas = []
        self.formula_templates = []
    
    def declarePredicate(self, name, domains, functional=None):
        '''
        Adds a predicate declaration to the MLN:
        - name:        name of the predicate (string)
        - domains:     list of domain names of arguments
        - functional:  indices of args which are functional (optional)
        '''
        if name in self.predicates.keys():
            raise Exception('Predicate cannot be declared twice (%s)' % name)
        assert type(domains) == list
        self.predicates[name] = domains
        if functional is not None:
            func = [i in functional for i, d in enumerate(domains)]
            self.blocks[name] = func
    
    def loadPRACDatabases(self, dbPath):
        '''
        Loads and returns all databases (*.db files) that are located in 
        the given directory and returns the corresponding Database objects.
        - dbPath:     the directory path to look for .db files
        '''
        dbs = []
#        senses = set()
        for dirname, dirnames, filenames in os.walk(dbPath): #@UnusedVariable
            for f in filenames:
                if not f.endswith('.db'):
                    continue
                p = os.path.join(dirname, f)
                print "  reading database %s" % p
                db = PRACDatabase(self, p)
#                senses.update(db.domains['sense'])
                dbs.append(db)
        print "  %d databases read" % len(dbs)
        return dbs
    
    def addFormula(self, formula, weight=0, hard=False, fixWeight=False):
        self._addFormula(formula, self.formulas, weight, hard, fixWeight)
    
    def addFormulaTemplate(self, formula, weight=0, hard=False, fixWeight=False):
        self._addFormula(formula, self.formula_templates, weight, hard, fixWeight)
        
    def _addFormula(self, formula, formulaSet, weight=0, hard=False, fixWeight=False):
        '''
        Adds the given formula to the MLN and extends the respective domains, if necessary.
        - formula:    a FOL.Formula object or a string
        - weight:     weight of the formula
        - hard:       determines if the formula is hard
        - fixWeight:  determines if the weight of the formula is fixed
        '''
        if self.materializedTemplates:
            raise Exception('Formula templates have already been materialized. Adding new formulas is no longer possible.')
        if type(formula) is str:
            formula = parsePracFormula(formula)
        formula.weight = weight
        formula.isHard = hard
        idxTemplate = len(formulaSet)
        if fixWeight:
            self.fixedWeightTemplateIndices.append(idxTemplate)
        formulaSet.append(formula)
        # extend domains
        constants = {}
        formula.getVariables(self, None, constants)
        for domain, constants in constants.iteritems():
            for c in constants: 
                self.addConstant(domain, c)
    
    def duplicate(self):
        '''
        Returns a deep copy of this MLN.
        '''
        mln = PRACMLN()
        mln.domains = deepcopy(self.domains)
        mln.predicates = deepcopy(self.predicates)
        mln.rigidPredicates = deepcopy(self.rigidPredicates)
        mln.formulas = deepcopy(self.formulas)
        mln.blocks = deepcopy(self.blocks)
        mln.domDecls = deepcopy(self.domDecls)
        mln.probreqs = deepcopy(self.probreqs)
        mln.posteriorProbReqs = deepcopy(self.posteriorProbReqs)
        mln.defaultInferenceMethod = self.defaultInferenceMethod
        mln.probabilityFittingInferenceMethod = InferenceMethods.Exact
        mln.probabilityFittingThreshold = self.probabilityFittingThreshold
        mln.probabilityFittingMaxSteps = self.probabilityFittingMaxSteps
        mln.parameterType = self.parameterType
        mln.formulaGroups = deepcopy(self.formulaGroups)
        mln.closedWorldPreds = deepcopy(self.closedWorldPreds)
        mln.learnWtsMode = self.learnWtsMode
        mln.vars = deepcopy(self.vars)
        mln.fixedWeightFormulas = deepcopy(self.fixedWeightFormulas)
        return mln
    
    def expandFormulaTemplates(self, dbs):
        '''
        Resolve 'taxonomy-aware' formulas
        '''
        self.formulas = self.formula_templates
        self.materializeFormulaTemplates(dbs)
        self.domains['sense'] = [str(c) for c in self.graph.traverse()]
        variants = []
        for formula in self.formulas:
            formula.printStructure()
            if not isConjunctionOfLiterals(formula):
                variants.append(formula)
                formula.idxFormula = len(variants)
                continue
            isaPreds, fstVars = getIsaPredicates(formula)
            if len(isaPreds) == 0:
                variants.append(formula)
                formula.idxFormula = len(variants)
                continue
            elif not isConjunctionOfLiterals(formula):
                raise Exception("Taxonomy-aware formulas must be conjunctions of literals")
            print 'expanding'
            isa = isaPreds[0]
            conceptQueue = [self.graph.root]
            while len(conceptQueue) > 0:
                concept = conceptQueue.pop()
                children = list(concept.children)
                truefalseCombinations = product([True, False], repeat=len(children))
                for boolean in truefalseCombinations:
                    query = booleanToConjuncts(boolean, children, fstVars[0])
                    condition = Lit(False, 'is_a', [ fstVars[0], str(concept)])
                    f_query = formula._groundTemplate({})[0]
                    f_cond = formula._groundTemplate({})[0]
                    newChildren = []
                    for c in f_query.children:
                        if type(c) is TaxLit:
                            newChildren.extend(query + [condition])
                        else:
                            newChildren.append(c)
                    f_query.children = newChildren
                    newChildren = []
                    for c in f_cond.children:
                        if type(c) is TaxLit:
                            newChildren.append(condition)
                        else:
                            newChildren.append(c)
                    f_cond.children = newChildren
                    c, d = getConditionalProbability(dbs, f_query, f_cond)
#                    print c, d
#                    print f_query, f_cond
                    if len(boolean) == 0:
                        continue
#                    print boolean, reduce(lambda x, y: x and y, boolean)
                    if not reduce(lambda x, y: x or y, boolean):
                        if float(c+concept.unseenNeg) == 0.0:
                            prob = 0.0
                        else:
                            print str(1), '/', str(d+concept.unseenPos + concept.unseenNeg)
                            prob = float(1)/float(d+concept.unseenPos + concept.unseenNeg)
                    else:
                        print c, d
                        if float(c) == 0.0:
                            prob = 0.0
                        else:
                            i = trueIdx(boolean)
                            if i < 0 :
                                prob = 0.0
                            else:
#                            print str(c + children[i].unseenPos), '/', str(d + concept.unseenPos + concept.unseenNeg)
                                print str(c), '/', str(d)
                                prob = float(c + children[i].unseenPos)/float(d + concept.unseenPos + concept.unseenNeg)
#                            prob = float(c)/float(d)
#                            prob = float(float(c)/float(d))
                    if prob != 1:
                        print logx(prob), strFormula(f_query)
                        f_query.weight = logx(prob)
                        f_query.idxFormula = len(variants)
                        f_query.isHard = False
                        variants.append(f_query)
                        self.fixedWeightFormulas.append(f_query)
                    i = trueIdx(boolean)
                    if i >= 0 and c > 0:
                        conceptQueue.append(children[i])
        self.formulas = variants
        
#        for f in self.fixedWeightFormulas:
#            print f
        
#        self.materializedTemplates = False
        

def trueIdx(v):
    found = -1
    for i, e in enumerate(v):
        if e is True:
            if found != -1: return -1
            found = i
    return found
        
                    
    
#    def expandFormulaTemplates(self, dbs):
#        '''
#        Resolve 'taxonomy-aware' formulas
#        '''
#        self.formulaTemplates = self.formulas
#        self.materialize
#        for idxTemplate, tf in enumerate(self.formulaTemplates):
#            variants = []
#            isaPreds, fstVars = getIsaPredicates(tf)
#            if len(isaPreds) == 0:
#                variants.append(tf)
#            elif not isConjunctionOfLiterals(tf):
#                raise Exception("taxonomy-aware formulas must be conjunctions of literals")
#            
#            for assignment in concepts(self.graph, [], len(isaPreds)):
#                ext_assign = []
#                for i, a in enumerate(assignment):
#                    for var in isaPreds[i]: pass
#                    isaCond = getConditionalIsaFormulas(a, fstVars[i])
##                    isas = []
##                    if len(isa) > 0:
##                        for q,e in zip(isa[0], isa[1]):
##                            f = mergeConjunctions(q,e)
##                            print getConditionalProbability(dbs, q, e), f
##                            isas.append(f)
##                    print 'isaCond:',isaCond
#                    ext_assign.append(isaCond)
##                    print 'ext',ext_assign
#                for assgn in combinations(ext_assign):
#                    substitutes = {}
#                    substitutes2 = {}
#                    for i,subst in enumerate(assgn):
#                        for var in isaPreds[i]: pass
#                        substitutes[var] = mergeConjunctions(assgn[i][0], assgn[i][1])
#                        substitutes2[var] = assgn[i][1]
#                    newFormula = copyAndReplace(tf, substitutes)
#                    condition = copyAndReplace(tf, substitutes2)
#                    oldDomains = self.domains
#                    self.domains = mergeDomains(self.domains, *[db.domains for db in dbs])
#                    newFormula = newFormula[0].getTemplateVariants(self)
#                    condition = condition[0].getTemplateVariants(self)
##                    self.domains = oldDomains
#                    for f, c in zip(newFormula, condition):
##                        print 'query:', f
##                        print 'cond:', c
##                        print getConditionalProbability(dbs, f, c)
#                        f.isHard = False
#                        f.weight = logx(getConditionalProbability(dbs, f, c))
#                        print f.weight, f
#                        idxTemplate = len(self.formulas) 
#                        self.fixedWeightTemplateIndices.append(idxTemplate)
#                        variants.append(f)
#            self.formulas.extend(variants)
#        return variants






def getConditionalProbability(dbs, query, condition):
#    print mergeConjunctions(query, condition)
#    print condition
    qCount = countTrueGroundings(dbs, mergeConjunctions(query, condition))
#    if qCount[0] == 0.0:
#        return 0.0, 0.0
    cCount = countTrueGroundings(dbs, condition)
#    print qCount, cCount
#    return float(qCount[0])/float(cCount[0])
    return qCount[0], cCount[0]

def getConditionalIsaFormulas(concept, var):
    '''
    Returns two lists of formulas representing the conditional
    probability P(f1 | f2), where the first one is f1 and second one is f2.
    '''
    if len(concept.children) == 0:
        return []
    concepts = list(concept.children) + [concept]
    joint = fullJoint(concepts, var, len(concept.children))
    res = []
    for q,c in zip(joint[0], joint[1]):
        res.append([q,c])
    return res
    

def concepts(graph, assignment, length):
    if len(assignment) == length:
        yield assignment
        return
    for e in graph.traverse():
        for assgn in concepts(graph, assignment + [e], length):
            yield assgn

        
def getIsaPredicates(f):
    assert isConjunctionOfLiterals(f)
    queue = [f]
    taxliterals = []
    fstVars = []
    while len(queue) > 0:
        f_ = queue.pop()
        if hasattr(f_, 'children'):
            queue.extend(f.children)
        if type(f_) is TaxLit:
            if len(taxliterals) == 0 or f_.params[1] not in reduce(lambda x,y: x.union(y), [x.keys() for x in taxliterals]):
                taxlits = {}
                taxliterals.append(taxlits)
                taxlits[f_.params[1]] = []
            else:
                for x in taxliterals:
                    for k in x.keys(): pass
                    if k == f.params[1]: taxlits = x
            taxlits[f_.params[1]].append(f_)
            fstVars.append(f_.params[0])
    return taxliterals, fstVars

def replaceIsaPredicates(f, isas, substitute):
    for isa in isas:
        i = f.children.index(isa)
        f.children.remove(isa)
        f.children = f.children[:i] + [substitute] + f.children[i:]

def copyAndReplace(f, substitutes):
    variants = [[]]
    if type(f) is Lit:
        return [Lit(f.negated, f.predName, f.params)]
    # replace the tax literals
    if type(f) is TaxLit:    
        return [substitutes[f.params[1]]]
    for child in f.children:
        childVariants = copyAndReplace(child, substitutes)
        new_variants = []
        for variant in variants:
            for childVariant in childVariants:
                v = list(variant)
                v.append(childVariant)
                new_variants.append(v)
        variants = new_variants
    final_variants = []
    for variant in variants:
        if type(f) == FOL.Exist:
            final_variants.append(FOL.Exist(f.getVariables(), variant[0]))
        else:
            final_variants.append(apply(type(f), (variant,)))

    return final_variants

def traverseFormula(f):
    def dfsEnqueue(self, f, queue):
        return 
    queue = [f]
    while len(queue) > 0:
        n = queue.pop()
        if hasattr(n, 'children'):
            queue.extend(n.children)
        yield n

def fullJoint(atoms, var, combineLength=None):
    if combineLength is None:
        combineLength = len(atoms)
    booleans = product([True, False], repeat=combineLength)
    query = []
    evidence = []
    for b in booleans:
        b = list(b)
#        b.extend([False] * (len(atoms)-combineLength))
        q = booleanToConjunction(b, atoms[:combineLength], var)
        e = booleanToConjunction([False] * (len(atoms)-combineLength), atoms[combineLength:], var)
        query.append(q)
        evidence.append(e)
#    print query,'|',evidence
    return query, evidence

def booleanToConjunction(boolean, atoms, var):
    assert len(boolean) == len(atoms)
    conjuncts = []
    for b, a in zip(boolean, atoms):
        conjuncts.append(Lit(b, 'is_a', [var, str(a)]))
    conj = FOL.Conjunction(conjuncts)
    return conj

def booleanToConjuncts(boolean, atoms, var):
    assert len(boolean) == len(atoms)
    conjuncts = []
    for b, a in zip(boolean, atoms):
        conjuncts.append(Lit(not b, 'is_a', [var, str(a)]))
    return conjuncts

def mergeConjunctions(conj1, conj2):
    return FOL.Conjunction(conj1.children + conj2.children)
    
    