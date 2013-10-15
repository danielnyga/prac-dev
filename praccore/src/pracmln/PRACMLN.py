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
from mln.MarkovLogicNetwork import MLN
from pracmln.PRACDatabase import *
from mln.methods import InferenceMethods

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
            func = [(i in functional) for i, _ in enumerate(domains)]
            self.blocks[name] = func
            
    def addDomainValue(self, domain, value):
        '''
        Appends a new value to the specified domain.
        '''
        dom = self.domains.get(domain, None)
        if dom is None:
            dom = []
            self.domains[domain] = dom
        dom.append(value)
    
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
            formula = parseFormula(formula)
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
  
    
    