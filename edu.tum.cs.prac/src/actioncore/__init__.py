# PROBABILISTIC ROBOT ACTION CORES 
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

import os

import nltk.data
nltk.data.path = [os.path.join('.', 'data', 'nltk_data')]

from MLN.MarkovLogicNetwork import MLN
from FOL import *
from utils.Graph import processNode
from itertools import *
from pracmln.PRACDatabase import countTrueGroundings
from nltk.corpus import wordnet as wn
import yaml
import os
from grammar import parsePracFormula
from pracmln.PRACMLN import *
from mlnLearningTool import MLNLearn
from MLN.methods import ParameterLearningMeasures
from sys import stdout
from MLN.util import strFormula

core_definitions = os.path.join('models', 'core.yaml')
action_cores_path = os.path.join('models', 'actioncores.yaml')
action_cores_probs = os.path.join('models', 'probabilities.yaml')

class PRAC(object):
    
    def __init__(self):
        core_params = yaml.load(open(core_definitions))
        self.predicates = []
        for pred in core_params['predicates']:
            self.predicates.append(pred)
        self.syn_predicates = []
        for pred in core_params['syntactic_predicates']:
            self.syn_predicates.append(pred)

        self.mln = PRACMLN()
        for pred in self.predicates + self.syn_predicates:
            assert(len(pred.keys()) == 1 and len(pred.values()) == 1)
            for name in pred.keys(): pass
            for values in pred.values(): pass
            args = []
            functional = []
            for arg in values:
                flags = set()
                if type(arg) == dict:
                    flags = arg.values()
                    for arg in arg.keys(): pass 
                else:
                    flags = set()
                args.append(arg)
                functional.append('unique' in flags)
            self.mln.predicates[name] = args
            if reduce(lambda x, y: x or y, functional):
                self.mln.blocks[name] = functional

        # load all action cores
        self.action_cores = {}
        ac_params = yaml.load_all(open(action_cores_path))
        self.concepts = set()
           
        for params in ac_params:
            action_core = ActionCore(params, self)
            self.action_cores[action_core.name] = action_core

        if os.path.exists(action_cores_probs):
            ac_probs = yaml.load_all(open(action_cores_probs))
            for ac_prob in ac_probs:
                if ac_prob is None:
                    continue
                name = ac_prob.get('action_core', None)
                acore = self.action_cores.get(name, None)
                acore.learnedMLN = self.mln.duplicate()
                if acore is not None:
                    formulas = ac_prob.get('weighted_formulas', None)
                    if formulas is not None:
                        for formula in formulas:
                            f = None
                            if type(formula) is dict:
                                for fstring in formula.keys(): pass
                                f = parsePracFormula(fstring)
                                f.weight = formula[fstring]
                                f.isHard = False
                            else:
                                f = parsePracFormula(formula)
                                f.isHard = True
                            acore.learnedMLN.addFormula(f, f.weight, f.isHard)
                    known_concepts = ac_prob.get('known_concepts', None)
                    if known_concepts is not None:
                        acore.known_concepts = set(known_concepts)
    
    def write(self):
        f = open(action_cores_probs, 'w+')
        for ac in self.action_cores.values():
            f.write('action_core: %s\n' % ac.name)
            f.write('known_concepts: %s\n' % str(list(ac.known_concepts)))
            f.write('weighted_formulas:\n')
            if ac.mln is not None:
                for formula in ac.learnedMLN.formulas:
                    if not formula.isHard and abs(formula.weight) <= 1e-6:
                        continue
                    f.write('    - "%s": %f\n' % (strFormula(formula), formula.weight))
            f.write('---\n')
            
class ActionRole(object):
    
    def __init__(self, args):
        assert(len(args.keys()) == 1)
        for self.name in args.keys(): pass
        self.definition = None
        for role_params in args.values(): pass
        for param in role_params:
            if param.get('definition', None) is not None:
                self.definition = param.get('definition', None)

    def __repr__(self):
        return str(self.name)
    
    def __str__(self):
        return repr(self)

class ActionCore(object):
    
    def __init__(self, args, prac):
        self.mln = prac.mln.duplicate()
        self.learnedMLN = None
        self.prac = prac
        self.name = args['action_core']
        self.definition = args.get('definition', None)
        self.known_concepts = set()
        self.inherits_from = args.get('inherits_from', [])
        if type(self.inherits_from) is not list:
            self.inherits_from = [self.inherits_from]
        self.action_roles = dict()
        if args.get('action_roles', None) is not None:
            for role in args['action_roles']:
                actionRole =  ActionRole(role)
                self.action_roles[actionRole.name] = actionRole
        self.formula_templates = set()
        if args.get('formula_templates', None) is not None:
            for templ in args.get('formula_templates',[]):
                f = parsePracFormula(templ)
                self.formula_templates.add(f)
                self.mln.addFormulaTemplate(f)
        self.action_verbs = []
        if args.get('action_verbs', None) is not None:
            for av in args.get('action_verbs',[]):
                self.action_verbs.append(av)
        
    def __repr__(self):
        return 'actioncore: %s, roles: %s' % (self.name, ','.join(map(str, self.action_roles)))
    
    def _addFormulas(self):
        for f in self.formula_templates:
            self.mln.addFormula(f)
    
    def _declarePredicates(self):
        for pred in self.prac.predicates + self.prac.syn_predicates:
            assert(len(pred.keys()) == 1 and len(pred.values()) == 1)
            for name in pred.keys(): pass
            for values in pred.values(): pass
            args = []
            functional = []
            for arg in values:
                flags = set()
                if type(arg) == dict:
                    flags = arg.values()
                    for arg in arg.keys(): pass 
                args.append(arg)
                functional.append('unique' in flags)
            self.mln.predicates[name] = args
            if reduce(lambda x, y: x or y, functional):
                self.mln.blocks[name] = functional


def PRACPIPE(method):
    def wrapper(self,*args,**kwargs):
        method(self,*args,**kwargs)
        return self
    return wrapper

class PRACReasoner(object):
    
    def __init__(self, name):
        self.name = name
    
    @PRACPIPE
    def run(self):
        '''
        Perform PRAC reasoning using this reasoner. Facts collected so far are stored 
        in the self.pracinference attribute. 
        '''    
        raise NotImplemented()
    
#    def _addEvidence(self, e, db, mln):
#        if type(e) == str:
#            db.addGroundAtom(e)
#        else:
#            mln.addFormula(e, 0, True)
    
    def __rshift__(self, other):
        other.pracinference = self.pracinference
        return other.run()



    
if __name__ == '__main__':
    prac = PRAC()
    action_core = prac.action_cores['Filling']
#    action_core.mln.write(stdout)
    prac.loadDatabases('/home/nyga/code/prac/models/filling/db')
    prac.learn(prac.dbs, action_core)
    prac.write()
#    print prac.concepts



