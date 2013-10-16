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
from string import whitespace, strip
import time
import datetime
import pickle
import sys
nltk.data.path = [os.path.join('.', 'data', 'nltk_data')]

from mln.MarkovLogicNetwork import readMLNFromFile
import yaml
from logic.grammar import parseFormula
from mln.util import strFormula
import logging
import praclog

# core_definitions = os.path.join('models', 'core.yaml')
# action_cores_path = os.path.join('models', 'actioncores.yaml')
# action_cores_probs = os.path.join('models', 'probabilities.yaml')
# action_cores_spat_probs = os.path.join('models', 'spatial_relation.yaml')

prac_module_path = os.path.join('pracmodules')

class PRAC(object):
    '''
    Represents the PRAC knowledge base.
    '''    
    
    def __init__(self):
        
        
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
                                f = parseFormula(fstring)
                                f.weight = formula[fstring]
                                f.isHard = False
                            else:
                                f = parseFormula(formula)
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
            
    def write_spatial(self, actioncore):
        f = open(action_cores_spat_probs, 'w+')
        for ac in self.action_cores.values():
            if ac.name == 'Filling' or ac.name == 'Flipping' :
                f.write('action_core: %s\n' % ac.name)
                f.write('known_concepts: %s\n' % str(list(ac.known_concepts)))
                f.write('weighted_formulas:\n')
                if ac.mln is not None:
                    for formula in ac.learnedMLN_spatial.formulas:
                        if not formula.isHard and abs(formula.weight) <= 1e-6:
                            continue
                        f.write('    - "%s": %f\n' % (strFormula(formula), formula.weight))
                f.write('---\n')
        
            
class ActionRole(object):
    '''
    Represents a deserialized action role.
    '''
    
    __props__ = []
    
    def __init__(self): pass

    def __repr__(self):
        return str(self.name)
    
    def __str__(self):
        return repr(self)

class ActionCore(object):
    '''
    Represents a deserialized action core object with the action core's 
    definitions.
    '''

    # action core properties in the yaml file    
    NAME = 'action_core'
    DEFINITION = 'definition'
    INHERITS_FROM = 'inherits_from'
    ACTION_ROLES = 'action_roles'
    ACTION_VERBS = 'action_verbs'
    TEMPLATE_MLN = 'template_mln'
    LEARNED_MLN = 'learned_mln'
    
    def __init__(self):
        pass
    
    def isLearned(self):
        '''
        Returns True if there is a learned MLN available for this action action
        core, or False, otherwise.
        '''
        return self.learned_mln_file is not None and self.learned_mln is not None
                
    @staticmethod
    def readFromFile(filepath):
        '''
        Deserializes an action core definition from the given file. The file
        must be given in YAML format. Returns an ActionCore object.
        '''
        log = logging.getLogger('PRAC')
        path = os.path.dirname(filepath)
        content = yaml.load(open(filepath))
        action_core = ActionCore()
        action_core.name = strip(content[ActionCore.NAME], whitespace + '"')
        action_core.definition = content[ActionCore.DEFINITION]
        # read the template MLN
        action_core.template_mln_file = content[ActionCore.TEMPLATE_MLN]
        action_core.template_mln = readMLNFromFile(os.path.join(path, action_core.template_mln_file))
        log.info('Read template MLN %s for action core %s' % (action_core.template_mln_file, action_core.name))
        # read the learned MLN, if any
        action_core.learned_mln_file = content.get(ActionCore.LEARNED_MLN, None)
        action_core.learned_mln = None
        if action_core.learned_mln_file is None:
            log.warning('No learned MLN specified for action core %s' % action_core.name)
        elif not os.path.exists(os.path.join(path, action_core.learned_mln_file)):
            log.info('Learned MLN %s doesn\'t exist for action core %s' % (action_core.learned_mln_file, action_core.name))
        else:
            action_core.learned_mln = readMLNFromFile(os.path.join(path, action_core.learned_mln_file))
            log.info('Read learned MLN file %s for action core %s' % (action_core.learned_mln_file, action_core.name))
        # read the action roles        
        actionroles = content[ActionCore.ACTION_ROLES]
        for role in actionroles:
            for name, params in role.iteritems(): break
            actionrole = ActionRole()
            actionrole.name = name
            for param in params:
                for prop, value in param.iteritems(): break
                setattr(actionrole, prop, value)
                if not prop in ActionRole.__props__:
                    ActionRole.__props__.append(prop)
        log.info('Read action core: %s' % action_core.name)
        
        
    def writeToFile(self):
        '''
        Write this action core into files.
        '''

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
    
    def __rshift__(self, other):
        other.pracinference = self.pracinference
        return other.run()


class PRACModule(object):
    '''
    Base class for all PRAC reasoning modules. Provides 
    some basic functionality for serializing, deserializing
    and running PRAC modules. Every PRAC module must subclass this.
    '''
    
    
    NAME = 'module_name'
    DESCRIPTION = 'description'
    UNIVERSAL = 'is_universal'
    DEPENDENCIES = 'depends_on'
    MAIN_CLASS = 'class_name'
    
    def __init__(self, prac_instance):
        self.prac = prac_instance
    
    def initialized(self):
        '''
        Called after the PRAC module has been loaded.
        Every PRAC module can do some initialization stuff in here.
        The default implementation does nothing.
        '''
        pass
    
    @staticmethod
    def fromDefinition(stream, prac):
        '''
        Read a PRAC module definition (yaml) file and return
        an empty PRACModule object.
        '''
        yamlData = yaml.load(stream, prac)
        classname = yamlData[PRACModule.MAIN_CLASS]
        module = eval('%s()' % classname)
        module.name = yamlData[PRACModule.NAME]
        module.description = yamlData[PRACModule.DESCRIPTION]
        module.is_universal = yamlData.get(PRACModule.UNIVERSAL, False)
        module.depends_on = yamlData.get(PRACModule.DEPENDENCIES, [])
        module.default_mln = readMLNFromFile(os.path.join(prac_module_path, 
                                                          module.name, 'mln', 
                                                          '%s.mln' % module.name))
        return module
    
    def loadExecutable(self, action_core_name=None):
        '''
        Loads a pickled PRAC module for the given action core.
        If the specified action core name is None, the module must be universal
        and the pickled module is named after the modules name.
        '''
        if action_core_name is None:
            binaryFileName = '%s.prac' % self.name
        else:
            binaryFileName = '%s.prac' % action_core_name
        return pickle.load(os.path.join(prac_module_path, self.name, 'bin', binaryFileName))
    
    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict[self.prac]
        return odict
    
    def __setstate__(self, d):
        self.__dict__.update(d)
        
    @PRACPIPE
    def run(self):
        '''
        Run this module. Facts collected so far are stored 
        in the self.pracinference attribute. 
        '''    
        raise NotImplemented()
    
    def __rshift__(self, other):
        '''
        Allows concatenated execution of multiple PRAC module in a series
        using the '>>' operator.
        '''
        other.pracinference = self.pracinference
        return other.run()


    
if __name__ == '__main__':
    '''
    main routine for testing and debugging purposes only!.
    '''
    log = logging.getLogger('PRAC')
#     ac = ActionCore.readFromFile('/home/nyga/code/prac/models/Flipping/actioncore.yaml')
    
    mod = PRACModule.fromDefinition(open('/home/nyga/code/prac/pracmodules/nl_parsing/pracmodule.yaml', 'r'))
    print mod.name
    print mod.description
    mod.default_mln.write(sys.stdout)
    
    
    
    
    



