# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Daniel Nyga (nyga@cs.tum.edu)
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
import sys
from prac.wordnet import WordNet
from mln.mln import MLN
from mln.database import Database

# adapt PYTHONPATH where necessary
PRAC_HOME = os.environ['PRAC_HOME']
prac_module_path = os.path.join(PRAC_HOME, 'pracmodules')

# add 3rd party components to pythonpath, if necessary
dill_path = os.path.join(PRAC_HOME, '3rdparty', 'dill-0.2b1')
if dill_path not in sys.path:
    sys.path.append(dill_path)

from string import whitespace, strip
import dill as pickle
from inference import PRACInference, PRACInferenceStep
import fnmatch
from mln import readMLNFromFile
import yaml
from praclog import logging



class PRAC(object):
    '''
    The PRAC reasoning system.
    '''    
    
    log = logging.getLogger('PRAC')
    
    def __init__(self):
        # read all the manifest files.
        self.moduleManifests = []
        self.moduleManifestByName = {}
        for module_path in os.listdir(prac_module_path):
            if not os.path.isdir(os.path.join(prac_module_path, module_path)):
                continue
            manifest_file_name = os.path.join(prac_module_path, module_path, 'pracmodule.yaml')
            if not os.path.exists(manifest_file_name):
                PRAC.log.warning('No module manifest file in path "%s".' % module_path)
                continue
            manifest_file = open(manifest_file_name, 'r')
            d = os.path.abspath(os.path.join(prac_module_path, module_path, 'src'))
            sys.path.append(d)
            module = PRACModuleManifest.read(manifest_file, self)
            self.moduleManifests.append(module)
            self.moduleManifestByName[module.name] = module
            PRAC.log.info('Read manifest file for module "%s".' % module.name)
        self.moduleByName = {}
        self.modules = []
        #TODO: replace this by real action core definitions
        self.action_cores = ['Flipping', 'Filling', 'BeingLocated']
        self.microtheories = self.action_cores
        self.wordnet = WordNet()
        
    def setKnownConcepts(self, concepts):
        self.wordnet = WordNet(concepts)
    
    def getModuleByName(self, modulename):
        '''
        Returns a loaded and initialized module given by the module name.
        '''
        if not modulename in self.moduleManifestByName:
            raise Exception('No such module: %s' % modulename)
        # lazily load the module
        if not modulename in self.moduleByName:
            module = PRACModule.fromManifest(self.moduleManifestByName[modulename], self)
            module.initialize()
            self.moduleByName[modulename] = module
        return self.moduleByName[modulename]
        
    def getActionCoreTrainingDBs(self, actioncore_name=None):
        '''
        Returns the list of training database file names associated to the
        given action core. Returns all training databases if actioncore_name is None.
        '''
        if actioncore_name is None:
            dbfiles = []
            for root, folder, files in os.walk('models'):
                dbfiles.extend(map(lambda x: os.path.join(root, x), fnmatch.filter(files, '*.db')))
            return dbfiles
        else:
            path = os.path.join('models', actioncore_name, 'db')
            dbfiles = fnmatch.filter(os.listdir(path), '*.db')
            dbfiles = map(lambda x: os.path.join(path, x), dbfiles)
            return dbfiles
    
    def infer(self, modulename, pracinference):
        '''
        Runs module with the given module name on the given PRACInference object.
        '''
        if not modulename in self.moduleManifestByName:
            raise Exception('No such module: %s' % modulename)
        # lazily load the module
        if not modulename in self.moduleByName:
            module = PRACModule.fromManifest(self.moduleManifestByName[modulename], self)
#             module.initialize()
            self.moduleByName[modulename] = module
        module = self.moduleByName[modulename]
        inferenceStep = module.infer(pracinference)
        if inferenceStep is None or type(inferenceStep) != PRACInferenceStep:
            PRAC.log.exception('%s.run() must return a PRACInferenceStep object.' % type(self.moduleByName[modulename]))
        pracinference.inference_steps.append(inferenceStep)
        steps = pracinference.module2infSteps.get(modulename, None)
        if steps is None:
            steps = []
            pracinference.module2infSteps[modulename] = steps
        steps.append(inferenceStep)

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

def DB_TRANSFORM(method):
    '''
    DB_TRANSFORM is a decorator which automates Database duplication with
    adaptation to a new MLN.
    '''
    def wrapper(self, *args, **kwargs):
        db = args[0]
        if not isinstance(db, Database):
            raise Exception('First argument must be a Database object.')
        db_ = db.duplicate(self.mln)
        args = list(args)
        args[0] = db_
        return method(self, *args, **kwargs)
    return wrapper

def PRACPIPE(method):
    def wrapper(self,*args,**kwargs):
        if not hasattr(self, '_initialized'):
            raise Exception('PRACModule subclasses must call their super constructor of PRACModule (%s)' % type(self))
        if not self._initialized:
            self.initialize()
            self._initialized = True
        return method(self,*args,**kwargs)
    return wrapper

class PRACModuleManifest(object):
    '''
    Represents a PRAC module manifest description usually
    stored in a pracmodule.yaml file.
    Members:
    - name:    the name of the module
    - description: the natural-language description of what this module does 
    - module_path: the path where the module is located (for loading local files)
    - is_universal: (bool) if this module is universal of if there is an 
                           individual module for each action core.
    - depends_on: (list) a list of PRAC module names this module depends on.
    '''
    
    
    # YAML tags
    NAME = 'module_name'
    DESCRIPTION = 'description'
    UNIVERSAL = 'is_universal'
    DEPENDENCIES = 'depends_on'
    MAIN_CLASS = 'class_name'
    TRAINABLE = 'trainable'
    
    @staticmethod
    def read(stream, prac):
        '''
        Read a PRAC module manifest (yaml) file and return
        a respective PRACModuleDefinition object.
        '''
        yamlData = yaml.load(stream)
        manifest = PRACModuleManifest()
        (manifest.modulename, manifest.classname) = yamlData[PRACModuleManifest.MAIN_CLASS].split('.')
        manifest.name = yamlData[PRACModuleManifest.NAME]
        manifest.module_path = os.path.join(prac_module_path, manifest.name)
        manifest.description = yamlData[PRACModuleManifest.DESCRIPTION]
        manifest.is_universal = yamlData.get(PRACModuleManifest.UNIVERSAL, False)
        manifest.depends_on = yamlData.get(PRACModuleManifest.DEPENDENCIES, [])
        manifest.is_trainable = yamlData.get(PRACModuleManifest.TRAINABLE, False)
        return manifest

class PRACKnowledgeBase(object):
    '''
    Base class for PRAC microtheroies. Every subclass must ensure
    that it is pickleable.
    '''
    
    def __init__(self, module, name):
        self.module = module
        self.name = name
    
    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['module']
        return odict
    
    def __setstate__(self, d):
        self.__dict__.update(d)
        
class PRACModule(object):
    '''
    Base class for all PRAC reasoning modules. Provides 
    some basic functionality for serializing, deserializing
    and running PRAC modules. Every PRAC module must subclass this.    
    '''
    
    def __init__(self, prac):
        self.prac = prac
        self._initialized = False
    
    def initialize(self):
        '''
        Called after the PRAC module has been loaded. 
        Every PRAC module can do some initialization stuff in here.
        The default implementation does nothing.
        '''
        pass
    
    def shutdown(self):
        '''
        Called when the PRAC reasoning system is to be
        shut down. Here modules can do some cleaning up.
        The default does nothing.
        '''
        pass
    
    @staticmethod
    def fromManifest(manifest, prac):
        '''
        Loads a Module from a given manifest.
        - manifest:    a PRACModuleManifest instance
        '''
        modulename = manifest.modulename
        classname = manifest.classname
        pymod = __import__(modulename)
        clazz = getattr(pymod, classname)
        module = clazz(prac)
        module.manifest = manifest
        module.module_path = manifest.module_path
        module.name = manifest.name
        return module
    
    def load_pracmt(self, mt_name):
        '''
        Loads a pickled PRAC microtheory for the given action core.
        If the specified action core name is None, the module must be universal
        and the pickled module is named after the modules name.
        '''
        binaryFileName = '%s.prac' % mt_name
        return pickle.load(open(os.path.join(prac_module_path, self.name, 'bin', binaryFileName), 'r'))
    
    def save_pracmt(self, prac_mt):
        '''
        Pickles the state of the given microtheory in its binary folder.
        - prac_mt:    instance of a PRACKnowledgeBase
        '''
        binaryFileName = '%s.prac' % prac_mt.name
        binPath = os.path.join(prac_module_path, self.name, 'bin')
        if not os.path.exists(binPath):
            os.mkdir(binPath)
        pickle.dump(prac_mt, open(os.path.join(prac_module_path, self.name, 'bin', binaryFileName), 'w+'))
    
    @PRACPIPE
    def infer(self, pracinference):
        '''
        Run this module. Facts collected so far are stored 
        in the self.pracinference attribute. 
        '''    
        raise NotImplemented()
    
    @PRACPIPE
    def train(self, praclearn):
        '''
        Run the learning process for this module.
        - microtheories:    specifies the microtheories which are to be (re)learned.
        '''
        pass
    
 
    
if __name__ == '__main__':
    '''
    main routine for testing and debugging purposes only!.
    '''
    log = logging.getLogger('PRAC')
#     ac = ActionCore.readFromFile('/home/nyga/code/prac/models/Flipping/actioncore.yaml')
    prac = PRAC()
    infer = PRACInference(prac, ['Flip the pancake around.', 'Put on a plate.'])
    prac.infer('nl_parsing', infer)
    prac.infer('wn_senses', infer)
    for i, db in enumerate(infer.inference_steps[-1].output_dbs):
        log.debug('\nInstruction #%d\n' % (i+1))
        for lit in db.iterGroundLiteralStrings():
            log.debug(lit)
#     mod = PRACModule.fromDefinition(open('/home/nyga/code/prac/pracmodules/nl_parsing/pracmodule.yaml', 'r'))
#     print mod.name
#     print mod.description
#     mod.default_mln.write(sys.stdout)
    
    
    
    
    



