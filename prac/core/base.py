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
import logging

import os
import sys
from prac.core.inference import PRACInferenceStep
from prac.core.wordnet import WordNet
from pracmln import Database, MLN

# adapt PYTHONPATH where necessary
import pracmln
from pracmln.mln.base import parse_mln
from pracmln.mln.database import parse_db
from pracmln.mln.util import mergedom, out, stop
from pracmln.mlnquery import MLNQuery
from pracmln.utils.config import query_config_pattern
from pracmln.utils.project import PRACMLNConfig

PRAC_HOME = os.environ['PRAC_HOME']
prac_module_path = os.path.join(PRAC_HOME, 'pracmodules')

# add 3rd party components to pythonpath, if necessary
dill_path = os.path.join(PRAC_HOME, '3rdparty', 'dill-0.2b1')
if dill_path not in sys.path:
    sys.path.append(dill_path)

from string import whitespace, strip
import pickle
import fnmatch
import yaml
from pracmln.utils.latexmath2png import math2png

log = pracmln.praclog.logger(__name__)

class PRAC(object):
    '''
    The PRAC reasoning system.
    '''    
    
    log = logging.getLogger('PRAC')
    
    def __init__(self):
        # read all the manifest files.
        self.actioncores = ActionCore.readFromFile(os.path.join(PRAC_HOME, 'models', 'actioncores.yaml'))
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
            module.module_path = os.path.join(prac_module_path, module_path)
            self.moduleManifests.append(module)
            self.moduleManifestByName[module.name] = module
            PRAC.log.info('Read manifest file for module "%s".' % module.name)
        self.moduleByName = {}
        self.modules = []
        #TODO: replace this by real action core definitions
        self.wordnet = WordNet()
        self.mln = self.readAllMLNDeclarations()

    def readAllMLNDeclarations(self):
        '''
        Reads all predicte declaration MLNs of all modules and returns an MLN
        with all predicates declared.
        '''
        mln = MLN(logic='FuzzyLogic', grammar='PRACGrammar')
        for name, manifest in self.moduleManifestByName.iteritems():
            module_path = manifest.module_path
            decl_mlns = manifest.pred_decls
            for mlnfile in decl_mlns:
                tmpmln = MLN(mlnfile=os.path.join(prac_module_path, module_path, 'mln', mlnfile), logic='FuzzyLogic', grammar='PRACGrammar')
                mln.update_predicates(tmpmln)
        return mln
    
    
    def getManifestByName(self, modulename):
        return self.moduleManifestByName.get(modulename, None)
        
        
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
    
    def uninitAllModules(self):
        for module in self.moduleByName.values():
            module._initialized = False
    
        
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
    
    
    def run(self, pracinfer, inference, *args, **kwargs):
        '''
        Runs module with the given module name on the given PRACInference object.
        - pracinfer:    the PRACInference object.
        - inference:    any callable object returning a PRACInferenceStep instance.
        - *args:        the arguments passed to the inference callable.
        - **kwargs:     the keyword arguments passed to the inference callable.
        '''
        inferenceStep = inference(pracinfer, *args, **kwargs)
        if inferenceStep is None or type(inferenceStep) != PRACInferenceStep:
            # PRAC.log.exception('%s.__call__() must return a PRACInferenceStep object.' % type(callable))
            PRAC.log.exception('%s.__call__() must return a PRACInferenceStep object.' % inference.name)
        pracinfer.inference_steps.append(inferenceStep)


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
    PRED_DECL = 'predicates'
    PLAN = 'cram_plan'
    
    
    def __init__(self):
        self._roles = []
        pass

    @property
    def roles(self):
        return self._roles

    @roles.setter
    def roles(self, rs):
        self._roles = rs

    def isLearned(self):
        '''
        Returns True if there is a learned MLN available for this action action
        core, or False, otherwise.
        '''
        return self.learned_mln_file is not None and self.learned_mln is not None
    
    def parameterize_plan(self, **roles):
        if self.plan is None: 
            raise Exception('Actioncore %s does not have a plan' % self.name)
        return self.plan.format(**roles)
                
    @staticmethod
    def readFromFile(filepath):
        '''
        Deserializes an action core definition from the given file. The file
        must be given in YAML format. Returns an ActionCore object.
        '''
        path = os.path.dirname(filepath)
        alldocs = yaml.load_all(open(filepath))
        actioncores = {}
        for content in alldocs:
            action_core = ActionCore()
            action_core.name = strip(content[ActionCore.NAME], whitespace + '"')
            action_core.definition = content.get(ActionCore.DEFINITION)
            actionroles = content.get(ActionCore.ACTION_ROLES)
            for role in actionroles:
                action_core.roles.append(role)
            log.info('Read action core: %s (roles: %s)' % (action_core.name, ', '.join(action_core.roles)))
            action_core.plan = content.get(ActionCore.PLAN)
            actioncores[action_core.name] = action_core
        return actioncores
        
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
        db_ = db.copy(self.mln)
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
        # transform output databases to be bound to global mln
        step = method(self,*args,**kwargs)
        for i, db in enumerate(step.output_dbs):
            step.output_dbs[i] = db.copy(self.prac.mln)
        return step

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
#     UNIVERSAL = 'is_universal'
    DEPENDENCIES = 'dependencies'
    MAIN_CLASS = 'class_name'
#     TRAINABLE = 'trainable'
    PRED_DECLS = 'declarations'
    DEFAULT_PROJECT = 'project'
    
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
#         manifest.is_universal = yamlData.get(PRACModuleManifest.UNIVERSAL, False)
        manifest.depends_on = yamlData.get(PRACModuleManifest.DEPENDENCIES, [])
    #         manifest.is_trainable = yamlData.get(PRACModuleManifest.TRAINABLE, False)
        manifest.pred_decls = yamlData.get(PRACModuleManifest.PRED_DECLS, [])
        manifest.project_path = os.path.join(prac_module_path, manifest.name, yamlData.get(PRACModuleManifest.DEFAULT_PROJECT, ''))
        return manifest


class PRACModule(object):
    '''
    Base class for all PRAC reasoning modules. Provides 
    some basic functionality for serializing, deserializing
    and running PRAC modules. Every PRAC module must subclass this.    
    '''
    
    def __init__(self, prac):
        self.prac = prac
        self._initialized = False
        self._name = None
        self._project_path = None
        self._module_path = None

    
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


    @property
    def name(self):
        return self._name


    @name.setter
    def name(self, rs):
        self._name = rs


    @property
    def project_path(self):
        return self._project_path


    @project_path.setter
    def project_path(self, rs):
        self._project_path = rs


    @property
    def module_path(self):
        return os.path.join(prac_module_path, self.name)


    @module_path.setter
    def module_path(self, rs):
        self._module_path = rs


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
        module.project_path = manifest.project_path
        return module
    
    
    def merge_all_domains(self, pracinference):
        all_dbs = []
        for step in pracinference.inference_steps:
            all_dbs.extend(step.input_dbs)
            all_dbs.extend(step.output_dbs)
        fullDomain = mergedom(*[db.domains for db in all_dbs])
        return fullDomain
        

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
    
    @PRACPIPE
    def dbfromstring(self, pracinference, dbstring):
        '''
        Parses a database which is given by a string. Returns a PRACInferenceStep
        instance.
        '''
        inf_step = PRACInferenceStep(pracinference, self)
        if len(pracinference.inference_steps) > 0:
            inf_step.input_dbs = list(pracinference.inference_steps[-1].output_dbs)
        dbs = parse_db(self.prac.mln, dbstring, ignore_unknown_preds=True)
        inf_step.output_dbs = dbs
        return inf_step
    
    @PRACPIPE
    def insertdbs(self, pracinference, *dbs):
        '''
        Creates a new inference step with the given dbs as output dbs and
        appends it to the pracinference chain.
        '''
        inf_step = PRACInferenceStep(pracinference, self)
        if len(pracinference.inference_steps) > 0:
            inf_step.input_dbs = list(pracinference.inference_steps[-1].output_dbs)
        inf_step.output_dbs = dbs
        pracinference.inference_steps.append(inf_step)
        return inf_step
    
 
    
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
    
    
    
    
    



