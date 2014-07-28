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
from mln.database import Database, readDBFromString, readDBFromFile
from mln.mln import readMLNFromString, MLN
from mln.util import mergeDomains

# adapt PYTHONPATH where necessary
PRAC_HOME = os.environ['PRAC_HOME']
prac_module_path = os.path.join(PRAC_HOME, 'pracmodules')

# add 3rd party components to pythonpath, if necessary
dill_path = os.path.join(PRAC_HOME, '3rdparty', 'dill-0.2b1')
if dill_path not in sys.path:
    sys.path.append(dill_path)

from string import whitespace, strip
import pickle
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
            module.module_path = os.path.join(prac_module_path, module_path)
            self.moduleManifests.append(module)
            self.moduleManifestByName[module.name] = module
            PRAC.log.info('Read manifest file for module "%s".' % module.name)
        self.moduleByName = {}
        self.modules = []
        #TODO: replace this by real action core definitions
        self.action_cores = ['Flipping', 'Filling', 'BeingLocated']
        self.microtheories = self.action_cores
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
                tmpmln = readMLNFromFile(os.path.join(prac_module_path, module_path, 'mln', mlnfile), logic='FuzzyLogic', grammar='PRACGrammar')
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
            PRAC.log.exception('%s.__call__() must return a PRACInferenceStep object.' % type(callable))
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
#     UNIVERSAL = 'is_universal'
    DEPENDENCIES = 'dependencies'
    MAIN_CLASS = 'class_name'
#     TRAINABLE = 'trainable'
    PRED_DECLS = 'declarations'
    
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
        return manifest


class PRACKnowledgeBase(object):
    '''
    Base class for PRAC microtheroies. Every subclass must ensure
    that it is pickleable.
    '''
    
    def __init__(self, prac):
        self.prac = prac
#         self.__prac_mln = prac.mln.duplicate()
        self.learn_mln = prac.mln.duplicate()
        self.learn_mln_str = ''
        self.learn_params = {}
        self.query_mln = prac.mln.duplicate()
        self.query_mln_str = ''
        self.query_params = {}
        self.dbs = []
        self.path = PRAC_HOME
        self.filename = None
        
        
    def set_querymln(self, mln_text, path='.'):
        self.query_mln_str = mln_text
        mln = self.prac.mln.duplicate()
        self.query_mln = readMLNFromString(mln_text, searchPath=path, logic=self.query_params.get('logic', 'FirstOrderLogic'), mln=mln)
#         self.query_mln.write(sys.stdout, color=True)
        
        
    def infer(self, *dbs):
        '''
        Runs MLN inference on the given database using this KB.
        Yields all resulting databases.
        '''
        self.dbs = dbs
        for db in dbs:
            yield self.query_mln.infer(evidence_db=db, **self.query_params)

    
    def __getstate__(self): # do not store
        odict = self.__dict__.copy()
        del odict['dbs']
        del odict['path']
        del odict['prac']
        del odict['query_mln']
        del odict['learn_mln']
        return odict
      
      
    def __setstate__(self, d):
        self.__dict__.update(d)



class DescriptionKnowledgeBase(object):
    '''
    Base class for descriptions of wordnet concepts. 
    '''
    
    def __init__(self):
        self.name = ''
        self.kbmln = MLN(logic='FuzzyLogic', grammar='PRACGrammar')
        self.concepts = [] # List of concept names described in the DKB

    def __getstate__(self): # do not store
        odict = self.__dict__.copy()
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
    
    
    def merge_all_domains(self, pracinference):
        all_dbs = []
        for step in pracinference.inference_steps:
            all_dbs.extend(step.input_dbs)
            all_dbs.extend(step.output_dbs)
        fullDomain =  mergeDomains(*[db.domains for db in all_dbs])
        return fullDomain
        
            
    def create_pracmt(self):
        '''
        Creates a new PRACKnowledgeBase instance initialized by PRAC.
        '''
        kb = PRACKnowledgeBase(self.prac)
    
    
    def load_pracmt(self, mt_name):
        '''
        Loads a pickled PRAC microtheory with given name.
        '''
        binaryFileName = '%s.pracmln' % mt_name
        filepath = os.path.join(self.module_path, 'bin')
        f = open(os.path.join(filepath, binaryFileName), 'r')
        kb = pickle.load(f)
        kb.prac = self.prac
        kb.set_querymln(kb.query_mln_str, os.path.join(self.module_path, 'mln'))
        f.close()
        
        return kb
    
    def save_pracmt(self, prac_mt, name=None):
        '''
        Pickles the state of the given microtheory in its binary folder.
        - prac_mt:    instance of a PRACKnowledgeBase
        '''
        if name is None and not hasattr(prac_mt, 'name'):
            raise Exception('No module name specified.')
        binaryFileName = '%s.pracmln' % (name if name is not None else prac_mt.name)
        binPath = os.path.join(prac_module_path, self.name, 'bin')
        if not os.path.exists(binPath):
            os.mkdir(binPath)
        f = open(os.path.join(prac_module_path, self.name, 'bin', binaryFileName), 'w+')
        pickle.dump(prac_mt, f)
        f.close()

    def create_dkb(self, name):
        '''
        Creates a new DescriptionKnowledgeBase instance
        - name:  The name of the dkb to be created
        '''
        dkb = DescriptionKnowledgeBase()#self.prac)
        dkb.name = name

        return dkb

    def load_dkb(self, dkb_name):
        '''
        Loads a pickled DescriptionKnowledgeBase with given name.
        - dkb_name:  The name of the dkb to be loaded
        '''
        binaryFileName = '{}.dkb'.format(dkb_name)
        filepath = os.path.join(self.module_path, 'kb')
        f = open(os.path.join(filepath, binaryFileName), 'r')
        dkb = pickle.load(f)
        f.close()
        
        return dkb
    
    def save_dkb(self, dkb, name):
        '''
        Pickles the state of the given DescriptionKnowledgeBase in its mln folder.
        - dkb:    instance of a DescriptionKnowledgeBase
        - name:   name of DescriptionKnowledgeBase
        '''
        if name is None and not hasattr(dkb, 'name'):
            raise Exception('No module name specified.')
        
        # update predicates
        tmpmln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'predicates.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
        dkb.kbmln.update_predicates(tmpmln)
        
        kbFileName = '{}.dkb'.format(name)
        kbPath = os.path.join(prac_module_path, self.name, 'kb')

        if not os.path.exists(kbPath):
            os.mkdir(kbPath)
        f = open(os.path.join(kbPath, kbFileName), 'w+')
        pickle.dump(dkb, f)
        f.close()
    
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
        dbs = readDBFromString(self.prac.mln, dbstring, ignoreUnknownPredicates=True)
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
    
    
    
    
    



