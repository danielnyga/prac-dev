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
# add 3rd party components to pythonpath, if necessary
import os
import sys

import locations
import prac_nltk
from prac import locations as praclocations
from pracmln import MLNQuery
from pracmln import praclog
from pracmln.mln import NoSuchPredicateError
from prac.core.inference import PRACInferenceStep
from prac.core.wordnet import WordNet
from pracmln import Database, MLN

from pracmln.mln.database import parse_db
from pracmln.mln.util import mergedom, ifNone
from ConfigParser import ConfigParser

from string import whitespace, strip
import fnmatch
import yaml

prac_nltk.data.path = [praclocations.nltk_data]

modules = ['nltk_2.0b9']
for module in modules:
    path = os.path.join(locations.thirdparty, module)
    if path not in sys.path:
        sys.path.append(path)


logger = praclog.logger(__name__, praclog.INFO)
praclogger = praclog.logger('PRAC', praclog.INFO)
aclogger = praclog.logger('actioncores', praclog.INFO)


class PRACConfig(ConfigParser):
    '''
    Global configuration data structure for PRAC.

    Wraps around a ConfigParser
    '''
    DEFAULTS = {
        'mongodb': {
            'host': 'localhost',
            'port': 27017,
            'user': '',
            'password': ''
        },
        'wordnet': {
            'concepts': '''water.n.06
                         cup.n.01
                         cup.n.02
                         bowl.n.01
                         bowl.n.02
                         milk.n.01'''
        }
    }

    def __init__(self, filename=None):
        ConfigParser.__init__(self, allow_no_value=True)
        for section, values in self.DEFAULTS.iteritems():
            self.add_section(section)
            for key, value in values.iteritems():
                self.set(section, key, value)
        if filename is not None:
            self.read(os.path.join(locations.home, filename))


    def write(self, filename=None):
        '''
        Saves this configuration file to disk.

        :param filename:    the name of the config file.
        '''
        filename = ifNone(filename, 'pracconf')
        filepath = os.path.join(locations.home, filename)
        with open(filepath, 'w+') as f:
            ConfigParser.write(self, f)


    def getlist(self, section, key, separator='\n'):
        return filter(bool, [s.strip() for s in self.get(section, key).split(separator)])


class PRAC(object):
    '''
    The PRAC reasoning system.
    '''

    def __init__(self, configfile='pracconf'):
        # read all the manifest files.
        self.config = PRACConfig(configfile)
        self.actioncores = ActionCore.load(os.path.join(praclocations.home, 'models', 'actioncores.yaml'))
        self._manifests = []
        self._manifests_by_name = {}
        self.logger = praclogger
        self._verbose = 1
        for module_path in os.listdir(praclocations.modules):
            if not os.path.isdir(os.path.join(praclocations.modules, module_path)):
                continue
            manifest_file_name = os.path.join(praclocations.modules, module_path, 'pracmodule.yaml')
            if not os.path.exists(manifest_file_name):
                self.logger.warning('No module manifest file in path "{}".'.format(module_path))
                continue
            manifest_file = open(manifest_file_name, 'r')
            modulessrc = os.path.abspath(os.path.join(praclocations.modules, module_path, 'src'))
            sys.path.append(modulessrc)
            module = PRACModuleManifest.read(manifest_file)
            module.module_path = os.path.join(praclocations.modules, module_path)
            self._manifests.append(module)
            self._manifests_by_name[module.name] = module
            self.logger.debug('Read manifest file for module "{}".'.format(module.name))
        self._module_by_name = {}
        self._modules = []
        # TODO: replace this by real action core definitions
        self.wordnet = WordNet()
        self.mln = self.construct_global_mln()


    def construct_global_mln(self):
        '''
        Reads all predicte declaration MLNs of all modules and returns an MLN
        with all predicates declared.
        '''
        mln = MLN(logic='FuzzyLogic', grammar='PRACGrammar')
        for name, manifest in self._manifests_by_name.iteritems():
            module_path = manifest.module_path
            decl_mlns = manifest.pred_decls
            for mlnfile in decl_mlns:
                tmpmln = MLN(mlnfile=os.path.join(praclocations.modules, module_path, 'mln', mlnfile),
                             logic='FuzzyLogic', grammar='PRACGrammar')
                mln.update_predicates(tmpmln)
        return mln


    def manifest(self, modulename):
        return self._manifests_by_name.get(modulename, None)


    def set_known_concepts(self, concepts):
        self.wordnet = WordNet(concepts)


    def module(self, modulename):
        '''
        Returns a loaded and initialized module given by the module name.
        '''
        if not modulename in self._manifests_by_name:
            raise Exception('No such module: {}'.format(modulename))
        # lazily load the module
        if not modulename in self._module_by_name:
            module = PRACModule.from_manifest(self._manifests_by_name[modulename], self)
            module.initialize()
            self._module_by_name[modulename] = module
        return self._module_by_name[modulename]


    def deinit_modules(self):
        for module in self._module_by_name.values():
            module._initialized = False


    def training_dbs(self, actioncore_name=None):
        '''
        Returns the list of training database file names associated to the
        given action core. Returns all training databases if actioncore_name
        is None.
        '''
        if actioncore_name is None:
            dbfiles = []
            for root, folder, files in os.walk('models'):
                dbfiles.extend(map(lambda x: os.path.join(root, x),
                                   fnmatch.filter(files, '*.db')))
            return dbfiles
        else:
            path = os.path.join('models', actioncore_name, 'db')
            dbfiles = fnmatch.filter(os.listdir(path), '*.db')
            dbfiles = map(lambda x: os.path.join(path, x), dbfiles)
            return dbfiles


    def run(self, pracinfer, inference, *args, **kwargs):
        '''
        Runs module with the given module name on the given PRACInference
        object.
        :param pracinfer:   the PRACInference object.
        :param inference:   any callable object returning a PRACInferenceStep
                            instance.
        :param *args:       the arguments passed to the inference callable.
        :param **kwargs:    the keyword arguments passed to the inference
                            callable.
        '''

        inferenceStep = inference(pracinfer, *args, **kwargs)
        if inferenceStep is None or type(inferenceStep) != PRACInferenceStep:
            praclogger.exception('{}.__call__() must return a PRACInferenceStep object.'.format(inference.name))
        pracinfer.inference_steps.append(inferenceStep)


    @property
    def roles(self):
        return set([r for a in self.actioncores.values() for r in  a.roles])


    @property
    def verbose(self):
        return self._verbose


    @verbose.setter
    def verbose(self, v):
        self._verbose = v


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
    REQUIRED_ROLES = 'required_action_roles'
    ACTION_VERBS = 'action_verbs'
    TEMPLATE_MLN = 'template_mln'
    LEARNED_MLN = 'learned_mln'
    PRED_DECL = 'predicates'
    PLAN = 'cram_plan'


    def __init__(self):
        self._roles = []
        self._req_roles = []


    @property
    def roles(self):
        return self._roles


    @roles.setter
    def roles(self, rs):
        self._roles = rs


    @property
    def required_roles(self):
        return self._req_roles


    @required_roles.setter
    def required_roles(self, rr):
        self._req_roles = rr


    @property
    def learned(self):
        '''
        Returns True if there is a learned MLN available for this action action
        core, or False, otherwise.
        '''
        return self.learned_mln_file is not None and self.learned_mln is not None


    def parameterize_plan(self, **roles):
        if self.plan is None:
            raise Exception('Actioncore {} does not have a plan'.format(self.name))
        return self.plan.format(**roles)


    @staticmethod
    def load(filepath):
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
            aclogger.debug('Read action core: {} (roles: {})'.format(action_core.name, ', '.join(action_core.roles)))
            requiredroles = content.get(ActionCore.REQUIRED_ROLES)
            if requiredroles:
                for rc in requiredroles:
                    action_core.required_roles.append(rc)
            action_core.plan = content.get(ActionCore.PLAN)
            actioncores[action_core.name] = action_core
        return actioncores


    def tofile(self):
        '''
        Write this action core into files.
        '''


def DB_TRANSFORM(method):
    '''
    DB_TRANSFORM is a decorator which automates Database duplication with
    adaptation to a new MLN.
    :param method:  the decorated method to be executed
    :return:        the result of the executed decorated method
    '''

    def wrapper(self, *args, **kwargs):
        db = args[0]
        if not isinstance(db, Database):
            raise Exception('First argument must be a Database object but is {}.'.format(type(db)))
        db_ = db.copy(self.mln)
        args = list(args)
        args[0] = db_
        return method(self, *args, **kwargs)

    return wrapper


def PRACPIPE(method):
    '''
    Decorator to be used for call of the PRACModules. This decorator makes sure
    that each module is initialized before execution and transforms the output
    dbs to be bound to the global MLN.
    :param method:  the PRACModule's __call__ method
    :return:        the InferenceStep object returned by the PRACModule
    '''

    def wrapper(self, *args, **kwargs):
        if not hasattr(self, '_initialized'):
            raise Exception('PRACModule subclasses must call their super constructor of PRACModule ({})'.format(type(self)))
        if not self._initialized:
            self.initialize()
            self._initialized = True
        # transform output databases to be bound to global mln
        step = method(self, *args, **kwargs)
        for i, db in enumerate(step.output_dbs):
            step.output_dbs[i] = db.copy(self.prac.mln)
        return step

    return wrapper


class PRACModuleManifest(object):
    '''
    Represents a PRAC module manifest description usually
    stored in a pracmodule.yaml file.
    Members:
    :member name:         the name of the module
    :member module_path:  the path where the module is located (for loading local
                    files)
    :member description:  the natural-language description of what this module does
    :member depends_on:   (list) a list of PRAC module names this module depends on.
    :member pred_decls:   the path of the file containing the predicate declarations
                    relevant for this module
    :member project_path: the path of the default project
    '''

    # YAML tags
    NAME = 'module_name'
    DESCRIPTION = 'description'
    DEPENDENCIES = 'dependencies'
    MAIN_CLASS = 'class_name'
    PRED_DECLS = 'declarations'
    DEFAULT_PROJECT = 'project'


    @staticmethod
    def read(stream):
        '''
        Read a PRAC module manifest (yaml) file and return an instance of
        a respective PRACModuleManifest object.
        '''
        yamlData = yaml.load(stream)
        manifest = PRACModuleManifest()
        (manifest.modulename, manifest.classname) = yamlData[PRACModuleManifest.MAIN_CLASS].split('.')
        manifest.name = yamlData[PRACModuleManifest.NAME]
        manifest.module_path = os.path.join(praclocations.modules, manifest.name)
        manifest.description = yamlData[PRACModuleManifest.DESCRIPTION]
        manifest.depends_on = yamlData.get(PRACModuleManifest.DEPENDENCIES, [])
        manifest.pred_decls = yamlData.get(PRACModuleManifest.PRED_DECLS, [])
        manifest.project_path = os.path.join(praclocations.modules,
                                             manifest.name,
                                             yamlData.get(PRACModuleManifest.DEFAULT_PROJECT, ''))
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


    def mlnquery(self, config=None, verbose=None, **params):
        '''
        Wrapper for MLNQuery to replace the resultdb of the inference object
        with an MLN Database casted to a PRACDatabase
        :param config:  the configuration file for the inference
        :param verbose: boolean value whether verbosity logs will be
                        printed or not
        :param params:  dictionary of additional settings
        :return:        the inference object
        '''
        infer = MLNQuery(config=config, verbose=verbose, **params).run()
        pracdb = PRACDatabase(self.prac, db=infer.resultdb)
        infer._resultdb = pracdb
        return infer


    @property
    def name(self):
        return self._name


    @name.setter
    def name(self, mname):
        self._name = mname


    @property
    def project_path(self):
        return self._project_path


    @project_path.setter
    def project_path(self, ppath):
        self._project_path = ppath


    @property
    def module_path(self):
        return os.path.join(praclocations.modules, self.name)


    @module_path.setter
    def module_path(self, mpath):
        self._module_path = mpath


    @staticmethod
    def from_manifest(manifest, prac):
        '''
        Loads a Module from a given manifest.
        :param manifest:    a PRACModuleManifest instance
        :param prac:        a PRAC instance
        :return:            an instance of PRACModule
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


    @staticmethod
    def merge_all_domains(pracinference):
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
        :param pracinference:   instance of PRACInference to store facts
        '''
        raise NotImplemented()


    @PRACPIPE
    def train(self, praclearn):
        '''
        Run the learning process for this module.
        :param praclearn:   instance of PRACLearning representing a learning
                            step in PRAC
        '''
        pass


class PRACDatabase(Database):
    '''
    Represents a subclass of the MLN Database and extends it by frequently used
    convenience query methods.
    '''

    def __init__(self, prac, evidence=None, db=None, ignore_unknown_preds=False):
        self.prac = prac

        if evidence: pass
        elif db:
            evidence = db.evidence
        else:
            evidence = {}
        Database.__init__(self, prac.mln, evidence=evidence, dbfile=None,
                          ignore_unknown_preds=ignore_unknown_preds)


    def copy(self, mln=None):
        '''
        Returns a copy of this Database as a PRACDatabase.

        :param mln:     if `mln` is specified, the new MLN will be associated
                        with `mln`, if not, it will be associated with
                        `self.mln`.
        '''

        return PRACDatabase(self.prac, evidence=self.evidence)


    def union(self, dbs, mln=None):
        '''
        Returns a new PRACDatabase consisting of the union of all databases
        given in the arguments.
        '''

        db_ = PRACDatabase(self.prac)
        if type(dbs) is list:
            dbs = [e for d in dbs for e in list(d)] + list(self)
        if isinstance(dbs, Database):
            dbs = list(dbs) + list(self)

        for atom, truth in dbs:
            try:
                db_ << (atom, truth)
            except NoSuchPredicateError:
                pass
        return db_


    def actioncores(self):
        '''
        :return: a generator yielding dictionaries mapping words to action cores
        '''

        for q in self.query('action_core(?w,?ac)'):
            yield {q['?w']: q['?ac']}


    def achieved_by(self, actioncore='?ac1'):
        '''
        :return: a generator yielding dictionaries mapping action cores to other
         action cores which they can be achieved by
        '''

        for q in self.query('achieved_by({},?ac2)'.format(actioncore)):
            if q['?ac2'] == 'Complex': continue
            if actioncore == '?ac1':
                yield {q['?ac1']: q['?ac2']}
            else:
                yield {actioncore: q['?ac2']}


    def roles(self, actioncore):
        '''
        :param actioncore:  the action core whose roles are to be retrieved
        :return:            a generator yielding dictionaries mapping roles to
                            word senses
        '''

        roles = self.prac.actioncores[actioncore].roles

        for role in roles:
            for q in self.query('{}(?w,{}) ^ has_sense(?w,?s)'.format(role, actioncore)):
                yield {role: q['?s']}


if __name__ == '__main__':
    '''
    main routine for testing and debugging purposes only!.
    '''
    prac = PRAC()
    print prac.config.getlist('wordnet', 'concepts')
    
#     print prac.roles
#     print prac.actioncores['Neutralizing'].roles
#     
#     log = logging.getLogger('PRAC')
#     prac = PRAC()
#     infer = PRACInference(prac, ['Flip the pancake around.',
#                                  'Put on a plate.'])
#     prac.infer('nl_parsing', infer)
#     prac.infer('wn_senses', infer)
#     for i, db in enumerate(infer.inference_steps[-1].output_dbs):
#         log.debug('\nInstruction #%d\n' % (i + 1))
#         for lit in db.iterGroundLiteralStrings():
#             log.debug(lit)
