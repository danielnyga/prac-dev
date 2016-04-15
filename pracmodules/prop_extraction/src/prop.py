# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2014 by Mareike Picklum (mareikep@cs.uni-bremen.de)
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
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from pracmln import MLN, Database, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from webmln.gui.pages.utils import get_cond_prob_png


log = logger(__name__)


class PropExtraction(PRACModule):
    def initialize(self):
        pass


    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.info('Running {}'.format(self.name))

        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC PROPERTY EXTRACTION                    |',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print
        print colorize(
            'Inferring most probable ANNOTATION + simultaneous WORD SENSE DISMABIGUATION...',
            (None, 'white', True), True)

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            project = MLNProject.open(projectpath)
        else:
            log.info(
                colorize('Loading Project from params', (None, 'cyan', True),
                         True))
            projectpath = os.path.join(
                params.get('projectpath', None) or self.module_path,
                params.get('project').name)
            project = params.get('project')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = project.mlns.get(project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path],
                        projectpath=projectpath,
                        logic=project.queryconf.get('logic', 'FuzzyLogic'),
                        grammar=project.queryconf.get('grammar',
                                                      'PRACGrammar'))
        known_concepts = mln.domains.get('concept', [])
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # process databases
        for db in dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
            # add cluster to domains
            if 'cluster' in db.domains:
                domains = db.domains['cluster']
                domains.append('cluster')
            else:
                db.domains['cluster'] = ['cluster']

            # infer and update output
            infer = MLNQuery(config=project.queryconf, db=db, mln=mln).run()
            result_db = infer.resultdb

            unified_db = db.union(result_db)

            res_db = self.printResults(unified_db, pracinference.instructions,
                                       wordnet_module)

            inf_step.output_dbs.append(res_db)

        png, ratio = get_cond_prob_png(project.queryconf.get('queries', ''),
                                       dbs, filename=self.name)
        inf_step.png = (png, ratio)
        inf_step.applied_settings = project.queryconf.config
        return inf_step


    def train(self, praclearning):
        print colorize('+===============================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC LEARNING PROPERTIES FROM NL DESCRIPTIONS |',
                       (None, 'green', True), True)
        print colorize('+===============================================+',
                       (None, 'green', True), True)

        mlnName = praclearning.otherParams.get('mln', None)
        mlnLogic = praclearning.otherParams.get('logic', None)

        mln = MLN(mlnfile=mlnName, logic=mlnLogic)
        pracTrainingDBS = praclearning.training_dbs

        if len(pracTrainingDBS) > 1 and type(
                pracTrainingDBS[0]) is str:  # db from file:
            log.info('Learning from db files...')
            inputdbs = Database(mln, dbfile=pracTrainingDBS,
                                ignore_unknown_preds=True)
        elif len(pracTrainingDBS) > 1:
            log.info('Learning from db files (xfold)...')
            inputdbs = pracTrainingDBS
        else:
            log.info('Learning from default db file...')
            dbFile = os.path.join(self.module_path, 'db/ts_stanford_wn_man.db')
            inputdbs = Database(mln, dbfile=dbFile, ignore_unknown_preds=True)

        evidencePreds = ['cop', 'prep_without', 'pobj', 'nsubj', 'is_a',
                         'amod', 'prep_with', 'root', 'has_pos', 'conj_and',
                         'conj_or', 'dobj']
        # trainedMLN = mln.learnWeights(inputdbs, LearningMethods.DCLL, evidencePreds=evidencePreds, gaussianPriorSigma=10, partSize=1, useMultiCPU=1, optimizer='bfgs')
        outputfile = '{}_trained.mln'.format(mlnName.split('.')[0])
        trainedMLN = mln.learnWeights(inputdbs, LearningMethods.DCLL,
                                      evidencePreds=evidencePreds, partSize=1,
                                      gaussianPriorSigma=10, useMultiCPU=0,
                                      optimizer='cg', learningRate=.9)

        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)

        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))
        log.info('Trained MLN saved to {}'.format(outputfile))

        return trainedMLN


    def printResults(self, result_db, instructions, wordnet_module):
        # rewrite result representation from property(...) to color(..), size(..) etc. and print results
        output_db = Database(self.prac.mln, ignore_unknown_preds=True)
        # print annotations found in result db
        for instr in instructions:
            print colorize('Inferred properties for instruction:',
                           (None, 'white', True), True), instr
            print
        for q in result_db.query(
                'property(?word, ?prop) ^ has_sense(?word, ?sense)'):
            if q['?sense'] == 'null': continue
            if q['?prop'] == 'null': continue
            prop = q['?prop']
            word = q['?sense']

            output_db << '{}({}, {})'.format(prop.lower(), 'cluster', word)

            print '{}({}, {})'.format(
                colorize(prop.lower(), (None, 'white', True), True),
                colorize('cluster', (None, 'magenta', True), True),
                colorize(word, (None, 'green', True), True))
        print

        print 'Inferred most probable word senses:'
        for q in result_db.query('has_sense(?w, ?s)'):
            if q['?s'] == 'null': continue
            print '{}:'.format(q['?w'])
            print 'get meanings of word', q['?w'], q['?s']

            wordnet_module.printWordSenses(
                wordnet_module.get_possible_meanings_of_word(result_db,
                                                             q['?w']), q['?s'])
        return output_db
