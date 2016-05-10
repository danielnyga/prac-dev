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
import sys
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from pracmln import MLNQuery, MLN, Database
from pracmln.praclog import logger
from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from pracmln.mln.util import colorize
from pracmln.utils.project import MLNProject


log = logger(__name__)
possibleProps = ['color', 'size', 'shape', 'hypernym',
                 'hasa']  # , 'dimension', 'consistency', 'material']


class NLObjectRecognition(PRACModule):
    def initialize(self):
        pass


    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.info('Running {}'.format(self.name))

        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print
        print colorize(
            'Inferring most probable object based on nl description properties...',
            (None, 'white', True), True)

        # load default project
        projectpath = self.project_path
        project = MLNProject.open(projectpath)

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = project.mlns.get(project.queryconf['mln'], None)
        mln = parse_mln(mlntext,
                        searchpaths=[self.module_path],
                        projectpath=projectpath,
                        logic=project.queryconf.get('logic', 'FuzzyLogic'),
                        grammar=project.queryconf.get('grammar',
                                                      'PRACGrammar'))

        wordnet_module = self.prac.getModuleByName('wn_senses')

        # adding evidence properties to new query db
        for db in dbs:
            # find properties and add word similarities
            log.error(db.domains)
            log.error(mln.domains)
            output_db = wordnet_module.add_similarities(db, mln)
            output_db.write()

            # infer and update output dbs
            infer = MLNQuery(config=project.queryconf,
                             db=output_db,
                             mln=mln).run()
            result_db = infer.resultdb

            inf_step.output_dbs.append(result_db)

        return inf_step


    def train(self, praclearning):
        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| TRAINING KNOWLEDGEBASE...                   |',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)

        mlnName = praclearning.otherParams.get('mln', None)
        mlnLogic = praclearning.otherParams.get('logic', None)
        objName = praclearning.otherParams.get('concept', None)
        onTheFly = praclearning.otherParams.get('onthefly', False)

        mln = MLN(mlnfile=os.path.abspath(mlnName), logic=mlnLogic,
                  grammar='PRACGrammar')

        pracTrainingDBS = praclearning.training_dbs
        trainingDBS = []

        if len(pracTrainingDBS) >= 1 and type(
                pracTrainingDBS[0]) is str:  # db from file
            log.info('Learning from db files...')
            inputdbs = Database.load(mln, dbfile=pracTrainingDBS,
                                     ignore_unknown_preds=True)
            trainingDBS += inputdbs
        elif len(pracTrainingDBS) > 1:
            log.info('Learning from db files (xfold)...')
            trainingDBS = pracTrainingDBS
        else:  # db from inference result
            log.info('Learning from inference result...')
            inputdbs = pracTrainingDBS
            for db in inputdbs:
                db << 'object(cluster, {})'.format(objName)
                trainingDBS.append(db)

        outputfile = '{}_trained.mln'.format(mlnName.split('.')[0])

        # learning mln
        trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL,
                                      evidencePreds=possibleProps, partSize=1,
                                      gaussianPriorSigma=10, useMultiCPU=0,
                                      optimizer='cg', learningRate=0.9)

        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)

        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))

        return trainedMLN
