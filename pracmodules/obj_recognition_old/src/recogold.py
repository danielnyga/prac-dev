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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE, DescriptionKnowledgeBase
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import sys, os
from utils import colorize

possibleProps = {'COLOR': 'color','SIZE':'size','SHAPE':'shape','HYPERNYM':'isA','HASA':'hasA'}

class OldNLObjectRecognition(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable object based on nl description properties...', (None, 'white', True), True)
        
        dkb = params.get('dkb',self.load_dkb('micro'))
        mln = dkb.trainedMLN

        if params.get('kb', None) is None:
            # load the default arguments
            kb = self.load_pracmt('default')
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

        kb.query_mln = mln
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # adding evidence properties to new query db
        for db in kb.dbs:
            propsFound = {}

            # process database for new mln and add word similarities
            output_db = self.processDB(db, mln, propsFound)
            print propsFound
            output_db = wordnet_module.add_similarities_old(output_db, mln.domains, propsFound)
            output_db.write(sys.stdout, color=True)
            log.info(mln.domains)

            # infer and update output dbs_old
            inferred_db = list(kb.infer(output_db))
            inf_step.output_dbs.extend(inferred_db)

        return inf_step

    # TODO: learn incrementally
    def train(self, praclearning):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| TRAINING KNOWLEDGEBASE...                   |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)

        mlnPath = os.path.join(self.module_path, 'mln')
        kbPath = os.path.join(self.module_path, 'kb')
        log = logging.getLogger(self.name)

        dkbName =  praclearning.otherParams.get('kb', None)
        objName = praclearning.otherParams.get('concept', None)
        dkbPath = '{}/{}.dkb'.format(kbPath, dkbName)

        # create or load knowledge base
        if os.path.isfile(dkbPath):
            log.info('Loading {} ...'.format(dkbPath))
            dkb = self.load_dkb(dkbName)
        else:
            log.info('{} does not exist. Creating...'.format(dkbPath))
            dkb = self.create_dkb(dkbName)

        mln = dkb.kbmln
        pracTrainingDBS = praclearning.training_dbs
        trainingDBS = dkb.dbs

        if len(pracTrainingDBS) >= 1 and type(pracTrainingDBS[0]) is str: # db from file
            inputdbs = readDBFromFile(readMLNFromFile(os.path.join(self.module_path, '../obj_recognition/mln/predicates.mln'), logic='FuzzyLogic'), pracTrainingDBS, ignoreUnknownPredicates=True)
            for db in inputdbs:
                db = self.preprocessDB(db, mln, {})
                trainingDBS.append(db)
        else: # db from inference result
            inputdbs = pracTrainingDBS
            for db in inputdbs:
                propsFound = {}
                db = self.processDB(db, mln, {})
                db.addGroundAtom('object(cluster, {})'.format(objName))
                trainingDBS.append(db)

        # only for debugging, print final training dbs
        for db in trainingDBS:
            db.write(sys.stdout, color=True)
            print 

        outputfile = '{}/{}.mln'.format(mlnPath, dkbName)
        trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=['prop'], gaussianPriorSigma=10, useMultiCPU=1, optimizer='cg')

        # update dkb
        dkb.trainedMLN = trainedMLN
        dkb.dbs = trainingDBS
        self.save_dkb(dkb, dkb.name)
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))


    def preprocessDB(self, db, mln, propsFound):
        output_db = Database(mln)
        for q in db.query('object(?cluster, ?objName)'):
            objName = q['?objName']
            cluster = q['?cluster']
            for prop in possibleProps:
                if any(ek.startswith(possibleProps[prop]) for ek in db.evidence):
                    for p in db.query('object({}, {}) ^ {}({}, ?sense)'.format(cluster, objName, possibleProps[prop],cluster)):
                        if p['?sense'] == 'null': continue
                        word = p['?sense']
                        if not prop in propsFound:
                            propsFound[prop] = [word]
                        else:
                            propsFound[prop].append(word)

                        output_db.addGroundAtom('prop({}, {}, {})'.format(cluster, word, prop))
            output_db.addGroundAtom('object({}, {})'.format(cluster, objName))
        return output_db


    def processDB(self, db, mln, propsFound):
        output_db = Database(mln)
        for q in db.query('property(?word, ?prop) ^ has_sense(?word, ?sense)'):
            print q
            if q['?sense'] == 'null': continue
            if q['?prop'] == 'null': continue
            prop = q['?prop']
            word = q['?sense']
            if not prop in propsFound:
                propsFound[prop] = [word]
            else:
                propsFound[prop].append(word)

            output_db.addGroundAtom('prop({}, {}, {})'.format('cluster', word, prop))
        return output_db