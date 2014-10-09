# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Mareike Picklum (mareikep@cs.tum.edu)
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

possibleProps = ['color','size','shape','isA','hasA']

class NLObjectRecognition(PRACModule):    


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
        
        dkb = params.get('dkb')
        mln = dkb.trainedMLN

        if params.get('kb', None) is None:
            # load the default arguments
            kb = self.load_pracmt('obj_recog')
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        for db in kb.dbs:
            # adding evidence properties to new query db
            propsFound = {}
            for p in possibleProps:
                if not any(x.startswith(p) for x in db.evidence): continue
                for res in db.query('{}(?cluster, ?word)'.format(p)):
                    if res['?cluster'] == 'null': continue
                    if res['?word'] == 'null': continue
                    if not p in propsFound:
                        propsFound[p] = [res['?word']]
                    else:
                        propsFound[p].append(res['?word'])

            # adding word similarities
            db = wordnet_module.add_similarities(db, mln.domains, propsFound)
            # db = wordnet_module.add_similarities(db, mln.domains, [item for sublist in propsFound.values() for item in sublist])
            # res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist] + ['Unknown'])
            db.write(sys.stdout, color=True)
            
            # infer and update output dbs

            inferred_db = mln.infer(evidence_db=db, **kb.query_params)
            inferred_db.write(sys.stdout, color=True)
            inf_step.output_dbs.extend([inferred_db])
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
        useOld =  praclearning.otherParams.get('useOld', 0)
        objName = praclearning.otherParams.get('concept', None)
        dkbPath = '{}/{}.dkb'.format(kbPath, dkbName)

        # create or load knowledge base
        if os.path.isfile(dkbPath):
            log.info('Loading {} ...'.format(dkbPath))
            dkb = self.load_dkb(dkbName)
        else:
            log.info('{} does not exist. Creating...'.format(dkbPath))
            dkb = self.create_dkb(dkbName)


        if useOld:
            mln = readMLNFromFile(os.path.join(self.module_path, 'mln/objInfOld.mln'), logic='FuzzyLogic')
        else:
            mln = dkb.kbmln

        pracTrainingDBS = praclearning.training_dbs

        if len(pracTrainingDBS) >= 1 and type(pracTrainingDBS[0]) is str: # db from file
            inputdbs = readDBFromFile(mln, pracTrainingDBS, ignoreUnknownPredicates=True)
        else: # db from inference result
            inputdbs = pracTrainingDBS
            for db in inputdbs:
                db.addGroundAtom('object(cluster, {})'.format(objName))

        trainingDBS = inputdbs + dkb.dbs
        outputfile = '{}/{}.mln'.format(mlnPath, dkbName)
        
        if useOld:
            updatedDBS = []
            wordnet_module = self.prac.getModuleByName('wn_senses')
            for db in trainingDBS:
                propsFound = {}
                for res in db.query('property(?cluster, ?sense, ?prop)'):
                    if res['?prop'] == 'null': continue
                    if res['?sense'] == 'null': continue
                    if not res['?prop'] in propsFound:
                        propsFound[res['?prop']] = [res['?sense']]
                    else:
                        propsFound[res['?prop']].append(res['?sense'])
                db = wordnet_module.add_senses_and_similiarities_for_words(db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist])
                updatedDBS.append(db)
            trainingDBS = updatedDBS
            trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=['property','similar'], gaussianPriorSigma=10, useMultiCPU=1, optimizer='bfgs')
        else:
            trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=['color','size','shape','isA','hasA'], gaussianPriorSigma=10, useMultiCPU=1, optimizer='bfgs')

        # update dkb
        dkb.trainedMLN = trainedMLN
        dkb.dbs = trainingDBS
        self.save_dkb(dkb, dkb.name)
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))
