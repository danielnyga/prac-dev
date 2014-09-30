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

possibleProps = ['COLOR','SIZE','SHAPE','HYPERNYM','HASA']

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
        
        if params.get('dkb') is not None:
            dkb = params.get('dkb')
        else:
            dkb = self.load_dkb('mini')
        dkb.printDKB()
        mln = dkb.kbmln

        if params.get('kb', None) is None:
            # load the default arguments
            kb = self.load_pracmt('obj_recog')
            dbs = pracinference.inference_steps[-1].output_dbs
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        result_dbs = []
        # process databases
        for db in kb.dbs:
            # adding evidence properties to new query db
            res_db = Database(mln)

            propsFound = {}
            for res in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if res['?prop'] == 'null': continue
                if res['?sense'] == 'null': continue
                if not res['?prop'] in propsFound:
                    propsFound[res['?prop']] = [res['?sense']]
                else:
                    propsFound[res['?prop']].append(res['?sense'])

                gndAtom = 'property({}, {}, {})'.format(res['?cluster'], res['?sense'], res['?prop'])
                res_db.addGroundAtom(gndAtom)

            # for each inferred property, assert all non-matching properties
            for prop in propsFound:
                for word in mln.domains.get('word', []):
                    if not word in propsFound[prop]:
                        gndAtom = '!property({}, {}, {})'.format(res['?cluster'], word, prop)
                        res_db.addGroundAtom(gndAtom)

            # for each NOT inferred property, assume it has value 'Unknown' (which has a similarity of 0.01 to everything)
            # todo: check if this makes much of a difference
            # for prop in set(propsFound.keys()).symmetric_difference(set(possibleProps)):
            #     gndAtom = 'property({}, {}, {})'.format(res['?cluster'], 'Unknown', prop)
            #     res_db.addGroundAtom(gndAtom)

            # adding word similarities
            res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist])
            # res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist] + ['Unknown'])
            # res_db.write(sys.stdout, color=True)
            
            # infer and update output dbs
            # log.info(kb.query_params)
            # res_db.write(sys.stdout, color=True)
            # inferred_db = mln.infer(evidence_db=res_db, groundingMethod='DefaultGroundingFactory',**kb.query_params)
            inferred_db = mln.infer(evidence_db=res_db, **kb.query_params)
            # print colorize('Inferred DB...', (None, 'green', True), True) 
            # inferred_db.write(sys.stdout,color=True)
            inf_step.output_dbs.extend([inferred_db])

            for q in inferred_db.query('object(?cluster, ?concept)'):
                # print annotations found in result db
                if q['?concept'] == 'null': continue
                log.info('Inferred: object({}, {})'.format(q['?cluster'], colorize(q['?concept'], (None, 'white', True), True)))
        return inf_step

    # TODO: learn incrementally
    def train(self, praclearning, inference, dkbName='mini', objName=None):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| TRAINING KNOWLEDGEBASE...                   |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)

        log = logging.getLogger(self.name)
        
        dkbPath = os.path.join(self.module_path, '../../pracmodules/obj_recognition/kb/{}.dkb'.format(dkbName))
        if os.path.isfile(dkbPath):
            dkb = self.load_dkb(dkbName)
        else:
            dkb = self.create_dkb(dkbName)

        mln = dkb.kbmln
        inputdbs = inference.inference_steps[-1].output_dbs
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # # build training set from inference step to learn weights for mln
        training_db = Database(mln)

        propsFound = {}
        propWords = {}
        trainingDBS = []
        for x in inputdbs:
            training_db = Database(mln)
            propsFound = {}

            # adding evidence properties to new query db
            for res in x.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if res['?prop'] == 'null': continue
                if res['?sense'] == 'null': continue
                if not res['?prop'] in propsFound:
                    propsFound[res['?prop']] = [res['?sense']]
                else:
                    propsFound[res['?prop']].append(res['?sense'])

                gndAtom = 'property({}, {}, {})'.format(res['?cluster'], res['?sense'], res['?prop'])
                training_db.addGroundAtom(gndAtom)
                training_db.addGroundAtom('object({}, {})'.format(res['?cluster'], objName))

            training_db = wordnet_module.add_senses_and_similiarities_for_words(training_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist])
            training_db.write(sys.stdout, color=True)
            trainingDBS.append(training_db)
            log.info('')
        
        outputfile = os.path.join(self.module_path, 'mln/{}.mln'.format(dkbName))
        trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=['property','similar'], optimizer='bfgs')
        dkb.kbmln = trainedMLN
        dkb.printDKB()
        self.save_dkb(dkb, dkbName)
        trainedMLN.write(file(outputfile, "w"))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
        sys.exit(0)