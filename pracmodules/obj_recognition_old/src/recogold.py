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

possibleProps = ['COLOR','SIZE','SHAPE','HYPERNYM','HASA']#,'DIMENSION', 'CONSISTENCY', 'MATERIAL']

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
        
        if params.get('kb', None) is None: # load the default arguments
            log.info('Using default knowledge base')
            kb = self.load_pracmt('default')
            kb.dbs = pracinference.inference_steps[-1].output_dbs
            mln = kb.query_mln
        else: # load arguments from given knowlegebase
            log.info('Using knowledge base from params')
            kb = params['kb']
            mln = kb.query_mln
        if not hasattr(kb, 'dbs'): # update dbs in knowledge base
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        if params.get('mln', None) is not None: # use mln from params
            mln = params.get('mln')
            kb.query_mln = mln
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # adding evidence properties to new query db
        for db in kb.dbs:
            propsFound = {}

            # process database for new mln and add word similarities
            db.write(sys.stdout, color=True)
            output_db = self.processDB(db, mln, propsFound)
            output_db = wordnet_module.add_similarities_old(output_db, mln.domains, propsFound)
            output_db.write(sys.stdout, color=True)

            # infer and update output dbs_old
            inferred_db = list(kb.infer(output_db))
            inf_step.output_dbs.extend(inferred_db)

        return inf_step

    def train(self, praclearning):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| TRAINING KNOWLEDGEBASE...                   |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)

        log = logging.getLogger(self.name)

        mlnName = praclearning.otherParams.get('mln', None) #.split('/')[-1]
        mlnLogic =  praclearning.otherParams.get('logic', None)
        objName = praclearning.otherParams.get('concept', None)

        mln = readMLNFromFile(mlnName, logic=mlnLogic)
        pracTrainingDBS = praclearning.training_dbs
        trainingDBS = []

        if len(pracTrainingDBS) >= 1 and type(pracTrainingDBS[0]) is str: # db from file
            inputdbs = readDBFromFile(readMLNFromFile(os.path.join(self.module_path, '../obj_recognition/mln/predicates.mln'), logic='FuzzyLogic'), pracTrainingDBS, ignoreUnknownPredicates=True)
            for db in inputdbs:
                db = self.preprocessDB(db, mln, {})
                trainingDBS.append(db)
        elif len(pracTrainingDBS) > 1:
            log.info('Learning from db files (xfold)...')
            for db in pracTrainingDBS:
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

        outputfile = '{}_trained.mln'.format(mlnName.split('.')[0])
        trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=['prop'], partSize=1, gaussianPriorSigma=10, useMultiCPU=0, optimizer='cg', learningRate=0.9)

        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.write(file(outputfile, "w"))

        return trainedMLN


    # rewrite database to match old predicate declaration (when training from database file)
    def preprocessDB(self, db, mln, propsFound):
        output_db = Database(mln)
        for q in db.query('object(?cluster, ?objName)'):
            objName = q['?objName']
            cluster = q['?cluster']
            for prop in possibleProps:
                if any(ek.startswith(prop.lower()) for ek in db.evidence):
                    for p in db.query('object({}, {}) ^ {}({}, ?sense)'.format(cluster, objName, prop.lower(),cluster)):
                        if p['?sense'] == 'null': continue
                        word = p['?sense']
                        if not prop in propsFound:
                            propsFound[prop] = [word]
                        else:
                            propsFound[prop].append(word)

                        output_db.addGroundAtom('prop({}, {}, {})'.format(cluster, word, prop))
            output_db.addGroundAtom('object({}, {})'.format(cluster, objName))
        return output_db


    # rewrite evidence to match old predicate declaration. 
    # extract found properties from evidence db. used for adding similarities
    def processDB(self, db, mln, propsFound):
        output_db = Database(mln)
        print db.evidence
        for prop in possibleProps:
            for q in db.query('{}(?cluster, ?word)'.format(prop.lower())):
                if q['?word'] == 'null': continue
                word = q['?word']
                cluster = q['?cluster']
                if not prop in propsFound:
                    propsFound[prop] = [word]
                else:
                    propsFound[prop].append(word)
            
                propsFound['cluster'] = cluster
                output_db.addGroundAtom('prop({}, {}, {})'.format(cluster, word, prop))

        return output_db