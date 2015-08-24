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
from pracmln import MLN, Database
from pracmln.mln.methods import LearningMethods
from pracmln.mln.util import colorize
from pracmln.praclog import logger


class PropExtraction(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logger(self.name)
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC PROPERTY EXTRACTION                    |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable ANNOTATION + simultaneous WORD SENSE DISMABIGUATION...', (None, 'white', True), True)
        
        if params.get('kb', None) is None: # load the default arguments
            kb = self.load_pracmt('default')
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

        kb.query_mln = MLN(mlnfile=os.path.join(self.module_path, 'mln/parsing_trained.mln'), logic='FuzzyLogic')

        known_concepts = kb.query_mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        # process databases
        for db in kb.dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
            # add cluster to domains
            if 'cluster' in db.domains:
                domains = db.domains['cluster']
                domains.append('cluster')
            else:
                db.domains['cluster'] = ['cluster']

            # infer and update output
            result_dbs = list(kb.infer(db))
            result_dbs = self.printResults(result_dbs, pracinference.instructions, wordnet_module)

            # inf_step.output_dbs.extend(output_dbs)
            inf_step.output_dbs.extend(result_dbs)
        return inf_step


    def train(self, praclearning):
        print colorize('+===============================================+', (None, 'green', True), True)
        print colorize('| PRAC LEARNING PROPERTIES FROM NL DESCRIPTIONS |', (None, 'green', True), True)
        print colorize('+===============================================+', (None, 'green', True), True)

        log = logger(self.name)

        mlnName =  praclearning.otherParams.get('mln', None)
        mlnLogic =  praclearning.otherParams.get('logic', None)

        mln = MLN(mlnfile=mlnName, logic=mlnLogic)
        pracTrainingDBS = praclearning.training_dbs

        if len(pracTrainingDBS) > 1 and type(pracTrainingDBS[0]) is str: # db from file:
            log.info('Learning from db files...')
            inputdbs = Database(mln, dbfile=pracTrainingDBS, ignoreUnknownPredicates=True)
        elif len(pracTrainingDBS) > 1:
            log.info('Learning from db files (xfold)...')
            inputdbs = pracTrainingDBS
        else:
            log.info('Learning from default db file...')
            dbFile = os.path.join(self.module_path, 'db/ts_stanford_wn_man.db')
            inputdbs = Database(mln, dbfile=dbFile, ignoreUnknownPredicates=True)

        evidencePreds=['cop', 'prep_without','pobj', 'nsubj','is_a','amod','prep_with','root','has_pos','conj_and','conj_or','dobj']
        # trainedMLN = mln.learnWeights(inputdbs, LearningMethods.DCLL, evidencePreds=evidencePreds, gaussianPriorSigma=10, partSize=1, useMultiCPU=1, optimizer='bfgs')
        outputfile = '{}_trained.mln'.format(mlnName.split('.')[0])
        trainedMLN = mln.learnWeights(inputdbs, LearningMethods.DCLL, evidencePreds=evidencePreds, partSize=1, gaussianPriorSigma=10, useMultiCPU=0, optimizer='cg', learningRate=.9)
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))
        log.info('Trained MLN saved to {}'.format(outputfile))

        return trainedMLN


    def printResults(self, result_dbs, instructions, wordnet_module):
        # rewrite result representation from property(...) to color(..), size(..) etc. and print results
        output_dbs = []
        for r_db in result_dbs:
            output_db = Database(MLN(mlnfile=os.path.join(self.module_path, '../obj_recognition/mln/objInf.mln'), logic='FuzzyLogic'))
            # print annotations found in result db
            for instr in instructions:
                print colorize('Inferred properties for instruction:', (None, 'white', True), True), instr
                print
            for q in r_db.query('property(?word, ?prop) ^ has_sense(?word, ?sense)'):
                if q['?sense'] == 'null': continue
                if q['?prop'] == 'null': continue
                prop = q['?prop']
                word = q['?sense']

                output_db.addGroundAtom('{}({}, {})'.format(prop.lower(), 'cluster', word))

                print '{}({}, {})'.format(  colorize(prop.lower(), (None, 'white', True), True), 
                                            colorize('cluster', (None, 'magenta', True), True), 
                                            colorize(word, (None, 'green', True), True))
            print

            print 'Inferred most probable word senses:'
            for q in r_db.query('has_sense(?w, ?s)'):
                if q['?s'] == 'null': continue
                print '{}:'.format(q['?w'])
                print 'get meanings of word',q['?w'], q['?s']

                wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
            output_dbs.append(output_db)
        return output_dbs