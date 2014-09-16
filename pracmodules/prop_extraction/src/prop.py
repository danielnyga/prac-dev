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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import sys, os
from utils import colorize

class PropExtraction(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC PROPERTY EXTRACTION                    |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable ANNOTATION + simultaneous WORD SENSE DISMABIGUATION...', (None, 'white', True), True)
        
        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

        # TODO: Remove when final mln exists
        kb.query_mln = readMLNFromFile(os.path.join(self.module_path, 'mln/dcll_parsing_stanford_wn_man_new_trainingsset_fixed.mln'), logic='FuzzyLogic')

        known_concepts = kb.query_mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        # process databases
        for db in kb.dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
            # db.write(sys.stdout,color=True)
            # add cluster to domains
            if 'cluster' in db.domains:
                domains = db.domains['cluster']
                domains.append('cluster')
            else:
                db.domains['cluster'] = ['cluster']

            # infer and update output dbs
            result_db = list(kb.infer(db))
            inf_step.output_dbs.extend(result_db)

            for r_db in result_db:
                # print annotations found in result db
                for instr in pracinference.instructions:
                    print colorize('Inferred properties for instruction:', (None, 'white', True), True), instr
                    print
                for q in r_db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                    if q['?prop'] == 'null': continue
                    if q['?sense'] == 'null': continue
                    print '{}({}, {}, {})'.format(colorize('property', (None, 'white', True), True), q['?cluster'], q['?sense'], colorize(q['?prop'], (None, 'yellow', True), True))
                print

                print 'Inferred most probable word senses:'
                for q in r_db.query('has_sense(?w, ?s)'):
                    if q['?s'] == 'null': continue
                    print '{}:'.format(q['?w'])
                    print 'get meanings of word',q['?w'], q['?s']
                    wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
        return inf_step


    def train(self, praclearning):
        print colorize('+===============================================+', (None, 'green', True), True)
        print colorize('| PRAC LEARNING PROPERTIES FROM NL DESCRIPTIONS |', (None, 'green', True), True)
        print colorize('+===============================================+', (None, 'green', True), True)

        log = logging.getLogger(self.name)
        # prac = praclearning.prac
        # get all the relevant training databases
        # kb = self.load_pracmt('obj_recog')
        # mln = kb.query_mln
        logging.getLogger().setLevel(logging.DEBUG)
        
        # mln = readMLNFromFile(os.path.join(self.module_path, 'mln/parsing.mln'), logic='FuzzyLogic')
        mln = readMLNFromFile(os.path.join(self.module_path, 'mln/parsing.mln'), logic='FirstOrderLogic')
        dbFile = os.path.join(self.module_path, 'db/ts_stanford_wn_man.db')
        outputfile = os.path.join(self.module_path, 'mln/dcll_parsing_stanford_wn_man_new_trainingsset_fixed.mln')
        inputdbs = readDBFromFile(mln, dbFile, ignoreUnknownPredicates=True)
        
        wordnet_module = self.prac.getModuleByName('wn_senses')
        training_dbs = []

        # train parsing mln
        log.info('Starting training with {} databases'.format(len(inputdbs)))
        # trainedMLN = mln.learnWeights(training_dbs, LearningMethods.CLL, evidencePreds=['prep_without','pobj', 'nsubj','is_a','amod','prep_with','root','has_pos','conj_and','conj_or','dobj'], gaussianPriorSigma=10, partSize=1, optimizer='bfgs')
        trainedMLN = mln.learnWeights(training_dbs, LearningMethods.DCLL, evidencePreds=['cop', 'prep_without','pobj', 'nsubj','is_a','amod','prep_with','root','has_pos','conj_and','conj_or','dobj'], gaussianPriorSigma=10, partSize=1, optimizer='bfgs')
        trainedMLN.write(file(outputfile, "w"))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| Learnt Formulas:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
