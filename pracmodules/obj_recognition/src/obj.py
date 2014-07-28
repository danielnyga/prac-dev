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
        print 'Using DKB {}:\n'.format(colorize(dkb.name, (None, 'yellow', True), True))
        dkb.kbmln.write(sys.stdout, color=True) # todo remove, debugging only
        print 

        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        mln = dkb.kbmln

        known_concepts = mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        result_dbs = []
        # process databases
        for db in kb.dbs:
            res_db = Database(mln)

            for res in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if res['?prop'] == 'null': continue
                if res['?sense'] == 'null': continue
                atom = 'property({}, {}, {})'.format(res['?cluster'], res['?word'].split('-')[0], res['?prop'])
                res_db.addGroundAtom(atom)
            # res_db.write(sys.stdout,color=True)

            for r_db in res_db.query('object(?cluster, ?concept)'):
                # print annotations found in result db
                if q['?concept'] == 'null': continue
                print 'object({}, {})'.format(q['?cluster'], colorize(q['?concept'], (None, 'white', True), True))
            
            # infer and update output dbs
            inferred_db = list(kb.infer(res_db))
            inf_step.output_dbs.extend(inferred_db)
        return inf_step


    def train(self, praclearning):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| DnD... learning in progress...              |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)

        log = logging.getLogger('prac')
        logging.getLogger().setLevel(logging.DEBUG)
        # prac = praclearning.prac
        
        # get all the relevant training databases
        kb = self.load_pracmt('obj_recog')

        # mln = readMLNFromString(kb.query_mln_str)
        mln = readMLNFromFile(os.path.join(self.module_path, 'mln/objectinference.mln'))

        dbFile = os.path.join(self.module_path, 'db/ts_stanford_wn_man.db')
        outputfile = os.path.join(self.module_path, 'mln/dcll_objectinference_stanford_wn_man.mln')
        inputdbs = readDBFromFile(mln, dbFile)
        
        known_concepts = mln.domains.get('concept', [])
        wordnet_module = self.prac.getModuleByName('wn_senses')
        training_dbs = []
        for db in inputdbs:
            db = wordnet_module.add_senses_and_similiarities_for_concepts(db, known_concepts)
            db.write(sys.stdout, color=True)
            training_dbs.append(db)

        # train parsing mln
        log.info('Starting training with {} databases'.format(len(training_dbs)))
        # trainedMLN = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, partSize=8, gaussianPriorSigma=10, verbose=False, optimizer='bfgs')
        # trainedMLN = mln.learnWeights(training_dbs, LearningMethods.DBPLL_CG, evidencePreds=['is_a'],  partSize=4, verbose=False, optimizer='bfgs')
        trainedMLN = mln.learnWeights(training_dbs, LearningMethods.DCLL, evidencePreds=['is_a', 'has_sense', 'property','has_pos'], partSize=1, verbose=False, optimizer='bfgs')
        trainedMLN.write(file(outputfile, "w"))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| Learnt Formulas:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()

        # annotationKB = AnnotationKB(prac)
        # annotationKB.train(dbs)
        # log.info('Saving KB of type {} in {}'.format(kb.__class__.__name__, 'annotations'))
        # self.save_pracmt(annotationKB,'annotations')




class AnnotationKB(PRACKnowledgeBase):
    '''
    Represents the probabilistic KB for learning and inferring
    the correct annotations.
    '''
    
    def train(self, training_dbs):
        mln = readMLNFromFile(os.path.join(self.module_path, 'mln/parsing.mln'))
        self.dbs = training_dbs
        self.learn_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs')
        outputfile = os.path.join(self.module_path, 'mln/wts_pll_ts_stanford_wn_man.db')
        # self.learn_mln.write(file(outputfile, "w"))