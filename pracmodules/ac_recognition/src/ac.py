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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE
import os
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import StringIO
import sys
from utils import colorize

# mapping from PennTreebank POS tags to NLTK POS Tags
nounTags = ['NN', 'NNS', 'NNP']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
    
class ActionCoreIdentification(PRACModule):
    

    def initialize(self):
        pass
#         self.mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'action_cores.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.debug('inference on %s' % self.name)
        
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING ACTION CORES |', (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable ACTION CORE', (None, 'white', True), True)
        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        mln = kb.query_mln
        known_concepts = mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        for db in kb.dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
#             db.write(sys.stdout, color=True)
#             print '---'
            result_db = list(kb.infer(db))
            inf_step.output_dbs.extend(result_db)
            print
            for r_db in result_db:
                for q in r_db.query('ac_word(?ac)'):
                    if q['?ac'] == 'null': continue
                    print 'Identified Action Core(s):', colorize(q['?ac'], (None, 'white', True), True)
                print
        return inf_step
    
    
    def train(self, praclearning):
        log = logging.getLogger('prac')
        prac = praclearning.prac
        # get all the relevant training databases
        db_files = prac.getActionCoreTrainingDBs()
        nl_module = prac.getModuleByName('nl_parsing')
        syntactic_preds = nl_module.mln.predicates
        log.debug(db_files)
        dbs = filter(lambda x: type(x) is Database, map(lambda name: readDBFromFile(self.mln, name, True), db_files))
        log.debug(dbs)
        new_dbs = []
        training_dbs = []
        known_concepts = []
        log.debug(self.mln.domains)
        for db in dbs:
            if not 'actioncore' in db.domains: continue
            if not 'concept' in db.domains: continue
            for c in db.domains['concept']:
                known_concepts.append(c)
            new_dbs.append(db)
        wordnet = WordNet()
        for db in new_dbs:
            new_db = db.duplicate()
            for sol in db.query('has_sense(?w, ?s) ^ is_a(?s, ?c)'):
                word = sol['?w']
                sense = sol['?s']
                concept = sol['?c']
                synset = wordnet.synset(concept)
                for known_concept in known_concepts:
                    known_synset = wordnet.synset(known_concept)
                    if known_synset is None or synset is None: sim = 0
                    else: sim = wordnet.wup_similarity(synset, known_synset)
                    new_db.addGroundAtom('is_a(%s,%s)' % (sense, known_concept), sim)
            training_dbs.append(new_db)
        log.info('Starting training with %d databases' % len(training_dbs))
        actioncore_KB = ActionCoreKB(self, 'action_cores')
        actioncore_KB.wordnet = wordnet
        actioncore_KB.train(training_dbs)
        self.save_pracmt(actioncore_KB)
        
        
class ActionCoreKB(PRACKnowledgeBase):
    '''
    Represents the probabilistic KB for learning and inferring
    the correct action core.
    '''
    
    def train(self, training_dbs):
        mln = self.module.mln
        self.training_dbs = training_dbs
        self.trained_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs')
        
        
        
        