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
from mln.methods import ParameterLearningMeasures
from prac.wordnet import WordNet
from actioncore.inference import PRACInferenceStep
from wcsp.converter import WCSPConverter

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
        self.mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'action_cores.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
    
    @PRACPIPE
    def infer(self, pracinference):
        log = logging.getLogger('acid')
        log.debug('inference on %s' % self.name)
        
        kb = self.load_pracmt('action_cores')
        wordnet = kb.wordnet
        mln = kb.trained_mln
        inf_step = PRACInferenceStep(pracinference, self)
        for input_db in pracinference.module2infSteps['wn_senses'][0].output_dbs:
             
            db = Database(self.mln)
            for truth, lit in input_db.iterGroundLiteralStrings():
                db.addGroundAtom(lit, truth)
            # add the possible word senses
#             inf_step.input_dbs.append(db)
#             mrf = mln.groundMRF(db)
#             wcsp = WCSPConverter(mrf)
#             result_db = wcsp.getMostProbableWorldDB()
#             result_db.printEvidence()
            db.addGroundAtom('action_core(Filling)')
            inf_step.output_dbs.append(db)
        return inf_step
    
    def train(self, praclearning):
        log = logging.getLogger('prac')
        prac = praclearning.prac
        # get all the relevant training databases
        db_files = prac.getActionCoreTrainingDBs()
        nl_module = prac.getModuleByName('nl_parsing')
        syntactic_preds = nl_module.syntax_mln.predicates
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
        self.trained_mln = mln.learnWeights(training_dbs, ParameterLearningMeasures.BPLL_CG, verbose=True, optimizer='bfgs')
        
        
        
        